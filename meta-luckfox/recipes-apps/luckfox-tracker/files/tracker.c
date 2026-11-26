#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/videodev2.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <rga/im2d.h>
#include <rga/rga.h>

/* ---------------- Configuration ---------------- */
#define CAM_DEV         "/dev/video11"
#define DMA_HEAP_DEV    "/dev/rk_dma_heap/rk-dma-heap-cma"
#define CAM_W           1920
#define CAM_H           1080
#define CAM_BUFS        3

#define WIDE_W          480
#define WIDE_H          272     // 16-aligned for RGA2 NV12

#define ROI_W           128
#define ROI_H           128

#define UDP_RX_PORT     14602   // Clicks from crsf_mux
#define UDP_TX_PORT     14601   // Target (x,y) to crsf_mux

#define TPL_SIZE        16      // Tight 16x16 core for far targets & fast CPU
#define RAW_PATCH_SIZE  32      // 32x32 buffer for 3-scale probing

static volatile sig_atomic_t g_quit = 0;
static void on_sig(int sig) { (void)sig; g_quit = 1; }

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR);
    return r;
}

/* ---------------- CMA DMA-BUF Allocator + RGA Handle Cache ---------------- */
typedef struct {
    int                 fd;
    size_t              size;
    uint8_t            *map;
    int                 w, h, stride;
    rga_buffer_handle_t rga_h;
} dma_img_t;

static int alloc_cma_nv12(int heap_fd, dma_img_t *img, int w, int h, int stride, size_t min_bytes) {
    memset(img, 0, sizeof(*img));
    img->fd = -1;
    img->w = w; img->h = h; img->stride = stride;
    size_t sz = (size_t)stride * (size_t)h * 3 / 2;
    if (min_bytes > sz) sz = min_bytes;
    sz = (sz + 4095u) & ~4095u;

    struct dma_heap_allocation_data alloc = {
        .len = sz, .fd_flags = O_RDWR | O_CLOEXEC, .heap_flags = 0
    };
    if (xioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &alloc) < 0) return -1;

    uint8_t *p = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, (int)alloc.fd, 0);
    if (p == MAP_FAILED) {
        close((int)alloc.fd);
        return -1;
    }
    img->fd = (int)alloc.fd;
    img->size = sz;
    img->map = p;

    im_handle_param_t hp = {
        .width = (uint32_t)stride,
        .height = (uint32_t)h,
        .format = RK_FORMAT_YCbCr_420_SP
    };
    img->rga_h = importbuffer_fd(img->fd, &hp);
    return 0;
}

static void free_cma_img(dma_img_t *img) {
    if (img->rga_h) releasebuffer_handle(img->rga_h);
    if (img->map && img->map != MAP_FAILED) munmap(img->map, img->size);
    if (img->fd >= 0) close(img->fd);
    memset(img, 0, sizeof(*img));
    img->fd = -1;
}

static inline rga_buffer_t rga_wrap_img(const dma_img_t *img) {
    if (img->rga_h) {
        return wrapbuffer_handle_t(img->rga_h, img->w, img->h, img->stride, img->h, RK_FORMAT_YCbCr_420_SP);
    }
    return wrapbuffer_fd_t(img->fd, img->w, img->h, img->stride, img->h, RK_FORMAT_YCbCr_420_SP);
}

static void dmabuf_sync_cpu(int fd, bool start) {
    struct dma_buf_sync s = {
        .flags = (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END) | DMA_BUF_SYNC_READ
    };
    xioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
}

/* ---------------- Camera + RGA Session ---------------- */
typedef struct {
    int       heap_fd;
    int       v4l2_fd;
    uint32_t  buf_type;
    dma_img_t cam[CAM_BUFS];
    dma_img_t wide[2];
    dma_img_t roi[2];
    int       ping;
    bool      active;
} capture_ctx_t;

static int qbuf_cam(capture_ctx_t *c, uint32_t idx) {
    struct v4l2_buffer b = {0};
    struct v4l2_plane pl = {0};
    b.type = c->buf_type;
    b.memory = V4L2_MEMORY_DMABUF;
    b.index = idx;
    if (c->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        pl.m.fd = c->cam[idx].fd;
        pl.length = (uint32_t)c->cam[idx].size;
        b.m.planes = &pl;
        b.length = 1;
    } else {
        b.m.fd = c->cam[idx].fd;
        b.length = (uint32_t)c->cam[idx].size;
    }
    return xioctl(c->v4l2_fd, VIDIOC_QBUF, &b);
}

static void stop_capture(capture_ctx_t *c) {
    if (!c->active) return;
    if (c->v4l2_fd >= 0) {
        enum v4l2_buf_type t = c->buf_type;
        xioctl(c->v4l2_fd, VIDIOC_STREAMOFF, &t);
        close(c->v4l2_fd);
        c->v4l2_fd = -1;
    }
    for (int i = 0; i < CAM_BUFS; i++) free_cma_img(&c->cam[i]);
    for (int i = 0; i < 2; i++) {
        free_cma_img(&c->wide[i]);
        free_cma_img(&c->roi[i]);
    }
    if (c->heap_fd >= 0) { close(c->heap_fd); c->heap_fd = -1; }
    c->active = false;
    fprintf(stderr, "[tracker] Paused: /dev/video11 closed, CMA freed (0%% load)\n");
}

static int start_capture(capture_ctx_t *c) {
    if (c->active) return 0;
    c->heap_fd = open(DMA_HEAP_DEV, O_RDWR | O_CLOEXEC);
    if (c->heap_fd < 0) { perror("open dma_heap"); return -1; }

    c->v4l2_fd = open(CAM_DEV, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (c->v4l2_fd < 0) { perror("open video11"); goto fail; }

    struct v4l2_capability cap;
    if (xioctl(c->v4l2_fd, VIDIOC_QUERYCAP, &cap) < 0) { perror("QUERYCAP"); goto fail; }
    c->buf_type = (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
                  ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    struct v4l2_format fmt = {0};
    fmt.type = c->buf_type;
    if (c->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        fmt.fmt.pix_mp.width = CAM_W; fmt.fmt.pix_mp.height = CAM_H;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
    } else {
        fmt.fmt.pix.width = CAM_W; fmt.fmt.pix.height = CAM_H;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }
    if (xioctl(c->v4l2_fd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT"); goto fail; }

    int stride = (c->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                 ? (int)fmt.fmt.pix_mp.plane_fmt[0].bytesperline
                 : (int)fmt.fmt.pix.bytesperline;
    size_t drv_size = (c->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                      ? (size_t)fmt.fmt.pix_mp.plane_fmt[0].sizeimage
                      : (size_t)fmt.fmt.pix.sizeimage;
    if (stride < CAM_W) stride = CAM_W;

    size_t aligned_h = (size_t)((CAM_H + 15) & ~15);
    size_t alloc_sz  = (size_t)stride * aligned_h * 3 / 2;
    if (drv_size > alloc_sz) alloc_sz = drv_size;

    for (int i = 0; i < CAM_BUFS; i++) {
        if (alloc_cma_nv12(c->heap_fd, &c->cam[i], CAM_W, CAM_H, stride, alloc_sz) < 0) {
            perror("alloc cam"); goto fail;
        }
    }
    for (int i = 0; i < 2; i++) {
        if (alloc_cma_nv12(c->heap_fd, &c->wide[i], WIDE_W, WIDE_H, WIDE_W, 0) < 0) {
            perror("alloc wide"); goto fail;
        }
        if (alloc_cma_nv12(c->heap_fd, &c->roi[i],  ROI_W,  ROI_H,  ROI_W,  0) < 0) {
            perror("alloc roi"); goto fail;
        }
    }

    struct v4l2_requestbuffers req = { .count = CAM_BUFS, .type = c->buf_type, .memory = V4L2_MEMORY_DMABUF };
    if (xioctl(c->v4l2_fd, VIDIOC_REQBUFS, &req) < 0 || req.count < CAM_BUFS) {
        perror("REQBUFS"); goto fail;
    }
    for (uint32_t i = 0; i < CAM_BUFS; i++) {
        if (qbuf_cam(c, i) < 0) { perror("QBUF"); goto fail; }
    }

    enum v4l2_buf_type t = c->buf_type;
    if (xioctl(c->v4l2_fd, VIDIOC_STREAMON, &t) < 0) { perror("STREAMON"); goto fail; }

    c->ping = 0;
    c->active = true;
    fprintf(stderr, "[tracker] Active: /dev/video11 1080p NV12 + cached RGA handles (stride=%d)\n", stride);
    return 0;

fail:
    c->active = true;
    stop_capture(c);
    return -1;
}

/* ---------------- Vision Primitives (Fast Float + Integer) ---------------- */
static inline float sample_bilinear(const uint8_t *img, int w, int h, float x, float y) {
    if (x < 0.0f) x = 0.0f; else if (x > w - 1.001f) x = w - 1.001f;
    if (y < 0.0f) y = 0.0f; else if (y > h - 1.001f) y = h - 1.001f;
    int ix = (int)x, iy = (int)y;
    float fx = x - ix, fy = y - iy;
    const uint8_t *p = img + iy * w + ix;
    float v0 = p[0] + fx * (p[1] - p[0]);
    float v1 = p[w] + fx * (p[w + 1] - p[w]);
    return v0 + fy * (v1 - v0);
}

static bool lk_track_point(const uint8_t *prev, const uint8_t *curr, int w, int h,
                           float px, float py, float *cx, float *cy, int win_r) {
    float x = *cx, y = *cy;
    if (px < win_r + 1 || px >= w - win_r - 1 || py < win_r + 1 || py >= h - win_r - 1) return false;

    float gxx = 0, gxy = 0, gyy = 0;
    for (int dy = -win_r; dy <= win_r; dy++) {
        const uint8_t *r0 = prev + ((int)py + dy) * w + (int)px;
        for (int dx = -win_r; dx <= win_r; dx++) {
            float ix = 0.5f * ((float)r0[dx + 1] - (float)r0[dx - 1]);
            float iy = 0.5f * ((float)r0[dx + w] - (float)r0[dx - w]);
            gxx += ix * ix; gxy += ix * iy; gyy += iy * iy;
        }
    }
    float det = gxx * gyy - gxy * gxy;
    if (det < 200.0f) return false;

    for (int it = 0; it < 4; it++) {
        if (x < win_r + 1 || x >= w - win_r - 1 || y < win_r + 1 || y >= h - win_r - 1) return false;
        float bx = 0, by = 0;
        for (int dy = -win_r; dy <= win_r; dy++) {
            const uint8_t *r0 = prev + ((int)py + dy) * w + (int)px;
            for (int dx = -win_r; dx <= win_r; dx++) {
                float ix = 0.5f * ((float)r0[dx + 1] - (float)r0[dx - 1]);
                float iy = 0.5f * ((float)r0[dx + w] - (float)r0[dx - w]);
                float it_diff = sample_bilinear(curr, w, h, x + dx, y + dy) - (float)r0[dx];
                bx += ix * it_diff; by += iy * it_diff;
            }
        }
        float vx = (gyy * bx - gxy * by) / det;
        float vy = (gxx * by - gxy * bx) / det;
        x -= vx; y -= vy;
        if (vx * vx + vy * vy < 0.02f) break;
    }
    *cx = x; *cy = y;
    return true;
}

static bool solve_3pt_affine(const float sx[3], const float sy[3],
                             const float dx[3], const float dy[3], float M[6]) {
    float det = sx[0]*(sy[1]-sy[2]) - sy[0]*(sx[1]-sx[2]) + (sx[1]*sy[2] - sx[2]*sy[1]);
    if (fabsf(det) < 1e-3f) return false;
    float id = 1.0f / det;
    float inv[9] = {
        (sy[1] - sy[2]) * id, (sy[2] - sy[0]) * id, (sy[0] - sy[1]) * id,
        (sx[2] - sx[1]) * id, (sx[0] - sx[2]) * id, (sx[1] - sx[0]) * id,
        (sx[1]*sy[2] - sx[2]*sy[1]) * id, (sx[2]*sy[0] - sx[0]*sy[2]) * id, (sx[0]*sy[1] - sx[1]*sy[0]) * id
    };
    M[0] = inv[0]*dx[0] + inv[1]*dx[1] + inv[2]*dx[2];
    M[1] = inv[3]*dx[0] + inv[4]*dx[1] + inv[5]*dx[2];
    M[2] = inv[6]*dx[0] + inv[7]*dx[1] + inv[8]*dx[2];
    M[3] = inv[0]*dy[0] + inv[1]*dy[1] + inv[2]*dy[2];
    M[4] = inv[3]*dy[0] + inv[4]*dy[1] + inv[5]*dy[2];
    M[5] = inv[6]*dy[0] + inv[7]*dy[1] + inv[8]*dy[2];
    return true;
}

static void estimate_wide_gmc(const uint8_t *w_prev, const uint8_t *w_curr,
                              float tgt_x_1080, float tgt_y_1080, float tgt_r_1080,
                              float M_1080[6], float *out_scale) {
    M_1080[0] = 1; M_1080[1] = 0; M_1080[2] = 0;
    M_1080[3] = 0; M_1080[4] = 1; M_1080[5] = 0;
    *out_scale = 1.0f;

    const float sx_scale = (float)CAM_W / WIDE_W;
    const float sy_scale = (float)CAM_H / WIDE_H;

    float px_pts[24], py_pts[24], cx_pts[24], cy_pts[24];
    int n_pts = 0, still_pts = 0;
    int cell_w = WIDE_W / 6, cell_h = WIDE_H / 4;

    for (int gy = 0; gy < 4; gy++) {
        for (int gx = 0; gx < 6; gx++) {
            int x0 = gx * cell_w + 10, x1 = (gx + 1) * cell_w - 10;
            int y0 = gy * cell_h + 10, y1 = (gy + 1) * cell_h - 10;
            int best_x = -1, best_y = -1;
            float best_score = 400.0f;

            for (int y = y0; y < y1; y += 5) {
                for (int x = x0; x < x1; x += 5) {
                    float x1080 = x * sx_scale, y1080 = y * sy_scale;
                    if (fabsf(x1080 - tgt_x_1080) < tgt_r_1080 * 1.5f + 24.0f &&
                        fabsf(y1080 - tgt_y_1080) < tgt_r_1080 * 1.5f + 24.0f) {
                        continue;
                    }
                    const uint8_t *p = w_prev + y * WIDE_W + x;
                    float ix = (float)p[1] - (float)p[-1];
                    float iy = (float)p[WIDE_W] - (float)p[-WIDE_W];
                    float sc = ix * ix + iy * iy;
                    if (sc > best_score) { best_score = sc; best_x = x; best_y = y; }
                }
            }
            if (best_x >= 0) {
                float cx = (float)best_x, cy = (float)best_y;
                if (lk_track_point(w_prev, w_curr, WIDE_W, WIDE_H, (float)best_x, (float)best_y, &cx, &cy, 3)) {
                    float dx = cx - best_x, dy = cy - best_y;
                    float d2 = dx * dx + dy * dy;
                    if (d2 < 32.0f * 32.0f) {
                        if (d2 < 0.45f * 0.45f) still_pts++;
                        px_pts[n_pts] = (float)best_x * sx_scale;
                        py_pts[n_pts] = (float)best_y * sy_scale;
                        cx_pts[n_pts] = cx * sx_scale;
                        cy_pts[n_pts] = cy * sy_scale;
                        n_pts++;
                    }
                }
            }
        }
    }
    if (n_pts < 6 || (still_pts * 2 >= n_pts)) return;

    int best_inliers = -1;
    float best_M[6] = {1, 0, 0, 0, 1, 0};
    uint32_t rng = 0x12345678u;

    for (int it = 0; it < 30; it++) {
        int i0 = (rng = rng * 1664525u + 1013904223u) % n_pts;
        int i1 = (rng = rng * 1664525u + 1013904223u) % n_pts;
        int i2 = (rng = rng * 1664525u + 1013904223u) % n_pts;
        if (i0 == i1 || i1 == i2 || i0 == i2) continue;

        float sx3[3] = { px_pts[i0], px_pts[i1], px_pts[i2] };
        float sy3[3] = { py_pts[i0], py_pts[i1], py_pts[i2] };
        float dx3[3] = { cx_pts[i0], cx_pts[i1], cx_pts[i2] };
        float dy3[3] = { cy_pts[i0], cy_pts[i1], cy_pts[i2] };
        float M[6];
        if (!solve_3pt_affine(sx3, sy3, dx3, dy3, M)) continue;
        float det = M[0] * M[4] - M[1] * M[3];
        if (det < 0.85f || det > 1.18f) continue;

        int inliers = 0;
        for (int k = 0; k < n_pts; k++) {
            float ex = (M[0]*px_pts[k] + M[1]*py_pts[k] + M[2]) - cx_pts[k];
            float ey = (M[3]*px_pts[k] + M[4]*py_pts[k] + M[5]) - cy_pts[k];
            if (ex*ex + ey*ey < 16.0f) inliers++;
        }
        if (inliers > best_inliers) {
            best_inliers = inliers;
            memcpy(best_M, M, sizeof(M));
        }
    }

    if (best_inliers >= 4) {
        memcpy(M_1080, best_M, sizeof(best_M));
        float det = M_1080[0] * M_1080[4] - M_1080[1] * M_1080[3];
        float s = sqrtf(clampf(det, 0.85f, 1.15f));
        *out_scale = (fabsf(s - 1.0f) < 0.008f) ? 1.0f : s;
    }
}

/* ---------------- Weighted 16x16 Template + Fast SAD/NCC ---------------- */
static float g_win_w[TPL_SIZE * TPL_SIZE];
static float g_win_sum = 0.0f;

static void init_template_weights(void) {
    if (g_win_sum > 0.0f) return;
    float sum = 0.0f;
    for (int y = 0; y < TPL_SIZE; y++) {
        for (int x = 0; x < TPL_SIZE; x++) {
            // 2.0x weight on inner 8x8 core for tiny far objects; 1.0x on border
            float w = (x >= 4 && x < 12 && y >= 4 && y < 12) ? 2.0f : 1.0f;
            g_win_w[y * TPL_SIZE + x] = w;
            sum += w;
        }
    }
    g_win_sum = sum;
}

typedef struct {
    float   w_centered[TPL_SIZE * TPL_SIZE]; // w_i * (T_i - T_mean)
    float   norm;
    uint8_t u8[TPL_SIZE * TPL_SIZE];
    int     u8_mean_64;                      // mean of 8x8 subsampled points
    bool    valid;
} fast_tpl_t;

typedef struct {
    bool     tracking;
    bool     has_prev;
    float    cx, cy;
    float    vx, vy;
    float    radius;
    im_rect  prev_crop;

    fast_tpl_t tpl_dyn;
    fast_tpl_t tpl_key;
    uint8_t    dyn_raw[RAW_PATCH_SIZE * RAW_PATCH_SIZE];
    uint8_t    key_raw[RAW_PATCH_SIZE * RAW_PATCH_SIZE];
    int        key_crop_w;
    int        key_scaled_w;
    int        frames_since_key;

    bool     has_chroma;
    float    ref_u, ref_v;
    float    mot_area_ema;
} tracker_state_t;

static void build_tpl_from_u8(const uint8_t patch[TPL_SIZE * TPL_SIZE], fast_tpl_t *out) {
    memcpy(out->u8, patch, TPL_SIZE * TPL_SIZE);
    float wsum = 0.0f;
    for (int i = 0; i < TPL_SIZE * TPL_SIZE; i++) {
        wsum += g_win_w[i] * (float)patch[i];
    }
    float mean = wsum / g_win_sum;
    float n2 = 0.0f;
    for (int i = 0; i < TPL_SIZE * TPL_SIZE; i++) {
        float d = (float)patch[i] - mean;
        out->w_centered[i] = g_win_w[i] * d;
        n2 += g_win_w[i] * d * d;
    }
    out->norm = sqrtf(n2);
    out->valid = (out->norm > 25.0f);

    int s64 = 0;
    for (int y = 0; y < TPL_SIZE; y += 2)
        for (int x = 0; x < TPL_SIZE; x += 2)
            s64 += patch[y * TPL_SIZE + x];
    out->u8_mean_64 = s64 >> 6;
}

static void copy_raw_patch(const uint8_t *roi, float u_c, float v_c, uint8_t out_raw[RAW_PATCH_SIZE * RAW_PATCH_SIZE]) {
    int x0 = (int)roundf(u_c) - RAW_PATCH_SIZE / 2;
    int y0 = (int)roundf(v_c) - RAW_PATCH_SIZE / 2;
    if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
    if (x0 + RAW_PATCH_SIZE > ROI_W) x0 = ROI_W - RAW_PATCH_SIZE;
    if (y0 + RAW_PATCH_SIZE > ROI_H) y0 = ROI_H - RAW_PATCH_SIZE;
    for (int y = 0; y < RAW_PATCH_SIZE; y++)
        memcpy(&out_raw[y * RAW_PATCH_SIZE], &roi[(y0 + y) * ROI_W + x0], RAW_PATCH_SIZE);
}

static bool capture_tpl_roi(const uint8_t *roi, float u_c, float v_c, fast_tpl_t *out) {
    int x0 = (int)roundf(u_c) - TPL_SIZE / 2;
    int y0 = (int)roundf(v_c) - TPL_SIZE / 2;
    if (x0 < 0 || y0 < 0 || x0 + TPL_SIZE > ROI_W || y0 + TPL_SIZE > ROI_H) return false;
    uint8_t tmp[TPL_SIZE * TPL_SIZE];
    for (int y = 0; y < TPL_SIZE; y++)
        memcpy(&tmp[y * TPL_SIZE], &roi[(y0 + y) * ROI_W + x0], TPL_SIZE);
    build_tpl_from_u8(tmp, out);
    return out->valid;
}

static void resample_tpl_from_raw(const uint8_t raw[RAW_PATCH_SIZE * RAW_PATCH_SIZE], float ratio, fast_tpl_t *out) {
    uint8_t tmp[TPL_SIZE * TPL_SIZE];
    float half = RAW_PATCH_SIZE * 0.5f;
    for (int y = 0; y < TPL_SIZE; y++) {
        for (int x = 0; x < TPL_SIZE; x++) {
            float sx = half + (x + 0.5f - TPL_SIZE * 0.5f) * ratio;
            float sy = half + (y + 0.5f - TPL_SIZE * 0.5f) * ratio;
            tmp[y * TPL_SIZE + x] = (uint8_t)clampf(sample_bilinear(raw, RAW_PATCH_SIZE, RAW_PATCH_SIZE, sx, sy), 0.0f, 255.0f);
        }
    }
    build_tpl_from_u8(tmp, out);
}

static float eval_weighted_ncc(const fast_tpl_t *t, const uint8_t *roi, int x0, int y0) {
    float swc = 0.0f, swcc = 0.0f, dot = 0.0f;
    for (int y = 0; y < TPL_SIZE; y++) {
        const uint8_t *row = roi + (y0 + y) * ROI_W + x0;
        const float *wrow = g_win_w + y * TPL_SIZE;
        const float *trow = t->w_centered + y * TPL_SIZE;
        for (int x = 0; x < TPL_SIZE; x++) {
            float c = (float)row[x];
            float wc = wrow[x] * c;
            swc += wc;
            swcc += wc * c;
            dot += c * trow[x];
        }
    }
    float var = swcc - (swc * swc) / g_win_sum;
    if (var < 10.0f) return -1.0f;
    return dot / (sqrtf(var) * t->norm);
}

// Ultra-fast: 8x8 subsampled Zero-Mean Integer SAD -> Top 4 + pred -> exact Weighted NCC + 3x3 fine!
static float search_fast_ncc(const fast_tpl_t *t, const uint8_t *roi,
                             float pred_u, float pred_v, int search_r,
                             float *out_u, float *out_v) {
    if (!t->valid) return 0.0f;
    int cx0 = (int)roundf(pred_u) - TPL_SIZE / 2;
    int cy0 = (int)roundf(pred_v) - TPL_SIZE / 2;

    int top_sad[4] = { 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF };
    int top_x[4]   = { cx0, cx0, cx0, cx0 };
    int top_y[4]   = { cy0, cy0, cy0, cy0 };

    for (int dy = -search_r; dy <= search_r; dy += 2) {
        int y0 = cy0 + dy;
        if (y0 < 0 || y0 + TPL_SIZE > ROI_H) continue;
        for (int dx = -search_r; dx <= search_r; dx += 2) {
            int x0 = cx0 + dx;
            if (x0 < 0 || x0 + TPL_SIZE > ROI_W) continue;

            int csum = 0;
            for (int y = 0; y < TPL_SIZE; y += 2) {
                const uint8_t *row = roi + (y0 + y) * ROI_W + x0;
                for (int x = 0; x < TPL_SIZE; x += 2) csum += row[x];
            }
            int dc = (csum >> 6) - t->u8_mean_64;
            int sad = 0;
            for (int y = 0; y < TPL_SIZE; y += 2) {
                const uint8_t *row = roi + (y0 + y) * ROI_W + x0;
                const uint8_t *trow = t->u8 + y * TPL_SIZE;
                for (int x = 0; x < TPL_SIZE; x += 2) {
                    int d = (int)row[x] - (int)trow[x] - dc;
                    sad += (d < 0) ? -d : d;
                }
            }
            if (sad < top_sad[3]) {
                int pos = 3;
                while (pos > 0 && sad < top_sad[pos - 1]) {
                    top_sad[pos] = top_sad[pos - 1];
                    top_x[pos]   = top_x[pos - 1];
                    top_y[pos]   = top_y[pos - 1];
                    pos--;
                }
                top_sad[pos] = sad; top_x[pos] = x0; top_y[pos] = y0;
            }
        }
    }

    float best_ncc = -1.0f;
    int best_x = cx0, best_y = cy0;
    // Check predicted spot + Top 4 SAD peaks
    if (cx0 >= 0 && cy0 >= 0 && cx0 + TPL_SIZE <= ROI_W && cy0 + TPL_SIZE <= ROI_H) {
        best_ncc = eval_weighted_ncc(t, roi, cx0, cy0);
    }
    for (int k = 0; k < 4; k++) {
        if (top_sad[k] == 0x7FFFFFFF) break;
        float ncc = eval_weighted_ncc(t, roi, top_x[k], top_y[k]);
        if (ncc > best_ncc) { best_ncc = ncc; best_x = top_x[k]; best_y = top_y[k]; }
    }

    // Fine 1-px 3x3 polish
    int fx0 = best_x, fy0 = best_y;
    for (int dy = -1; dy <= 1; dy++) {
        int y0 = fy0 + dy;
        if (y0 < 0 || y0 + TPL_SIZE > ROI_H) continue;
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int x0 = fx0 + dx;
            if (x0 < 0 || x0 + TPL_SIZE > ROI_W) continue;
            float ncc = eval_weighted_ncc(t, roi, x0, y0);
            if (ncc > best_ncc) { best_ncc = ncc; best_x = x0; best_y = y0; }
        }
    }

    *out_u = (float)best_x + TPL_SIZE * 0.5f;
    *out_v = (float)best_y + TPL_SIZE * 0.5f;
    return best_ncc;
}

static void capture_chroma_sig(tracker_state_t *ts, const uint8_t *roi_nv12, float u_c, float v_c) {
    const uint8_t *uv = roi_nv12 + (ROI_W * ROI_H);
    int cx = (int)roundf(u_c), cy = (int)roundf(v_c);
    int su = 0, sv = 0, cnt = 0;
    for (int dy = -4; dy <= 4; dy += 2) {
        int y = cy + dy;
        if (y < 0 || y >= ROI_H) continue;
        for (int dx = -4; dx <= 4; dx += 2) {
            int x = cx + dx;
            if (x < 0 || x >= ROI_W) continue;
            int idx = (y >> 1) * ROI_W + (x & ~1);
            su += uv[idx]; sv += uv[idx + 1]; cnt++;
        }
    }
    if (cnt > 0) {
        ts->ref_u = (float)su / cnt;
        ts->ref_v = (float)sv / cnt;
        ts->has_chroma = true;
    }
}

static bool check_chroma_match(const tracker_state_t *ts, const uint8_t *roi_nv12, float u_c, float v_c) {
    if (!ts->has_chroma) return true;
    const uint8_t *uv = roi_nv12 + (ROI_W * ROI_H);
    int cx = (int)roundf(u_c), cy = (int)roundf(v_c);
    int su = 0, sv = 0, cnt = 0;
    for (int dy = -4; dy <= 4; dy += 2) {
        int y = cy + dy;
        if (y < 0 || y >= ROI_H) continue;
        for (int dx = -4; dx <= 4; dx += 2) {
            int x = cx + dx;
            if (x < 0 || x >= ROI_W) continue;
            int idx = (y >> 1) * ROI_W + (x & ~1);
            su += uv[idx]; sv += uv[idx + 1]; cnt++;
        }
    }
    if (cnt == 0) return false;
    float du = (float)su / cnt - ts->ref_u;
    float dv = (float)sv / cnt - ts->ref_v;
    return (du * du + dv * dv) < (26.0f * 26.0f);
}

static im_rect compute_adaptive_crop(float cx, float cy, float radius) {
    int size = (int)roundf(radius * 5.5f);
    if (size < 128) size = 128;
    if (size > CAM_H) size = CAM_H;
    size = (size + 3) & ~3;

    int x0 = ((int)roundf(cx) - size / 2) & ~1;
    int y0 = ((int)roundf(cy) - size / 2) & ~1;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x0 + size > CAM_W) x0 = (CAM_W - size) & ~1;
    if (y0 + size > CAM_H) y0 = (CAM_H - size) & ~1;
    return (im_rect){ x0, y0, size, size };
}

static void process_target_roi(tracker_state_t *ts, const uint8_t *roi_prev, const uint8_t *roi_curr,
                               im_rect curr_crop, const float M_1080[6], float bg_scale,
                               int *out_mode, float *out_ncc) {
    // Rescale T_key if crop changed modestly since keyframe
    if (ts->tpl_key.valid && ts->key_scaled_w != curr_crop.width && ts->key_crop_w > 0) {
        float ratio = (float)curr_crop.width / (float)ts->key_crop_w;
        if (ratio >= 0.55f && ratio <= 1.85f) {
            resample_tpl_from_raw(ts->key_raw, ratio, &ts->tpl_key);
            ts->key_scaled_w = curr_crop.width;
        }
    }

    float pred_1080_x = M_1080[0]*ts->cx + M_1080[1]*ts->cy + M_1080[2] + ts->vx;
    float pred_1080_y = M_1080[3]*ts->cx + M_1080[4]*ts->cy + M_1080[5] + ts->vy;

    float inv_scale = (float)ROI_W / curr_crop.width;
    float pred_u = clampf((pred_1080_x - curr_crop.x) * inv_scale, 10.0f, ROI_W - 10.0f);
    float pred_v = clampf((pred_1080_y - curr_crop.y) * inv_scale, 10.0f, ROI_H - 10.0f);

    // 1. Fast SAD -> Weighted NCC Search on T_dyn (T_key only checked at spot or on re-acquire!)
    float dyn_u = pred_u, dyn_v = pred_v;
    float dyn_ncc = search_fast_ncc(&ts->tpl_dyn, roi_curr, pred_u, pred_v, 22, &dyn_u, &dyn_v);
    float key_ncc = 0.0f;

    int ku0 = (int)roundf(dyn_u) - TPL_SIZE / 2;
    int kv0 = (int)roundf(dyn_v) - TPL_SIZE / 2;
    if (ts->tpl_key.valid && ku0 >= 0 && kv0 >= 0 && ku0 + TPL_SIZE <= ROI_W && kv0 + TPL_SIZE <= ROI_H) {
        key_ncc = eval_weighted_ncc(&ts->tpl_key, roi_curr, ku0, kv0);
    }

    // If T_dyn dipped, search wider with T_key to recover
    if (dyn_ncc < 0.52f && ts->tpl_key.valid) {
        float ku = pred_u, kv = pred_v;
        float kn = search_fast_ncc(&ts->tpl_key, roi_curr, ROI_W * 0.5f, ROI_H * 0.5f, 44, &ku, &kv);
        if (kn > dyn_ncc + 0.08f) {
            dyn_u = ku; dyn_v = kv; dyn_ncc = kn; key_ncc = kn;
        }
    }

    // 2. Local Motion Residual (bounded around target, ZERO expf() calls!)
    float det = M_1080[0]*M_1080[4] - M_1080[1]*M_1080[3];
    float id = (fabsf(det) > 1e-4f) ? (1.0f / det) : 1.0f;
    float iM[6] = {
         M_1080[4]*id, -M_1080[1]*id, (M_1080[1]*M_1080[5] - M_1080[4]*M_1080[2])*id,
        -M_1080[3]*id,  M_1080[0]*id, (M_1080[3]*M_1080[2] - M_1080[0]*M_1080[5])*id
    };
    float prev_inv_scale = (float)ROI_W / ts->prev_crop.width;

    const uint8_t *uv_curr = roi_curr + (ROI_W * ROI_H);
    float r_roi = clampf(ts->radius * inv_scale, 7.0f, 42.0f);
    float gate_u = (dyn_ncc > 0.55f) ? dyn_u : pred_u;
    float gate_v = (dyn_ncc > 0.55f) ? dyn_v : pred_v;
    float max_r  = r_roi * 1.45f + 10.0f;
    float r2_max = max_r * max_r;
    float inv_r2 = 1.0f / r2_max;

    int u_min = (int)fmaxf(2.0f, gate_u - max_r);
    int u_max = (int)fminf((float)(ROI_W - 3), gate_u + max_r);
    int v_min = (int)fmaxf(2.0f, gate_v - max_r);
    int v_max = (int)fminf((float)(ROI_H - 3), gate_v + max_r);

    double mot_sum_x = 0, mot_sum_y = 0, mot_mass = 0;
    int local_mot_px = 0, outer_mot_px = 0;

    for (int v = v_min; v <= v_max; v++) {
        float y1080 = curr_crop.y + v / inv_scale;
        float dv = (float)v - gate_v;
        for (int u = u_min; u <= u_max; u++) {
            float du = (float)u - gate_u;
            float d2 = du * du + dv * dv;
            if (d2 >= r2_max) continue;

            float x1080 = curr_crop.x + u / inv_scale;
            const uint8_t *p = roi_curr + v * ROI_W + u;
            int gx = (int)p[1] - (int)p[-1];
            int gy = (int)p[ROI_W] - (int)p[-ROI_W];
            int gmag = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);

            float px1080 = iM[0]*x1080 + iM[1]*y1080 + iM[2];
            float py1080 = iM[3]*x1080 + iM[4]*y1080 + iM[5];
            float pu = (px1080 - ts->prev_crop.x) * prev_inv_scale;
            float pv = (py1080 - ts->prev_crop.y) * prev_inv_scale;

            if (pu >= 1 && pu < ROI_W - 2 && pv >= 1 && pv < ROI_H - 2) {
                float clean = fabsf((float)p[0] - sample_bilinear(roi_prev, ROI_W, ROI_H, pu, pv))
                              - 0.35f * (float)gmag - 12.0f;
                if (clean > 0.0f) {
                    float c_w = 1.0f;
                    if (ts->has_chroma) {
                        int uv_i = (v >> 1) * ROI_W + (u & ~1);
                        float du_c = uv_curr[uv_i] - ts->ref_u;
                        float dv_c = uv_curr[uv_i + 1] - ts->ref_v;
                        float duv2 = du_c * du_c + dv_c * dv_c;
                        c_w = (duv2 < 900.0f) ? (1.0f - duv2 * (1.0f / 900.0f)) : 0.05f;
                    }
                    float w = clean * c_w * (1.0f - d2 * inv_r2);
                    if (d2 < (r_roi * r_roi + 64.0f)) {
                        mot_sum_x += u * w;
                        mot_sum_y += v * w;
                        mot_mass  += w;
                        if (w > 2.0f) local_mot_px++;
                    } else if (clean > 6.0f) {
                        outer_mot_px++; // Motion in surrounding ring (potential decoy entering)
                    }
                }
            }
        }
    }

    // 3. Tight 3x3 Internal LK Flow (9 points, fast!)
    float step = clampf(r_roi * 0.45f, 2.5f, 6.0f);
    float vx_sum = 0, vy_sum = 0;
    int   lk_ok = 0;
    for (int gy = -1; gy <= 1; gy++) {
        for (int gx = -1; gx <= 1; gx++) {
            float tx1080 = ts->cx + (gx * step) / inv_scale;
            float ty1080 = ts->cy + (gy * step) / inv_scale;
            float pu = (tx1080 - ts->prev_crop.x) * prev_inv_scale;
            float pv = (ty1080 - ts->prev_crop.y) * prev_inv_scale;
            float cu = gate_u + gx * step, cv = gate_v + gy * step;
            if (lk_track_point(roi_prev, roi_curr, ROI_W, ROI_H, pu, pv, &cu, &cv, 3)) {
                float bg_u = (M_1080[0]*tx1080 + M_1080[1]*ty1080 + M_1080[2] - curr_crop.x) * inv_scale;
                float bg_v = (M_1080[3]*tx1080 + M_1080[4]*ty1080 + M_1080[5] - curr_crop.y) * inv_scale;
                float rvx = cu - bg_u, rvy = cv - bg_v;
                if (rvx*rvx + rvy*rvy < 20.0f * 20.0f) { vx_sum += rvx; vy_sum += rvy; lk_ok++; }
            }
        }
    }

    // 4. Anti-Decoy & Rotation/Scale Fusion
    bool area_spike = (ts->mot_area_ema > 20.0f) && ((float)(local_mot_px + outer_mot_px) > ts->mot_area_ema * 2.6f)
                      && (outer_mot_px > local_mot_px);
    bool is_moving  = (local_mot_px >= 10 && mot_mass > 140.0);
    bool decoy_hit  = false;
    bool chroma_ok  = check_chroma_match(ts, roi_curr, dyn_u, dyn_v);

    float new_u = dyn_u, new_v = dyn_v;
    bool  allow_dyn_update = false;

    if (dyn_ncc < 0.45f) {
        if (is_moving && !area_spike) {
            float mot_u = (float)(mot_sum_x / mot_mass);
            float mot_v = (float)(mot_sum_y / mot_mass);
            if (check_chroma_match(ts, roi_curr, mot_u, mot_v)) {
                new_u = mot_u; new_v = mot_v;
                allow_dyn_update = true;
            } else {
                decoy_hit = true;
                new_u = pred_u; new_v = pred_v;
                ts->vx *= 0.65f; ts->vy *= 0.65f;
            }
        } else if (lk_ok >= 4 && check_chroma_match(ts, roi_curr, pred_u + vx_sum / lk_ok, pred_v + vy_sum / lk_ok)) {
            new_u = pred_u + (vx_sum / lk_ok);
            new_v = pred_v + (vy_sum / lk_ok);
            allow_dyn_update = true;
        } else {
            new_u = clampf((ts->cx - curr_crop.x) * inv_scale, 10.0f, ROI_W - 10.0f);
            new_v = clampf((ts->cy - curr_crop.y) * inv_scale, 10.0f, ROI_H - 10.0f);
            ts->vx *= 0.60f; ts->vy *= 0.60f;
        }
    } else if (is_moving && !area_spike) {
        float mot_u = (float)(mot_sum_x / mot_mass);
        float mot_v = (float)(mot_sum_y / mot_mass);
        float jump  = hypotf(mot_u - gate_u, mot_v - gate_v);

        if (dyn_ncc > 0.65f && jump > (r_roi * 0.75f + 6.0f)) {
            decoy_hit = true;
            new_u = dyn_u; new_v = dyn_v;
        } else if (dyn_ncc > 0.58f) {
            new_u = 0.72f * dyn_u + 0.28f * mot_u;
            new_v = 0.72f * dyn_v + 0.28f * mot_v;
        } else {
            new_u = mot_u; new_v = mot_v;
        }
        if (!decoy_hit && dyn_ncc > 0.58f && chroma_ok) allow_dyn_update = true;
        ts->mot_area_ema = (ts->mot_area_ema < 1.0f) ? (float)local_mot_px
                           : (0.9f * ts->mot_area_ema + 0.1f * local_mot_px);
    } else {
        if (area_spike && dyn_ncc > 0.55f) decoy_hit = true;
        if (lk_ok >= 4 && dyn_ncc >= 0.52f) {
            float lk_u = pred_u + (vx_sum / lk_ok);
            float lk_v = pred_v + (vy_sum / lk_ok);
            new_u = 0.78f * dyn_u + 0.22f * lk_u;
            new_v = 0.78f * dyn_v + 0.22f * lk_v;
        } else {
            new_u = dyn_u; new_v = dyn_v;
        }
        if (!decoy_hit && dyn_ncc > 0.62f && chroma_ok) allow_dyn_update = true;
    }

    // 5. Fast 3-Scale Probe on T_dyn (drives close <-> far crop scaling)
    int px0 = (int)roundf(new_u) - TPL_SIZE / 2;
    int py0 = (int)roundf(new_v) - TPL_SIZE / 2;
    if (ts->tpl_dyn.valid && !decoy_hit && px0 >= 0 && py0 >= 0 && px0 + TPL_SIZE <= ROI_W && py0 + TPL_SIZE <= ROI_H) {
        fast_tpl_t s_tpl;
        float ncc_100 = eval_weighted_ncc(&ts->tpl_dyn, roi_curr, px0, py0);
        resample_tpl_from_raw(ts->dyn_raw, 0.88f, &s_tpl);
        float ncc_088 = s_tpl.valid ? eval_weighted_ncc(&s_tpl, roi_curr, px0, py0) : -1.0f;
        resample_tpl_from_raw(ts->dyn_raw, 1.14f, &s_tpl);
        float ncc_114 = s_tpl.valid ? eval_weighted_ncc(&s_tpl, roi_curr, px0, py0) : -1.0f;

        if (ncc_114 > ncc_100 + 0.018f && ncc_114 > ncc_088) {
            ts->radius = clampf(ts->radius * 1.04f, 8.0f, 420.0f);
        } else if (ncc_088 > ncc_100 + 0.018f && ncc_088 > ncc_114) {
            ts->radius = clampf(ts->radius * 0.96f, 8.0f, 420.0f);
        } else if (bg_scale != 1.0f) {
            ts->radius = clampf(ts->radius * bg_scale, 8.0f, 420.0f);
        }
    }

    if (allow_dyn_update) {
        capture_tpl_roi(roi_curr, new_u, new_v, &ts->tpl_dyn);
        copy_raw_patch(roi_curr, new_u, new_v, ts->dyn_raw);

        // Refresh T_key when crop scaled by >=1.35x or every ~45 frames of strong lock
        ts->frames_since_key++;
        bool scale_changed = (ts->key_crop_w > 0) &&
            (curr_crop.width >= ts->key_crop_w * 1.35f || curr_crop.width <= ts->key_crop_w * 0.74f);
        if (!ts->tpl_key.valid || (scale_changed && dyn_ncc > 0.70f) ||
            (ts->frames_since_key >= 45 && dyn_ncc > 0.78f && key_ncc > 0.55f)) {
            capture_tpl_roi(roi_curr, new_u, new_v, &ts->tpl_key);
            copy_raw_patch(roi_curr, new_u, new_v, ts->key_raw);
            ts->key_crop_w = curr_crop.width;
            ts->key_scaled_w = curr_crop.width;
            ts->frames_since_key = 0;
        }
    }

    float new_cx = curr_crop.x + new_u / inv_scale;
    float new_cy = curr_crop.y + new_v / inv_scale;
    if (dyn_ncc >= 0.45f || allow_dyn_update) {
        float inst_vx = new_cx - (M_1080[0]*ts->cx + M_1080[1]*ts->cy + M_1080[2]);
        float inst_vy = new_cy - (M_1080[3]*ts->cx + M_1080[4]*ts->cy + M_1080[5]);
        ts->vx = 0.55f * ts->vx + 0.45f * clampf(inst_vx, -30.0f, 30.0f);
        ts->vy = 0.55f * ts->vy + 0.45f * clampf(inst_vy, -30.0f, 30.0f);
    }
    ts->cx = clampf(new_cx, 4.0f, CAM_W - 4.0f);
    ts->cy = clampf(new_cy, 4.0f, CAM_H - 4.0f);

    *out_mode = decoy_hit ? 2 : (is_moving ? 1 : 0);
    *out_ncc  = dyn_ncc;
}

static void send_target_udp(int sock, const struct sockaddr_in *tx, float cx, float cy, bool visible) {
    uint16_t rx = 0, ry = 0;
    if (visible) {
        rx = (uint16_t)lroundf(clampf(cx / CAM_W, 0.0f, 1.0f) * 65535.0f);
        ry = (uint16_t)lroundf(clampf(cy / CAM_H, 0.0f, 1.0f) * 65535.0f);
        if (rx == 0 && ry == 0) rx = 1;
    }
    uint8_t pkt[4] = { (uint8_t)(rx >> 8), (uint8_t)(rx & 0xFF), (uint8_t)(ry >> 8), (uint8_t)(ry & 0xFF) };
    sendto(sock, pkt, 4, 0, (const struct sockaddr *)tx, sizeof(*tx));
}

int main(void) {
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    nice(5); // Yield CPU priority to vtx & arduplane
    init_template_weights();

    int udp = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    struct sockaddr_in rx_addr = {
        .sin_family = AF_INET, .sin_port = htons(UDP_RX_PORT), .sin_addr.s_addr = htonl(INADDR_LOOPBACK)
    };
    struct sockaddr_in tx_addr = {
        .sin_family = AF_INET, .sin_port = htons(UDP_TX_PORT), .sin_addr.s_addr = htonl(INADDR_LOOPBACK)
    };
    if (bind(udp, (struct sockaddr *)&rx_addr, sizeof(rx_addr)) < 0) {
        perror("bind 14602");
        return 1;
    }

    capture_ctx_t cap = { .heap_fd = -1, .v4l2_fd = -1 };
    tracker_state_t ts = {0};

    fprintf(stderr, "[tracker v4] Ready & Sleeping (0%% CPU) waiting for Mac click on UDP %d...\n", UDP_RX_PORT);

    uint32_t frm_cnt = 0;
    double   last_log_ms = now_ms();
    double   sum_rga_ms = 0, sum_cpu_ms = 0;

    while (!g_quit) {
        struct pollfd pfds[2];
        int nfds = 1;
        pfds[0] = (struct pollfd){ .fd = udp, .events = POLLIN };
        if (cap.active) {
            pfds[1] = (struct pollfd){ .fd = cap.v4l2_fd, .events = POLLIN };
            nfds = 2;
        }

        int ret = poll(pfds, (nfds_t)nfds, cap.active ? 100 : -1);
        if (ret < 0) continue;

        /* 1. Drain any incoming click commands from crsf_mux */
        if (pfds[0].revents & POLLIN) {
            uint8_t cmd[16];
            ssize_t r;
            while ((r = recv(udp, cmd, sizeof(cmd), MSG_DONTWAIT)) > 0) {
                if (r < 4) continue;
                uint16_t raw_x = ((uint16_t)cmd[0] << 8) | cmd[1];
                uint16_t raw_y = ((uint16_t)cmd[2] << 8) | cmd[3];

                if (raw_x == 0 && raw_y == 0) {
                    memset(&ts, 0, sizeof(ts));
                    send_target_udp(udp, &tx_addr, 0, 0, false);
                    stop_capture(&cap);
                } else {
                    if (!cap.active && start_capture(&cap) < 0) continue;
                    memset(&ts, 0, sizeof(ts));
                    ts.tracking = true;
                    ts.cx = ((float)raw_x / 65535.0f) * CAM_W;
                    ts.cy = ((float)raw_y / 65535.0f) * CAM_H;
                    ts.radius = 14.0f; // Tighter initial lock for far targets
                    fprintf(stderr, "[tracker] Click locked at (%.1f, %.1f) in 1080p\n", ts.cx, ts.cy);
                }
            }
        }

        if (!cap.active || !ts.tracking || nfds < 2 || !(pfds[1].revents & POLLIN)) continue;

        /* 2. Dequeue 1080p NV12 frame from /dev/video11 */
        struct v4l2_buffer buf = {0};
        struct v4l2_plane  pl  = {0};
        buf.type = cap.buf_type;
        buf.memory = V4L2_MEMORY_DMABUF;
        if (cap.buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            buf.m.planes = &pl; buf.length = 1;
        }
        if (xioctl(cap.v4l2_fd, VIDIOC_DQBUF, &buf) < 0) continue;

        int idx = (int)buf.index;
        int cur = cap.ping;
        int prv = 1 - cur;

        double t0 = now_ms();

        /* 3. RGA Hardware Pass 1 (Wide GMC 480x272) + Pass 2 (Adaptive Target ROI 128x128) */
        rga_buffer_t src_rga  = rga_wrap_img(&cap.cam[idx]);
        rga_buffer_t wide_rga = rga_wrap_img(&cap.wide[cur]);
        rga_buffer_t roi_rga  = rga_wrap_img(&cap.roi[cur]);

        im_rect curr_crop = compute_adaptive_crop(ts.cx, ts.cy, ts.radius);
        rga_buffer_t pat  = {0};
        im_rect drect     = { 0, 0, ROI_W, ROI_H };
        im_rect prect     = {0};

        imresize_t(src_rga, wide_rga, 0, 0, IM_INTERP_DEFAULT, 1);
        improcess(src_rga, roi_rga, pat, curr_crop, drect, prect, IM_SYNC);

        qbuf_cam(&cap, (uint32_t)idx);
        double t1 = now_ms();

        /* 4. CPU Cache Sync & Vision Core */
        dmabuf_sync_cpu(cap.wide[cur].fd, true);
        dmabuf_sync_cpu(cap.roi[cur].fd,  true);

        int   trk_mode  = 0;
        float ncc_score = 0.0f;
        if (!ts.has_prev) {
            float inv_s = (float)ROI_W / curr_crop.width;
            float u0 = (ts.cx - curr_crop.x) * inv_s;
            float v0 = (ts.cy - curr_crop.y) * inv_s;
            capture_tpl_roi(cap.roi[cur].map, u0, v0, &ts.tpl_dyn);
            copy_raw_patch(cap.roi[cur].map, u0, v0, ts.dyn_raw);
            ts.tpl_key = ts.tpl_dyn;
            memcpy(ts.key_raw, ts.dyn_raw, sizeof(ts.key_raw));
            ts.key_crop_w = curr_crop.width;
            ts.key_scaled_w = curr_crop.width;
            capture_chroma_sig(&ts, cap.roi[cur].map, u0, v0);
            ts.prev_crop = curr_crop;
            ts.has_prev = true;
        } else {
            float M_1080[6], bg_scale = 1.0f;
            estimate_wide_gmc(cap.wide[prv].map, cap.wide[cur].map, ts.cx, ts.cy, ts.radius, M_1080, &bg_scale);
            process_target_roi(&ts, cap.roi[prv].map, cap.roi[cur].map, curr_crop, M_1080, bg_scale, &trk_mode, &ncc_score);
            ts.prev_crop = curr_crop;
        }

        dmabuf_sync_cpu(cap.wide[cur].fd, false);
        dmabuf_sync_cpu(cap.roi[cur].fd,  false);

        double t2 = now_ms();
        sum_rga_ms += (t1 - t0);
        sum_cpu_ms += (t2 - t1);
        frm_cnt++;
        cap.ping = prv;

        /* 5. Send tracked crosshair back to Mac via crsf_mux (~30 Hz at 60 fps) */
        if ((frm_cnt & 1U) == 0) {
            send_target_udp(udp, &tx_addr, ts.cx, ts.cy, true);
        }

        if (t2 - last_log_ms >= 2000.0) {
            double dt_s = (t2 - last_log_ms) / 1000.0;
            const char *mstr = (trk_mode == 2) ? "DCY" : (trk_mode == 1 ? "MOV" : "STP");
            fprintf(stderr, "[tracker] %.1f fps | rga=%.2f ms cpu=%.2f ms | %s ncc=%.2f | pos=(%.0f,%.0f) r=%.0f crop=%d\n",
                    frm_cnt / dt_s, sum_rga_ms / frm_cnt, sum_cpu_ms / frm_cnt,
                    mstr, ncc_score, ts.cx, ts.cy, ts.radius, curr_crop.width);
            frm_cnt = 0; sum_rga_ms = 0; sum_cpu_ms = 0;
            last_log_ms = t2;
        }
    }

    stop_capture(&cap);
    close(udp);
    return 0;
}