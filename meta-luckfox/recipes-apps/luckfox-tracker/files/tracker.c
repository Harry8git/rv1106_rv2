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

#define NUM_RAYS        36
#define TPL_SIZE        32      // 32x32 Zero-Mean NCC template inside ROI

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

/* ---------------- CMA DMA-BUF Allocator ---------------- */
typedef struct {
    int      fd;
    size_t   size;
    uint8_t *map;
    int      w, h, stride;
} dma_img_t;

static int alloc_cma_nv12(int heap_fd, dma_img_t *img, int w, int h, int stride, size_t min_bytes) {
    memset(img, 0, sizeof(*img));
    img->fd = -1;
    img->w = w; img->h = h; img->stride = stride;
    size_t sz = (size_t)stride * (size_t)h * 3 / 2;
    if (min_bytes > sz) sz = min_bytes;
    sz = (sz + 4095u) & ~4095u; // 4K page-aligned for DMA-BUF

    struct dma_heap_allocation_data alloc = {
        .len = sz,
        .fd_flags = O_RDWR | O_CLOEXEC,
        .heap_flags = 0
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
    return 0;
}

static void free_cma_img(dma_img_t *img) {
    if (img->map && img->map != MAP_FAILED) munmap(img->map, img->size);
    if (img->fd >= 0) close(img->fd);
    memset(img, 0, sizeof(*img));
    img->fd = -1;
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

    fprintf(stderr, "[tracker] S_FMT type=%u planes=%u stride=%d drv_size=%zu alloc_sz=%zu\n",
            c->buf_type,
            (c->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) ? fmt.fmt.pix_mp.num_planes : 1u,
            stride, drv_size, alloc_sz);

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
    fprintf(stderr, "[tracker] Active: /dev/video11 1080p NV12 + RGA running (stride=%d)\n", stride);
    return 0;

fail:
    c->active = true;
    stop_capture(c);
    return -1;
}

/* ---------------- Vision Primitives (Y8) ---------------- */
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

    for (int it = 0; it < 5; it++) {
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
        if (vx * vx + vy * vy < 0.01f) break;
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

// Wide GMC: estimates 6-DOF affine M_bg in FULL 1920x1080 coordinates (prev -> curr)
static void estimate_wide_gmc(const uint8_t *w_prev, const uint8_t *w_curr,
                              float tgt_x_1080, float tgt_y_1080, float tgt_r_1080,
                              float M_1080[6], float *out_scale) {
    // Identity default
    M_1080[0] = 1; M_1080[1] = 0; M_1080[2] = 0;
    M_1080[3] = 0; M_1080[4] = 1; M_1080[5] = 0;
    *out_scale = 1.0f;

    const float sx_scale = (float)CAM_W / WIDE_W;
    const float sy_scale = (float)CAM_H / WIDE_H;

    // 6x4 grid of best Shi-Tomasi corners outside target exclusion zone
    float px_pts[24], py_pts[24], cx_pts[24], cy_pts[24];
    int n_pts = 0;
    int cell_w = WIDE_W / 6, cell_h = WIDE_H / 4;

    for (int gy = 0; gy < 4; gy++) {
        for (int gx = 0; gx < 6; gx++) {
            int x0 = gx * cell_w + 10, x1 = (gx + 1) * cell_w - 10;
            int y0 = gy * cell_h + 10, y1 = (gy + 1) * cell_h - 10;
            int best_x = -1, best_y = -1;
            float best_score = 400.0f;

            for (int y = y0; y < y1; y += 4) {
                for (int x = x0; x < x1; x += 4) {
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
                if (lk_track_point(w_prev, w_curr, WIDE_W, WIDE_H, (float)best_x, (float)best_y, &cx, &cy, 4)) {
                    float dx = cx - best_x, dy = cy - best_y;
                    if (dx * dx + dy * dy < 32.0f * 32.0f) {
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
    if (n_pts < 6) return;

    int best_inliers = -1;
    float best_M[6] = {1, 0, 0, 0, 1, 0};
    uint32_t rng = 0x12345678u;

    for (int it = 0; it < 35; it++) {
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
        // Sanity check: scale near 1.0
        float det = M[0] * M[4] - M[1] * M[3];
        if (det < 0.80f || det > 1.25f) continue;

        int inliers = 0;
        for (int k = 0; k < n_pts; k++) {
            float ex = (M[0]*px_pts[k] + M[1]*py_pts[k] + M[2]) - cx_pts[k];
            float ey = (M[3]*px_pts[k] + M[4]*py_pts[k] + M[5]) - cy_pts[k];
            if (ex*ex + ey*ey < 16.0f) inliers++; // 4 px in 1080p (~1 px in 480x272)
        }
        if (inliers > best_inliers) {
            best_inliers = inliers;
            memcpy(best_M, M, sizeof(M));
        }
    }

    if (best_inliers >= 4) {
        memcpy(M_1080, best_M, sizeof(best_M));
        float det = M_1080[0] * M_1080[4] - M_1080[1] * M_1080[3];
        *out_scale = sqrtf(clampf(det, 0.85f, 1.15f));
    }
}

/* ---------------- Tracker State ---------------- */
typedef struct {
    bool     tracking;
    bool     has_prev;
    float    cx, cy;           // Current target center in 1920x1080
    float    vx, vy;           // Velocity in 1920x1080 px/frame
    float    radius;           // Target radius in 1920x1080 px
    im_rect  prev_crop;
    float    rays[NUM_RAYS];   // Polar contour in ROI pixels (0..ROI_W/2)

    // Frozen Reference Template (32x32 zero-mean NCC patch)
    bool     has_tpl;
    float    tpl[TPL_SIZE * TPL_SIZE];
    float    tpl_norm;
} tracker_state_t;

static void capture_ncc_template(tracker_state_t *ts, const uint8_t *roi, float u_c, float v_c) {
    int x0 = (int)roundf(u_c) - TPL_SIZE / 2;
    int y0 = (int)roundf(v_c) - TPL_SIZE / 2;
    if (x0 < 0 || y0 < 0 || x0 + TPL_SIZE > ROI_W || y0 + TPL_SIZE > ROI_H) return;

    double sum = 0.0;
    for (int y = 0; y < TPL_SIZE; y++) {
        for (int x = 0; x < TPL_SIZE; x++) {
            sum += roi[(y0 + y) * ROI_W + (x0 + x)];
        }
    }
    float mean = (float)(sum / (TPL_SIZE * TPL_SIZE));
    double n2 = 0.0;
    for (int y = 0; y < TPL_SIZE; y++) {
        for (int x = 0; x < TPL_SIZE; x++) {
            float d = (float)roi[(y0 + y) * ROI_W + (x0 + x)] - mean;
            ts->tpl[y * TPL_SIZE + x] = d;
            n2 += d * d;
        }
    }
    ts->tpl_norm = sqrtf((float)n2);
    ts->has_tpl = (ts->tpl_norm > 50.0f);
}

static float search_ncc_template(const tracker_state_t *ts, const uint8_t *roi,
                                 float pred_u, float pred_v, int search_r,
                                 float *out_u, float *out_v) {
    if (!ts->has_tpl) return 0.0f;
    int cx0 = (int)roundf(pred_u) - TPL_SIZE / 2;
    int cy0 = (int)roundf(pred_v) - TPL_SIZE / 2;

    float best_ncc = -1.0f;
    int best_x = cx0, best_y = cy0;

    for (int dy = -search_r; dy <= search_r; dy += 2) {
        int y0 = cy0 + dy;
        if (y0 < 0 || y0 + TPL_SIZE > ROI_H) continue;
        for (int dx = -search_r; dx <= search_r; dx += 2) {
            int x0 = cx0 + dx;
            if (x0 < 0 || x0 + TPL_SIZE > ROI_W) continue;

            double sum = 0.0, sum2 = 0.0, dot = 0.0;
            for (int y = 0; y < TPL_SIZE; y++) {
                const uint8_t *row = roi + (y0 + y) * ROI_W + x0;
                const float *trow = ts->tpl + y * TPL_SIZE;
                for (int x = 0; x < TPL_SIZE; x++) {
                    float val = (float)row[x];
                    sum += val;
                    sum2 += val * val;
                    dot += val * trow[x];
                }
            }
            float var = (float)(sum2 - (sum * sum) / (TPL_SIZE * TPL_SIZE));
            if (var < 10.0f) continue;
            float ncc = (float)(dot / (sqrtf(var) * ts->tpl_norm));
            if (ncc > best_ncc) { best_ncc = ncc; best_x = x0; best_y = y0; }
        }
    }
    *out_u = (float)best_x + TPL_SIZE * 0.5f;
    *out_v = (float)best_y + TPL_SIZE * 0.5f;
    return best_ncc;
}

static im_rect compute_adaptive_crop(float cx, float cy, float radius) {
    int size = (int)roundf(radius * 5.0f);
    if (size < 128) size = 128;
    if (size > CAM_H) size = CAM_H;
    size = (size + 3) & ~3; // RGA2 4-px aligned width/height

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
                               bool *out_moving, float *out_ncc) {
    // Predict target center in 1080p via GMC + previous residual velocity
    float pred_1080_x = M_1080[0]*ts->cx + M_1080[1]*ts->cy + M_1080[2] + ts->vx;
    float pred_1080_y = M_1080[3]*ts->cx + M_1080[4]*ts->cy + M_1080[5] + ts->vy;

    float inv_scale = (float)ROI_W / curr_crop.width;
    float pred_u = clampf((pred_1080_x - curr_crop.x) * inv_scale, 16.0f, ROI_W - 16.0f);
    float pred_v = clampf((pred_1080_y - curr_crop.y) * inv_scale, 16.0f, ROI_H - 16.0f);

    // Invert M_1080 (prev -> curr) to map curr -> prev for dense background-aligned diff
    float det = M_1080[0]*M_1080[4] - M_1080[1]*M_1080[3];
    float id = (fabsf(det) > 1e-4f) ? (1.0f / det) : 1.0f;
    float iM[6] = {
         M_1080[4]*id, -M_1080[1]*id, (M_1080[1]*M_1080[5] - M_1080[4]*M_1080[2])*id,
        -M_1080[3]*id,  M_1080[0]*id, (M_1080[3]*M_1080[2] - M_1080[0]*M_1080[5])*id
    };

    float prev_inv_scale = (float)ROI_W / ts->prev_crop.width;

    // 1. Compute Background-Aligned Motion Residual & Sobel Gradient inside 128x128
    static uint8_t mot_map[ROI_W * ROI_H];
    static uint8_t grad_map[ROI_W * ROI_H];
    memset(mot_map, 0, sizeof(mot_map));
    memset(grad_map, 0, sizeof(grad_map));

    double mot_sum_x = 0, mot_sum_y = 0, mot_mass = 0;
    int mot_pixels = 0;

    for (int v = 2; v < ROI_H - 2; v++) {
        float y1080 = curr_crop.y + v / inv_scale;
        for (int u = 2; u < ROI_W - 2; u++) {
            float x1080 = curr_crop.x + u / inv_scale;
            const uint8_t *p = roi_curr + v * ROI_W + u;
            int gx = (int)p[1] - (int)p[-1];
            int gy = (int)p[ROI_W] - (int)p[-ROI_W];
            int gmag = abs(gx) + abs(gy);
            grad_map[v * ROI_W + u] = (uint8_t)(gmag > 255 ? 255 : gmag);

            float px1080 = iM[0]*x1080 + iM[1]*y1080 + iM[2];
            float py1080 = iM[3]*x1080 + iM[4]*y1080 + iM[5];
            float pu = (px1080 - ts->prev_crop.x) * prev_inv_scale;
            float pv = (py1080 - ts->prev_crop.y) * prev_inv_scale;

            if (pu >= 1 && pu < ROI_W - 2 && pv >= 1 && pv < ROI_H - 2) {
                float prev_val = sample_bilinear(roi_prev, ROI_W, ROI_H, pu, pv);
                float diff = fabsf((float)p[0] - prev_val);
                // Suppress static edge warp jitter
                float clean = diff - 0.35f * (float)gmag - 12.0f;
                if (clean > 0.0f) {
                    uint8_t m = (uint8_t)(clean > 255.0f ? 255.0f : clean);
                    mot_map[v * ROI_W + u] = m;
                    float du = u - pred_u, dv = v - pred_v;
                    float r_roi = ts->radius * inv_scale;
                    if (du*du + dv*dv < (r_roi * 1.8f + 16.0f)*(r_roi * 1.8f + 16.0f)) {
                        mot_sum_x += u * clean;
                        mot_sum_y += v * clean;
                        mot_mass  += clean;
                        mot_pixels++;
                    }
                }
            }
        }
    }

    // 2. Sparse 5x5 Micro-Grid LK Flow inside target to measure coherent velocity
    float r_roi = clampf(ts->radius * inv_scale, 8.0f, 44.0f);
    float step = clampf(r_roi * 0.4f, 3.0f, 9.0f);
    float vx_sum = 0, vy_sum = 0;
    int   lk_ok = 0;

    for (int gy = -2; gy <= 2; gy++) {
        for (int gx = -2; gx <= 2; gx++) {
            float tx1080 = ts->cx + (gx * step) / inv_scale;
            float ty1080 = ts->cy + (gy * step) / inv_scale;
            float pu = (tx1080 - ts->prev_crop.x) * prev_inv_scale;
            float pv = (ty1080 - ts->prev_crop.y) * prev_inv_scale;
            float cu = pred_u + gx * step;
            float cv = pred_v + gy * step;

            if (lk_track_point(roi_prev, roi_curr, ROI_W, ROI_H, pu, pv, &cu, &cv, 4)) {
                float bg_u = (M_1080[0]*tx1080 + M_1080[1]*ty1080 + M_1080[2] - curr_crop.x) * inv_scale;
                float bg_v = (M_1080[3]*tx1080 + M_1080[4]*ty1080 + M_1080[5] - curr_crop.y) * inv_scale;
                float rvx = cu - bg_u, rvy = cv - bg_v;
                if (rvx*rvx + rvy*rvy < 20.0f * 20.0f) {
                    vx_sum += rvx; vy_sum += rvy; lk_ok++;
                }
            }
        }
    }

    // 3. Zero-Mean NCC Template Anchor (keeps stationary targets locked!)
    float ncc_u = pred_u, ncc_v = pred_v;
    float ncc_score = search_ncc_template(ts, roi_curr, pred_u, pred_v, 14, &ncc_u, &ncc_v);

    // 4. Fuse Motion + NCC + Edges
    bool is_moving = (mot_pixels >= 18 && mot_mass > 350.0);
    float new_u = pred_u, new_v = pred_v;

    if (is_moving) {
        float mot_u = (float)(mot_sum_x / mot_mass);
        float mot_v = (float)(mot_sum_y / mot_mass);

        // Motion-Assisted Polar Contour (36 rays) for boundary & recentering
        float rx_sum = 0, ry_sum = 0;
        for (int k = 0; k < NUM_RAYS; k++) {
            float ang = (2.0f * (float)M_PI * k) / NUM_RAYS;
            float ca = cosf(ang), sa = sinf(ang);
            float best_r = ts->rays[k];
            float best_sc = -1e9f;
            int rmax = (int)clampf(r_roi * 1.8f + 12.0f, 12.0f, 56.0f);

            for (int r = 4; r <= rmax; r++) {
                int u = (int)roundf(mot_u + ca * r);
                int v = (int)roundf(mot_v + sa * r);
                if (u < 1 || u >= ROI_W - 1 || v < 1 || v >= ROI_H - 1) break;
                int u_out = (int)roundf(mot_u + ca * (r + 3));
                int v_out = (int)roundf(mot_v + sa * (r + 3));
                float m_in  = mot_map[v * ROI_W + u];
                float m_out = (u_out >= 0 && u_out < ROI_W && v_out >= 0 && v_out < ROI_H)
                              ? mot_map[v_out * ROI_W + u_out] : 0.0f;
                float edge  = grad_map[v * ROI_W + u];
                float sc = (m_in - 1.2f * m_out) + 0.4f * edge - 0.5f * fabsf((float)r - ts->rays[k]);
                if (sc > best_sc) { best_sc = sc; best_r = (float)r; }
            }
            ts->rays[k] = 0.7f * ts->rays[k] + 0.3f * best_r;
            rx_sum += ts->rays[k];
            ry_sum += ts->rays[k];
        }
        float mean_r_roi = rx_sum / NUM_RAYS;
        ts->radius = 0.85f * ts->radius + 0.15f * clampf(mean_r_roi / inv_scale, 6.0f, 450.0f);

        new_u = 0.55f * mot_u + 0.45f * ((ncc_score > 0.65f) ? ncc_u : pred_u);
        new_v = 0.55f * mot_v + 0.45f * ((ncc_score > 0.65f) ? ncc_v : pred_v);

        // Refresh template safely while motion is high
        capture_ncc_template(ts, roi_curr, new_u, new_v);
    } else {
        // Stationary / Stopped: trust NCC + LK + GMC scale expansion
        if (ncc_score > 0.60f && lk_ok >= 4) {
            float lk_u = pred_u + (vx_sum / lk_ok);
            float lk_v = pred_v + (vy_sum / lk_ok);
            new_u = 0.65f * ncc_u + 0.35f * lk_u;
            new_v = 0.65f * ncc_v + 0.35f * lk_v;
        } else if (ncc_score > 0.55f) {
            new_u = ncc_u; new_v = ncc_v;
        } else if (lk_ok >= 4) {
            new_u = pred_u + (vx_sum / lk_ok);
            new_v = pred_v + (vy_sum / lk_ok);
        }
        ts->radius = clampf(ts->radius * bg_scale, 6.0f, 450.0f);
        for (int k = 0; k < NUM_RAYS; k++) ts->rays[k] = clampf(ts->rays[k] * bg_scale, 4.0f, 56.0f);
    }

    float new_cx = curr_crop.x + new_u / inv_scale;
    float new_cy = curr_crop.y + new_v / inv_scale;
    float inst_vx = new_cx - (M_1080[0]*ts->cx + M_1080[1]*ts->cy + M_1080[2]);
    float inst_vy = new_cy - (M_1080[3]*ts->cx + M_1080[4]*ts->cy + M_1080[5]);

    ts->vx = 0.65f * ts->vx + 0.35f * clampf(inst_vx, -40.0f, 40.0f);
    ts->vy = 0.65f * ts->vy + 0.35f * clampf(inst_vy, -40.0f, 40.0f);
    ts->cx = clampf(new_cx, 4.0f, CAM_W - 4.0f);
    ts->cy = clampf(new_cy, 4.0f, CAM_H - 4.0f);

    *out_moving = is_moving;
    *out_ncc = ncc_score;
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

    fprintf(stderr, "[tracker v3] Ready & Sleeping (0%% CPU) waiting for Mac click on UDP %d...\n", UDP_RX_PORT);

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

        // Sleep infinitely when paused (0% CPU!); 100ms timeout when active
        int ret = poll(pfds, (nfds_t)nfds, cap.active ? 100 : -1);
        if (ret < 0) continue;

        /* 1. Drain any incoming click commands from crsf_mux */
        if (pfds[0].revents & POLLIN) {
            uint8_t cmd[16];
            ssize_t r;
            while ((r = recv(udp, cmd, sizeof(cmd), MSG_DONTWAIT)) > 0) {
                fprintf(stderr, "[tracker] UDP 14602 rx %zd bytes\n", r);
                if (r < 4) continue;
                uint16_t raw_x = ((uint16_t)cmd[0] << 8) | cmd[1];
                uint16_t raw_y = ((uint16_t)cmd[2] << 8) | cmd[3];

                if (raw_x == 0 && raw_y == 0) {
                    // Pause & Power Down
                    ts.tracking = false;
                    ts.has_prev = false;
                    ts.has_tpl  = false;
                    send_target_udp(udp, &tx_addr, 0, 0, false);
                    stop_capture(&cap);
                } else {
                    // Wake up & Initialize Target
                    if (!cap.active && start_capture(&cap) < 0) {
                        continue;
                    }
                    memset(&ts, 0, sizeof(ts));
                    ts.tracking = true;
                    ts.cx = ((float)raw_x / 65535.0f) * CAM_W;
                    ts.cy = ((float)raw_y / 65535.0f) * CAM_H;
                    ts.radius = 22.0f;
                    for (int k = 0; k < NUM_RAYS; k++) ts.rays[k] = 22.0f;
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
        rga_buffer_t src_rga  = wrapbuffer_fd_t(cap.cam[idx].fd, CAM_W, CAM_H, cap.cam[idx].stride, CAM_H, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t wide_rga = wrapbuffer_fd_t(cap.wide[cur].fd, WIDE_W, WIDE_H, WIDE_W, WIDE_H, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t roi_rga  = wrapbuffer_fd_t(cap.roi[cur].fd,  ROI_W,  ROI_H,  ROI_W,  ROI_H,  RK_FORMAT_YCbCr_420_SP);

        im_rect curr_crop = compute_adaptive_crop(ts.cx, ts.cy, ts.radius);
        rga_buffer_t pat  = {0};
        im_rect drect     = { 0, 0, ROI_W, ROI_H };
        im_rect prect     = {0};

        imresize_t(src_rga, wide_rga, 0, 0, IM_INTERP_DEFAULT, 1);
        improcess(src_rga, roi_rga, pat, curr_crop, drect, prect, IM_SYNC);

        qbuf_cam(&cap, (uint32_t)idx); // Return 1080p frame to ISP immediately!
        double t1 = now_ms();

        /* 4. CPU Cache Sync & Vision Core */
        dmabuf_sync_cpu(cap.wide[cur].fd, true);
        dmabuf_sync_cpu(cap.roi[cur].fd,  true);

        bool  is_moving = false;
        float ncc_score = 0.0f;
        if (!ts.has_prev) {
            float inv_s = (float)ROI_W / curr_crop.width;
            capture_ncc_template(&ts, cap.roi[cur].map, (ts.cx - curr_crop.x) * inv_s, (ts.cy - curr_crop.y) * inv_s);
            ts.prev_crop = curr_crop;
            ts.has_prev = true;
        } else {
            float M_1080[6], bg_scale = 1.0f;
            estimate_wide_gmc(cap.wide[prv].map, cap.wide[cur].map, ts.cx, ts.cy, ts.radius, M_1080, &bg_scale);
            process_target_roi(&ts, cap.roi[prv].map, cap.roi[cur].map, curr_crop, M_1080, bg_scale, &is_moving, &ncc_score);
            ts.prev_crop = curr_crop;
        }

        dmabuf_sync_cpu(cap.wide[cur].fd, false);
        dmabuf_sync_cpu(cap.roi[cur].fd,  false);

        double t2 = now_ms();
        sum_rga_ms += (t1 - t0);
        sum_cpu_ms += (t2 - t1);
        frm_cnt++;
        cap.ping = prv;

        /* 5. Send tracked crosshair back to Mac via crsf_mux (~30 Hz: every 2nd frame) */
        if ((frm_cnt & 1U) == 0) {
            send_target_udp(udp, &tx_addr, ts.cx, ts.cy, true);
        }

        if (t2 - last_log_ms >= 2000.0) {
            double dt_s = (t2 - last_log_ms) / 1000.0;
            fprintf(stderr, "[tracker] %.1f fps | rga=%.2f ms cpu=%.2f ms | %s ncc=%.2f | pos=(%.0f,%.0f) r=%.0f crop=%d\n",
                    frm_cnt / dt_s, sum_rga_ms / frm_cnt, sum_cpu_ms / frm_cnt,
                    is_moving ? "MOV" : "STP", ncc_score, ts.cx, ts.cy, ts.radius, curr_crop.width);
            frm_cnt = 0; sum_rga_ms = 0; sum_cpu_ms = 0;
            last_log_ms = t2;
        }
    }

    stop_capture(&cap);
    close(udp);
    return 0;
}