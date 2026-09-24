/*
 * Dedicated Low-Latency 720p60 USB CDC Streamer (VTX)
 * Pure N-1 Short-Term Reference Pipeline with GIR (128x32 Intra Box)
 * Infinite GOP (rc:gop = 65536) with periodic header resync for mid-stream decoders
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/dma-buf.h>
#include <linux/videodev2.h>

#include <rockchip/rk_mpi.h>
#include <rockchip/rk_mpi_cmd.h>
#include <rockchip/rk_venc_cmd.h>
#include <rockchip/rk_venc_rc.h>
#include <rockchip/rk_venc_cfg.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/rk_mpp_cfg.h>
#include <rockchip/rk_venc_ref.h>

#define MAX_V4L2_BUFFERS   2
#define FIFO_SIZE          2
#define MPP_ALIGN(x, a)    (((x) + (a) - 1) & ~((a) - 1))

typedef struct {
    char        v4l2_dev[64];
    char        cdc_dev[64];
    uint32_t    width;
    uint32_t    height;
    uint32_t    fps;
    uint32_t    bitrate_kbps;
    uint32_t    gop;
    int         codec_type;
    int         slicing;
} VtxConfig;

typedef struct {
    int         dma_fd;
    size_t      length;
    MppBuffer   mpp_buf;
} CamBuffer;

typedef struct {
    VtxConfig       cfg;
    int             v4l2_fd;
    uint32_t        buf_type;
    MppBufferGroup  buf_group;
    CamBuffer       buffers[MAX_V4L2_BUFFERS];
    uint32_t        buf_count;

    MppCtx          mpp_ctx;
    MppApi         *mpi;
    MppEncCfg       enc_cfg;
    MppEncRefCfg    ref_cfg;

    uint8_t         hdr_buf[512];
    size_t          hdr_len;

    int             fifo[FIFO_SIZE];
    int             fifo_head;
    int             fifo_tail;
    pthread_mutex_t fifo_lock;

    int             cdc_fd;
    pthread_t       cap_thd;
    pthread_t       tx_thd;
} VtxContext;

static volatile bool quit = false;

static void sigterm_handler(int sig) {
    (void)sig;
    quit = true;
}

static int set_serial_raw(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return 0;
    cfmakeraw(&tty);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF);
    tty.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cflag |= (CS8 | CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tcflush(fd, TCIOFLUSH);
    return tcsetattr(fd, TCSANOW, &tty);
}

static inline int cdc_write_all(int fd, const uint8_t *buf, size_t len) {
    size_t total_written = 0;
    struct pollfd pfd = { .fd = fd, .events = POLLOUT };
    int retry = 0;

    while (total_written < len && !quit) {
        int ret = poll(&pfd, 1, 10);
        if (ret < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, ">>> CDC ERROR: poll failed: %s <<<\n", strerror(errno));
            return -1;
        }
        if (ret == 0) {
            if (++retry > 50) {
                fprintf(stderr, ">>> CDC TIMEOUT: Dropped %zu bytes <<<\n", len - total_written);
                return -1;
            }
            continue;
        }
        retry = 0;

        ssize_t n = write(fd, buf + total_written, len - total_written);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            fprintf(stderr, ">>> CDC ERROR: write failed: %s <<<\n", strerror(errno));
            return -1;
        }
        total_written += (size_t)n;
    }
    return (int)total_written;
}

static int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

/* ----------------------------------------------------------------------------
 * THREAD 1: Full-Speed Camera Capture (Runs at real 60 FPS)
 * ---------------------------------------------------------------------------- */
static void *camera_capture_thread(void *arg) {
    VtxContext *ctx = (VtxContext *)arg;
    prctl(PR_SET_NAME, "vtx_cap", 0, 0, 0);

    struct sched_param sp = { .sched_priority = 85 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    uint32_t hor_stride = MPP_ALIGN(ctx->cfg.width, 16);
    uint32_t ver_stride = MPP_ALIGN(ctx->cfg.height, 16);
    uint32_t cap_cnt = 0;
    static uint32_t last_seq = 0;

    while (!quit) {
        struct pollfd pfd = { .fd = ctx->v4l2_fd, .events = POLLIN };
        int poll_ret = poll(&pfd, 1, 30);
        if (poll_ret <= 0) continue;

        struct v4l2_buffer buf;
        struct v4l2_plane planes[1];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = ctx->buf_type;
        buf.memory = V4L2_MEMORY_DMABUF;
        if (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            buf.m.planes = planes;
            buf.length = 1;
        }

        if (xioctl(ctx->v4l2_fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno != EAGAIN) {
                fprintf(stderr, "VIDIOC_DQBUF failed: %s\n", strerror(errno));
            }
            continue;
        }

        /* Check for skipped or dropped camera frames */
        if (last_seq != 0 && buf.sequence != last_seq + 1) {
            fprintf(stderr, ">>> CAMERA DROP: missed %u frame(s)! (seq %u vs %u) <<<\n",
                    buf.sequence - (last_seq + 1), buf.sequence, last_seq + 1);
        }
        last_seq = buf.sequence;

        uint32_t idx = buf.index;

        MppFrame frame = NULL;
        mpp_frame_init(&frame);
        mpp_frame_set_width(frame, ctx->cfg.width);
        mpp_frame_set_height(frame, ctx->cfg.height);
        mpp_frame_set_hor_stride(frame, hor_stride);
        mpp_frame_set_ver_stride(frame, ver_stride);
        mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
        mpp_frame_set_buffer(frame, ctx->buffers[idx].mpp_buf);
        mpp_frame_set_pts(frame, (uint64_t)cap_cnt * 1000000ULL / ctx->cfg.fps);

        MPP_RET put_ret;
        do {
            put_ret = ctx->mpi->encode_put_frame(ctx->mpp_ctx, frame);
            if (put_ret != MPP_OK)
                usleep(500);
        } while (put_ret != MPP_OK && !quit);

        if (put_ret == MPP_OK) {
            pthread_mutex_lock(&ctx->fifo_lock);
            ctx->fifo[ctx->fifo_head] = idx;
            ctx->fifo_head = (ctx->fifo_head + 1) % FIFO_SIZE;
            pthread_mutex_unlock(&ctx->fifo_lock);
        }

        mpp_frame_deinit(&frame);
        cap_cnt++;
    }
    return NULL;
}

/* ----------------------------------------------------------------------------
 * THREAD 2: USB CDC Output & Safe V4L2 Buffer Requeue
 * ---------------------------------------------------------------------------- */
static void *venc_tx_thread(void *arg) {
    VtxContext *ctx = (VtxContext *)arg;
    prctl(PR_SET_NAME, "vtx_tx", 0, 0, 0);

    struct sched_param sp = { .sched_priority = 80 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    uint32_t frame_cnt = 0;
    size_t total_bytes = 0;
    struct timespec last_time, cur_time;
    clock_gettime(CLOCK_MONOTONIC, &last_time);

    int first_pkt_of_frame = 1;

    while (!quit) {
    MppPacket packet = NULL;
    MPP_RET ret = ctx->mpi->encode_get_packet(ctx->mpp_ctx, &packet);

    if (ret != MPP_OK || !packet) {
        usleep(300);
        continue;
    }

    void *ptr = mpp_packet_get_pos(packet);
    size_t len = mpp_packet_get_length(packet);
    int partition = mpp_packet_is_partition(packet);
    int eoi = mpp_packet_is_eoi(packet);

    if (ptr && len > 0) {
        if (first_pkt_of_frame && ctx->hdr_len > 0 &&
            (frame_cnt == 0 || frame_cnt % 60 == 0)) {
            cdc_write_all(ctx->cdc_fd, ctx->hdr_buf, ctx->hdr_len);
            total_bytes += ctx->hdr_len;
        }

        cdc_write_all(ctx->cdc_fd, (const uint8_t *)ptr, len);
        total_bytes += len;
        first_pkt_of_frame = 0;
    }

    mpp_packet_deinit(&packet);

    int frame_complete = partition ? eoi : 1;
    if (frame_complete) {
        first_pkt_of_frame = 1;

        pthread_mutex_lock(&ctx->fifo_lock);
        int return_idx = ctx->fifo[ctx->fifo_tail];
        ctx->fifo_tail = (ctx->fifo_tail + 1) % FIFO_SIZE;
        pthread_mutex_unlock(&ctx->fifo_lock);

        struct v4l2_buffer ret_buf;
        struct v4l2_plane ret_planes[1];
        memset(&ret_buf, 0, sizeof(ret_buf));
        memset(ret_planes, 0, sizeof(ret_planes));

        ret_buf.type = ctx->buf_type;
        ret_buf.memory = V4L2_MEMORY_DMABUF;
        ret_buf.index = return_idx;

        if (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            ret_planes[0].m.fd = ctx->buffers[return_idx].dma_fd;
            ret_planes[0].length = ctx->buffers[return_idx].length;
            ret_buf.m.planes = ret_planes;
            ret_buf.length = 1;
        } else {
            ret_buf.m.fd = ctx->buffers[return_idx].dma_fd;
            ret_buf.length = ctx->buffers[return_idx].length;
        }

        xioctl(ctx->v4l2_fd, VIDIOC_QBUF, &ret_buf);
        frame_cnt++;

        if (frame_cnt % 60 == 0) {
            clock_gettime(CLOCK_MONOTONIC, &cur_time);
            double elapsed = (cur_time.tv_sec - last_time.tv_sec) +
                             (cur_time.tv_nsec - last_time.tv_nsec) / 1000000000.0;
            double real_fps = 60.0 / elapsed;

            printf(">>> VTX: %u frames | Real FPS: %.1f | Bitrate: %.1f Kbps <<<\n",
                   frame_cnt, real_fps, (total_bytes * 8.0 / 1000.0) / elapsed);

            total_bytes = 0;
            last_time = cur_time;
            fflush(stdout);
        }
    }
}
}

static int v4l2_and_mpp_init(VtxContext *ctx) {
    ctx->v4l2_fd = open(ctx->cfg.v4l2_dev, O_RDWR | O_NONBLOCK, 0);
    if (ctx->v4l2_fd < 0) return -1;

    struct v4l2_capability cap;
    if (xioctl(ctx->v4l2_fd, VIDIOC_QUERYCAP, &cap) < 0) return -1;

    ctx->buf_type = (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) ?
                    V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = ctx->buf_type;

    if (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        fmt.fmt.pix_mp.width = ctx->cfg.width;
        fmt.fmt.pix_mp.height = ctx->cfg.height;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
    } else {
        fmt.fmt.pix.width = ctx->cfg.width;
        fmt.fmt.pix.height = ctx->cfg.height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }

    if (xioctl(ctx->v4l2_fd, VIDIOC_S_FMT, &fmt) < 0) return -1;

    MPP_RET ret = mpp_buffer_group_get_internal(&ctx->buf_group, MPP_BUFFER_TYPE_ION);
    if (ret != MPP_OK) return -1;

    uint32_t hor_stride = MPP_ALIGN(ctx->cfg.width, 16);
    uint32_t ver_stride = MPP_ALIGN(ctx->cfg.height, 16);
    size_t frame_buf_size = hor_stride * ver_stride * 3 / 2;
    ctx->buf_count = MAX_V4L2_BUFFERS;

    for (uint32_t i = 0; i < ctx->buf_count; ++i) {
        ret = mpp_buffer_get(ctx->buf_group, &ctx->buffers[i].mpp_buf, frame_buf_size);
        if (ret != MPP_OK) return -1;
        ctx->buffers[i].dma_fd = mpp_buffer_get_fd(ctx->buffers[i].mpp_buf);
        ctx->buffers[i].length = frame_buf_size;
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = ctx->buf_count;
    req.type = ctx->buf_type;
    req.memory = V4L2_MEMORY_DMABUF;

    if (xioctl(ctx->v4l2_fd, VIDIOC_REQBUFS, &req) < 0) return -1;

    for (uint32_t i = 0; i < ctx->buf_count; ++i) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[1];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = ctx->buf_type;
        buf.memory = V4L2_MEMORY_DMABUF;
        buf.index = i;

        if (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            planes[0].m.fd = ctx->buffers[i].dma_fd;
            planes[0].length = ctx->buffers[i].length;
            buf.m.planes = planes;
            buf.length = 1;
        } else {
            buf.m.fd = ctx->buffers[i].dma_fd;
            buf.length = ctx->buffers[i].length;
        }

        if (xioctl(ctx->v4l2_fd, VIDIOC_QBUF, &buf) < 0) return -1;
    }

    /* Start camera stream now so buffers are ready before thread execution */
    enum v4l2_buf_type type = ctx->buf_type;
    if (xioctl(ctx->v4l2_fd, VIDIOC_STREAMON, &type) < 0) return -1;
    usleep(100000);
    ret = mpp_create(&ctx->mpp_ctx, &ctx->mpi);
    if (ret != MPP_OK) return -1;

    MppPollType timeout = MPP_POLL_NON_BLOCK;
    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_SET_INPUT_TIMEOUT, &timeout);
    if (ret != MPP_OK) return -1;

    timeout = MPP_POLL_BLOCK;
    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
    if (ret != MPP_OK) return -1;

    ret = mpp_init(ctx->mpp_ctx, MPP_CTX_ENC, (MppCodingType)ctx->cfg.codec_type);



    ret = mpp_init(ctx->mpp_ctx, MPP_CTX_ENC, (MppCodingType)ctx->cfg.codec_type);
    if (ret != MPP_OK) return -1;

    ret = mpp_enc_cfg_init(&ctx->enc_cfg);
    if (ret != MPP_OK) return -1;

    mpp_enc_cfg_set_s32(ctx->enc_cfg, "prep:width", ctx->cfg.width);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "prep:height", ctx->cfg.height);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "prep:hor_stride", hor_stride);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "prep:ver_stride", ver_stride);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "prep:format", MPP_FMT_YUV420SP);

    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:mode", MPP_ENC_RC_MODE_CBR);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:bps_target", ctx->cfg.bitrate_kbps * 1000);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:bps_max", ctx->cfg.bitrate_kbps * 1000);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:bps_min", ctx->cfg.bitrate_kbps * 1000 * 8 / 10);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:fps_in_flex", 0);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:fps_in_num", ctx->cfg.fps);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:fps_in_denorm", 1);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:fps_out_flex", 0);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:fps_out_num", ctx->cfg.fps);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:fps_out_denorm", 1);

    /* Infinite GOP: 1 initial IDR frame, then continuous P-frames */
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:gop", ctx->cfg.gop);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:max_reenc_times", 0);

    /* Hardware GIR: 128x32 Intra Box */
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:refresh_en", 0);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:refresh_mode", 0);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:refresh_num", 11);

    mpp_enc_cfg_set_s32(ctx->enc_cfg, "codec:type", ctx->cfg.codec_type);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:profile", 1);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:scaling_list", 0);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:sao_luma_disable", 1);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:sao_chroma_disable", 1);

    mpp_enc_cfg_set_s32(ctx->enc_cfg, "base:low_delay", 0);

    if (ctx->cfg.slicing) {
        mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:mode", MPP_ENC_SPLIT_BY_CTU);
        mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:arg", 60);
        mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:out", 1);
    } else {
        mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:mode", MPP_ENC_SPLIT_NONE);
        mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:arg", 0);
        mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:out", 0);
    }

    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_CFG, ctx->enc_cfg);
    if (ret != MPP_OK) { fprintf(stderr, "MPP_ENC_SET_CFG failed: %d\n", ret); return -1; }

    /* ---- Reference config: Pure N-1 short-term reference chain ---- */
    mpp_enc_ref_cfg_init(&ctx->ref_cfg);
    mpp_enc_ref_cfg_set_cfg_cnt(ctx->ref_cfg, 0, 1);

    MppEncRefStFrmCfg st_cfg;
    memset(&st_cfg, 0, sizeof(st_cfg));
    st_cfg.is_non_ref  = 0;
    st_cfg.temporal_id = 0;
    st_cfg.ref_mode    = REF_TO_PREV_REF_FRM;
    st_cfg.ref_arg     = 0;
    st_cfg.repeat      = 0;
    mpp_enc_ref_cfg_add_st_cfg(ctx->ref_cfg, 1, &st_cfg);

    ret = mpp_enc_ref_cfg_check(ctx->ref_cfg);
    if (ret != MPP_OK) { fprintf(stderr, "mpp_enc_ref_cfg_check failed: %d\n", ret); return -1; }

    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_REF_CFG, ctx->ref_cfg);
    if (ret != MPP_OK) { fprintf(stderr, "MPP_ENC_SET_REF_CFG failed: %d\n", ret); return -1; }

    MppEncHeaderMode hdr_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
    ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_HEADER_MODE, &hdr_mode);

    MppEncSeiMode sei_mode = MPP_ENC_SEI_MODE_DISABLE;
    ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_SEI_CFG, &sei_mode);

    MppPacket hdr_pkt = NULL;
    ret = mpp_packet_init(&hdr_pkt, ctx->hdr_buf, sizeof(ctx->hdr_buf));
    if (ret != MPP_OK) return -1;

    mpp_packet_set_length(hdr_pkt, 0);
    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_GET_HDR_SYNC, hdr_pkt);
    if (ret != MPP_OK) {
        ctx->hdr_len = 0;
    } else {
        ctx->hdr_len = mpp_packet_get_length(hdr_pkt);
        fprintf(stderr, ">>> H.265 header: %zu bytes <<<\n", ctx->hdr_len);
    }
    mpp_packet_deinit(&hdr_pkt);

    return 0;
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    signal(SIGINT, sigterm_handler);
    signal(SIGTERM, sigterm_handler);

    VtxContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    pthread_mutex_init(&ctx.fifo_lock, NULL);

    strncpy(ctx.cfg.v4l2_dev, "/dev/video11", sizeof(ctx.cfg.v4l2_dev) - 1);
    strncpy(ctx.cfg.cdc_dev, "/dev/ttyGS0", sizeof(ctx.cfg.cdc_dev) - 1);
    ctx.cfg.width = 1280;
    ctx.cfg.height = 720;
    ctx.cfg.fps = 60;
    ctx.cfg.bitrate_kbps = 2400;
    ctx.cfg.gop = 65536;
    ctx.cfg.codec_type = MPP_VIDEO_CodingHEVC;

    const char *slice_env = getenv("VTX_SLICING");
    ctx.cfg.slicing = slice_env ? (atoi(slice_env) != 0) : 1;

    if (v4l2_and_mpp_init(&ctx) < 0) {
        fprintf(stderr, "Failed to initialize V4L2 and MPP\n");
        return -1;
    }

    /* Open CDC device before launching threads */
    ctx.cdc_fd = open(ctx.cfg.cdc_dev, O_WRONLY | O_NONBLOCK | O_NOCTTY);
    if (ctx.cdc_fd < 0) {
        fprintf(stderr, "ERROR: Cannot open output '%s': %s\n", ctx.cfg.cdc_dev, strerror(errno));
        return -1;
    }
    set_serial_raw(ctx.cdc_fd);

    pthread_create(&ctx.tx_thd, NULL, venc_tx_thread, &ctx);
    pthread_create(&ctx.cap_thd, NULL, camera_capture_thread, &ctx);

    while (!quit) sleep(1);

    pthread_join(ctx.cap_thd, NULL);
    pthread_join(ctx.tx_thd, NULL);

    if (ctx.cdc_fd >= 0) close(ctx.cdc_fd);

    enum v4l2_buf_type vtype = ctx.buf_type;
    xioctl(ctx.v4l2_fd, VIDIOC_STREAMOFF, &vtype);
    close(ctx.v4l2_fd);

    if (ctx.ref_cfg) mpp_enc_ref_cfg_deinit(&ctx.ref_cfg);
    if (ctx.enc_cfg) mpp_enc_cfg_deinit(ctx.enc_cfg);
    if (ctx.mpp_ctx) mpp_destroy(ctx.mpp_ctx);

    for (uint32_t i = 0; i < ctx.buf_count; ++i) {
        if (ctx.buffers[i].mpp_buf) mpp_buffer_put(ctx.buffers[i].mpp_buf);
    }
    if (ctx.buf_group) mpp_buffer_group_put(ctx.buf_group);
    pthread_mutex_destroy(&ctx.fifo_lock);

    return 0;
}