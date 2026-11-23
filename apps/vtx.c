/*
 * Dedicated Low-Latency 720p60 USB Vendor Bulk Streamer (VTX)
 * Linux FunctionFS (Vendor Class 0xFF) Bulk IN Endpoint Pipeline
 * Pure N-1 Short-Term Reference Pipeline (P-frames only after start/resume IDR)
 * Infinite GOP with periodic VPS+SPS+PPS header resync for mid-stream decoders
 */

#define _GNU_SOURCE
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <linux/aio_abi.h>
#include <linux/videodev2.h>
#include <linux/usb/ch9.h>
#include <linux/usb/functionfs.h>

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
#define MPP_ALIGN(x, a)    (((x) + (a) - 1) & ~((a) - 1))

#define FFS_EP1_PATH       "/dev/usb-ffs/vtx/ep1"

#define USB_TX_SLOTS       2
#define USB_TX_BUF_SIZE    (256 * 1024)

typedef struct {
    char        v4l2_dev[64];
    uint32_t    width;
    uint32_t    height;
    uint32_t    fps;
    uint32_t    bitrate_kbps;
    uint32_t    gop;
    int         codec_type;
} VtxConfig;

typedef struct {
    int         dma_fd;
    size_t      length;
    MppBuffer   mpp_buf;
    MppFrame    mpp_frame;
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

    int             ep1_fd;
    int             wake_efd;
    pthread_t       stream_thd;

    atomic_bool     streaming;
    atomic_bool     want_paused;
} VtxContext;

struct aio_old_timespec {
    long tv_sec;
    long tv_nsec;
};

typedef struct {
    aio_context_t   aio_ctx;
    struct iocb     iocbs[USB_TX_SLOTS];
    bool            busy[USB_TX_SLOTS];
    uint8_t         buf[USB_TX_SLOTS][USB_TX_BUF_SIZE];
} UsbAioState;

static UsbAioState s_usb_aio;
static atomic_bool quit = false;
static int g_wake_efd = -1;

static void wake_stream_thread(const VtxContext *ctx) {
    if (ctx->wake_efd >= 0) {
        uint64_t one = 1;
        ssize_t r = write(ctx->wake_efd, &one, sizeof(one));
        (void)r;
    }
}

static void sigterm_handler(int sig) {
    (void)sig;
    atomic_store(&quit, true);
    if (g_wake_efd >= 0) {
        uint64_t one = 1;
        ssize_t r = write(g_wake_efd, &one, sizeof(one));
        (void)r;
    }
}

static int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

static inline int queue_v4l2_buf(VtxContext *ctx, uint32_t idx) {
    struct v4l2_buffer buf;
    struct v4l2_plane planes[1];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));

    buf.type = ctx->buf_type;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = idx;

    if (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        planes[0].m.fd = ctx->buffers[idx].dma_fd;
        planes[0].length = ctx->buffers[idx].length;
        buf.m.planes = planes;
        buf.length = 1;
    } else {
        buf.m.fd = ctx->buffers[idx].dma_fd;
        buf.length = ctx->buffers[idx].length;
    }

    return xioctl(ctx->v4l2_fd, VIDIOC_QBUF, &buf);
}

static int usb_aio_init(void) {
    memset(&s_usb_aio, 0, sizeof(s_usb_aio));
    if (syscall(SYS_io_setup, USB_TX_SLOTS * 2, &s_usb_aio.aio_ctx) < 0) {
        fprintf(stderr, "[vtx] io_setup failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static void usb_aio_deinit(void) {
    if (s_usb_aio.aio_ctx) {
        struct io_event events[USB_TX_SLOTS];
        struct aio_old_timespec ts = { .tv_sec = 0, .tv_nsec = 50000000L };
        int busy_cnt = 0;
        for (int i = 0; i < USB_TX_SLOTS; i++) {
            if (s_usb_aio.busy[i]) busy_cnt++;
        }
        if (busy_cnt > 0) {
            syscall(SYS_io_getevents, s_usb_aio.aio_ctx,
                    busy_cnt, USB_TX_SLOTS, events, &ts);
        }
        syscall(SYS_io_destroy, s_usb_aio.aio_ctx);
        s_usb_aio.aio_ctx = 0;
    }
}

static void usb_aio_reap(const VtxContext *ctx, long wait_ns) {
    static uint32_t aio_err_cnt = 0;
    struct io_event events[USB_TX_SLOTS];
    struct aio_old_timespec ts = { .tv_sec = 0, .tv_nsec = wait_ns };
    long min_nr = (wait_ns > 0) ? 1 : 0;

    int n;
    do {
        n = (int)syscall(SYS_io_getevents, s_usb_aio.aio_ctx,
                         min_nr, USB_TX_SLOTS, events, &ts);
    } while (n < 0 && errno == EINTR &&
             !atomic_load(&quit) && atomic_load(&ctx->streaming));

    for (int i = 0; i < n; i++) {
        int slot = (int)events[i].data;
        if (events[i].res < 0) {
            if ((aio_err_cnt++ % 60U) == 0) {
                fprintf(stderr, "[vtx] USB AIO completion err: %lld (count=%u)\n",
                        (long long)events[i].res, aio_err_cnt);
            }
        } else {
            aio_err_cnt = 0;
        }
        if (slot >= 0 && slot < USB_TX_SLOTS) {
            s_usb_aio.busy[slot] = false;
        }
    }
}

static inline int bulk_write_frame(VtxContext *ctx,
                                   const uint8_t *hdr, size_t hdr_len,
                                   const uint8_t *slice, size_t slice_len)
{
    if (!ctx || ctx->ep1_fd < 0 || !slice || slice_len == 0)
        return 0;

    size_t total = hdr_len + slice_len;
    size_t pad = ((total & 63U) == 0) ? 1U : 0U;
    if (total + pad > USB_TX_BUF_SIZE) {
        errno = EMSGSIZE;
        return -1;
    }

    /* 1. Non-blocking reap of any finished USB transfers */
    usb_aio_reap(ctx, 0);

    int slot = -1;
    for (int i = 0; i < USB_TX_SLOTS; i++) {
        if (!s_usb_aio.busy[i]) {
            slot = i;
            break;
        }
    }

    /* 2. If all slots are busy, wait up to 100ms for 1 slot to finish */
    if (slot < 0) {
        usb_aio_reap(ctx, 100000000L);
        for (int i = 0; i < USB_TX_SLOTS; i++) {
            if (!s_usb_aio.busy[i]) {
                slot = i;
                break;
            }
        }
        if (slot < 0) {
            errno = ETIMEDOUT;
            return -1;
        }
    }

    /* 3. Stage packet into slot buffer (prepend Annex B leading_zero_8bits if 64B-aligned) */
    uint8_t *dst = s_usb_aio.buf[slot];
    size_t off = 0;
    if (pad) {
        dst[off++] = 0x00;
    }
    if (hdr && hdr_len > 0) {
        memcpy(dst + off, hdr, hdr_len);
        off += hdr_len;
    }
    memcpy(dst + off, slice, slice_len);
    total += pad;

    /* 4. Queue asynchronously to FunctionFS UDC DMA */
    struct iocb *cb = &s_usb_aio.iocbs[slot];
    memset(cb, 0, sizeof(*cb));
    cb->aio_data       = (uint64_t)slot;
    cb->aio_lio_opcode = IOCB_CMD_PWRITE;
    cb->aio_fildes     = (uint32_t)ctx->ep1_fd;
    cb->aio_buf        = (uint64_t)(uintptr_t)dst;
    cb->aio_nbytes     = total;
    cb->aio_offset     = 0;

    if (syscall(SYS_io_submit, s_usb_aio.aio_ctx, 1, &cb) != 1) {
        return -1;
    }

    s_usb_aio.busy[slot] = true;
    return (int)total;
}

static int open_vtx_endpoint(VtxContext *ctx) {
    ctx->ep1_fd = open(FFS_EP1_PATH, O_RDWR | O_NONBLOCK);
    if (ctx->ep1_fd < 0) {
        fprintf(stderr, "ERROR: Cannot open '%s': %s\n",
                FFS_EP1_PATH, strerror(errno));
        return -1;
    }

    struct usb_endpoint_descriptor ep_desc;
    memset(&ep_desc, 0, sizeof(ep_desc));

    if (ioctl(ctx->ep1_fd, FUNCTIONFS_ENDPOINT_DESC, &ep_desc) == 0) {
        uint16_t max_packet = le16toh(ep_desc.wMaxPacketSize);
        fprintf(stderr, "[USB DIAG] FunctionFS ep1 actual address: 0x%02X\n",
                ep_desc.bEndpointAddress);
        fprintf(stderr, "[USB DIAG] Transfer type: %u, Max packet: %u\n",
                ep_desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK, max_packet);

        if (ep_desc.bEndpointAddress != 0x85) {
            fprintf(stderr, "[USB DIAG] WARNING: RP2350 currently expects 0x85!\n");
        } else {
            fprintf(stderr, "[USB DIAG] Endpoint matches RP2350 expectation.\n");
        }
    } else {
        fprintf(stderr, "[USB DIAG] Cannot query endpoint descriptor: %s\n",
                strerror(errno));
    }

    fprintf(stderr, ">>> VTX connected to USB Bulk Endpoint <<<\n");
    return 0;
}

static void configure_sensor_exposure(void) {
    int fd = open("/dev/v4l-subdev2", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Warning: Could not open /dev/v4l-subdev2: %s\n", strerror(errno));
        return;
    }

    struct v4l2_control ctrl = { .id = V4L2_CID_EXPOSURE, .value = 250 };
    int ret_exp = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_EXPOSURE;
    int ret_get = ioctl(fd, VIDIOC_G_CTRL, &ctrl);

    close(fd);
    fprintf(stderr, "[sensor-diag] Set exposure ret=%d, readback active exposure=%d (ret=%d)\n",
            ret_exp, ctrl.value, ret_get);
}

/* ----------------------------------------------------------------------------
 * SINGLE STREAMING THREAD: Capture -> Encode -> Requeue -> Async USB Bulk TX
 * ---------------------------------------------------------------------------- */
static void *vtx_stream_thread(void *arg) {
    VtxContext *ctx = (VtxContext *)arg;
    prctl(PR_SET_NAME, "vtx_stream", 0, 0, 0);

    struct sched_param sp = { .sched_priority = 85 };
    int sched_ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (sched_ret != 0) {
        fprintf(stderr, "[vtx] Warning: SCHED_FIFO failed (%s)\n", strerror(sched_ret));
    }

    if (usb_aio_init() < 0) {
        return NULL;
    }

    uint32_t cap_cnt = 0;
    uint32_t frame_cnt = 0;
    uint32_t tx_err_cnt = 0;
    uint32_t last_seq = 0;
    bool first_frame = true;
    bool is_paused = false;
    bool force_idr_hdr = false;

    while (!atomic_load(&quit) && atomic_load(&ctx->streaming)) {
        bool want_pause = atomic_load(&ctx->want_paused);

        if (want_pause && !is_paused) {
            enum v4l2_buf_type vtype = ctx->buf_type;
            if (xioctl(ctx->v4l2_fd, VIDIOC_STREAMOFF, &vtype) < 0) {
                fprintf(stderr, "[vtx] Pause STREAMOFF failed: %s\n", strerror(errno));
            }
            is_paused = true;
            first_frame = true;
            continue;
        }

        if (!want_pause && is_paused) {
            for (uint32_t i = 0; i < ctx->buf_count; ++i) {
                if (queue_v4l2_buf(ctx, i) < 0) {
                    fprintf(stderr, "[vtx] Resume QBUF[%u] failed: %s\n", i, strerror(errno));
                }
            }
            configure_sensor_exposure();
            enum v4l2_buf_type vtype = ctx->buf_type;
            if (xioctl(ctx->v4l2_fd, VIDIOC_STREAMON, &vtype) < 0) {
                fprintf(stderr, "[vtx] Resume STREAMON failed: %s\n", strerror(errno));
            }
            ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_IDR_FRAME, NULL);
            force_idr_hdr = true;
            is_paused = false;
            first_frame = true;
            continue;
        }

        if (is_paused) {
            struct pollfd wpfd = { .fd = ctx->wake_efd, .events = POLLIN };
            int r = poll(&wpfd, 1, -1);
            if (r > 0 && (wpfd.revents & POLLIN)) {
                uint64_t val;
                while (read(ctx->wake_efd, &val, sizeof(val)) > 0) {}
            }
            continue;
        }

        struct pollfd pfds[2] = {
            { .fd = ctx->v4l2_fd,  .events = POLLIN },
            { .fd = ctx->wake_efd, .events = POLLIN }
        };
        int poll_ret = poll(pfds, 2, 100);
        if (poll_ret <= 0) continue;

        if (pfds[1].revents & POLLIN) {
            uint64_t val;
            while (read(ctx->wake_efd, &val, sizeof(val)) > 0) {}
            if (atomic_load(&quit) || !atomic_load(&ctx->streaming) ||
                atomic_load(&ctx->want_paused)) {
                continue;
            }
        }

        if (pfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            usleep(5000);
            continue;
        }
        if (!(pfds[0].revents & POLLIN)) continue;

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
                usleep(5000);
            }
            continue;
        }

        if (buf.flags & V4L2_BUF_FLAG_ERROR) {
            fprintf(stderr, "[vtx] V4L2_BUF_FLAG_ERROR on seq=%u\n", buf.sequence);
        }

        if (!first_frame) {
            if (buf.sequence > last_seq + 1) {
                fprintf(stderr, ">>> CAMERA DROP: missed %u frame(s)! (seq %u vs expected %u) <<<\n",
                        buf.sequence - (last_seq + 1), buf.sequence, last_seq + 1);
            }
        } else {
            first_frame = false;
        }
        last_seq = buf.sequence;

        uint32_t idx = buf.index;
        if (idx >= ctx->buf_count) {
            fprintf(stderr, "[vtx] Invalid V4L2 buf index: %u\n", idx);
            continue;
        }

        if (cap_cnt == 0) {
            fprintf(stderr, "[vtx] first DQBUF ok (idx=%u)\n", idx);
        }

        /* Submit pre-bound MppFrame to MPP encoder */
        MppFrame frame = ctx->buffers[idx].mpp_frame;
        mpp_frame_set_pts(frame, (uint64_t)cap_cnt * 1000000ULL / ctx->cfg.fps);
        cap_cnt++;

        MPP_RET put_ret = ctx->mpi->encode_put_frame(ctx->mpp_ctx, frame);
        if (put_ret != MPP_OK) {
            fprintf(stderr, "[vtx] encode_put_frame failed: %d\n", put_ret);
            queue_v4l2_buf(ctx, idx);
            continue;
        }

        /* Retrieve encoded packet(s), requeue V4L2 buffer before USB I/O */
        bool buf_requeued = false;
        bool is_first_slice = true;
        while (!atomic_load(&quit) && atomic_load(&ctx->streaming)) {
            MppPacket packet = NULL;
            MPP_RET ret = ctx->mpi->encode_get_packet(ctx->mpp_ctx, &packet);
            if (ret != MPP_OK || !packet) {
                fprintf(stderr, "[vtx] encode_get_packet ret=%d pkt=%p\n", ret, (void *)packet);
                break;
            }

            void *ptr = mpp_packet_get_pos(packet);
            size_t len = mpp_packet_get_length(packet);
            int partition = mpp_packet_is_partition(packet);
            int eoi = mpp_packet_is_eoi(packet);
            int frame_complete = partition ? eoi : 1;

            if (frame_complete && !buf_requeued) {
                queue_v4l2_buf(ctx, idx);
                buf_requeued = true;
            }

            if (ptr && len > 0) {
                const uint8_t *u8 = (const uint8_t *)ptr;
                const uint8_t *send_hdr = NULL;
                size_t send_hdr_len = 0;

                /* Check if MPP already included HEVC VPS (NAL type 32) via EACH_IDR */
                bool has_vps = (len >= 5 &&
                                u8[0] == 0x00 && u8[1] == 0x00 &&
                                u8[2] == 0x00 && u8[3] == 0x01 &&
                                (((u8[4] >> 1) & 0x3F) == 32));

                if (is_first_slice) {
                    if (ctx->hdr_len > 0 && !has_vps &&
                        (frame_cnt == 0 || (frame_cnt % 60 == 0) || force_idr_hdr)) {
                        send_hdr = ctx->hdr_buf;
                        send_hdr_len = ctx->hdr_len;
                    }
                    force_idr_hdr = false;
                    is_first_slice = false;
                }

                int tx_bytes = bulk_write_frame(ctx, send_hdr, send_hdr_len, u8, len);
                if (frame_cnt < 2) {
                    fprintf(stderr, "[vtx] pkt frm=%u part=%d eoi=%d len=%zu hdr=%zu tx=%d\n",
                            frame_cnt, partition, eoi, len, send_hdr_len, tx_bytes);
                }
                if (tx_bytes < 0) {
                    if ((tx_err_cnt++ % 60U) == 0) {
                        fprintf(stderr, "[vtx] bulk_write_frame err: %s (len=%zu, count=%u)\n",
                                strerror(errno), send_hdr_len + len, tx_err_cnt);
                    }
                } else {
                    tx_err_cnt = 0;
                }
            }

            mpp_packet_deinit(&packet);

            if (frame_complete) {
                frame_cnt++;
                break;
            }
        }

        if (!buf_requeued) {
            queue_v4l2_buf(ctx, idx);
        }
    }

    usb_aio_deinit();
    return NULL;
}

static void cleanup_pipeline(VtxContext *ctx) {
    if (ctx->v4l2_fd >= 0) {
        enum v4l2_buf_type vtype = ctx->buf_type;
        xioctl(ctx->v4l2_fd, VIDIOC_STREAMOFF, &vtype);
        close(ctx->v4l2_fd);
        ctx->v4l2_fd = -1;
    }

    if (ctx->ref_cfg) {
        mpp_enc_ref_cfg_deinit(&ctx->ref_cfg);
        ctx->ref_cfg = NULL;
    }
    if (ctx->enc_cfg) {
        mpp_enc_cfg_deinit(ctx->enc_cfg);
        ctx->enc_cfg = NULL;
    }
    if (ctx->mpp_ctx) {
        mpp_destroy(ctx->mpp_ctx);
        ctx->mpp_ctx = NULL;
    }

    for (uint32_t i = 0; i < ctx->buf_count; ++i) {
        if (ctx->buffers[i].mpp_frame) {
            mpp_frame_deinit(&ctx->buffers[i].mpp_frame);
            ctx->buffers[i].mpp_frame = NULL;
        }
        if (ctx->buffers[i].mpp_buf) {
            mpp_buffer_put(ctx->buffers[i].mpp_buf);
            ctx->buffers[i].mpp_buf = NULL;
        }
    }
    if (ctx->buf_group) {
        mpp_buffer_group_put(ctx->buf_group);
        ctx->buf_group = NULL;
    }
}

static int start_pipeline(VtxContext *ctx) {
    ctx->v4l2_fd = open(ctx->cfg.v4l2_dev, O_RDWR | O_NONBLOCK, 0);
    if (ctx->v4l2_fd < 0) return -1;

    struct v4l2_capability cap;
    if (xioctl(ctx->v4l2_fd, VIDIOC_QUERYCAP, &cap) < 0) goto fail;

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

    if (xioctl(ctx->v4l2_fd, VIDIOC_S_FMT, &fmt) < 0) goto fail;

    uint32_t hor_stride = MPP_ALIGN(ctx->cfg.width, 16);
    uint32_t ver_stride = MPP_ALIGN(ctx->cfg.height, 16);
    size_t frame_buf_size = (size_t)hor_stride * ver_stride * 3 / 2;

    uint32_t drv_bpl = (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                       ? fmt.fmt.pix_mp.plane_fmt[0].bytesperline
                       : fmt.fmt.pix.bytesperline;
    uint32_t drv_size = (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                        ? fmt.fmt.pix_mp.plane_fmt[0].sizeimage
                        : fmt.fmt.pix.sizeimage;
    if (drv_bpl > hor_stride) hor_stride = drv_bpl;
    if (drv_size > frame_buf_size) frame_buf_size = drv_size;

    MPP_RET ret = mpp_buffer_group_get_internal(&ctx->buf_group, MPP_BUFFER_TYPE_ION);
    if (ret != MPP_OK) goto fail;

    ctx->buf_count = MAX_V4L2_BUFFERS;
    for (uint32_t i = 0; i < ctx->buf_count; ++i) {
        ret = mpp_buffer_get(ctx->buf_group, &ctx->buffers[i].mpp_buf, frame_buf_size);
        if (ret != MPP_OK) goto fail;
        ctx->buffers[i].dma_fd = mpp_buffer_get_fd(ctx->buffers[i].mpp_buf);
        ctx->buffers[i].length = frame_buf_size;

        ret = mpp_frame_init(&ctx->buffers[i].mpp_frame);
        if (ret != MPP_OK || !ctx->buffers[i].mpp_frame) goto fail;
        mpp_frame_set_width(ctx->buffers[i].mpp_frame, ctx->cfg.width);
        mpp_frame_set_height(ctx->buffers[i].mpp_frame, ctx->cfg.height);
        mpp_frame_set_hor_stride(ctx->buffers[i].mpp_frame, hor_stride);
        mpp_frame_set_ver_stride(ctx->buffers[i].mpp_frame, ver_stride);
        mpp_frame_set_fmt(ctx->buffers[i].mpp_frame, MPP_FMT_YUV420SP);
        mpp_frame_set_buffer(ctx->buffers[i].mpp_frame, ctx->buffers[i].mpp_buf);
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = ctx->buf_count;
    req.type = ctx->buf_type;
    req.memory = V4L2_MEMORY_DMABUF;

    if (xioctl(ctx->v4l2_fd, VIDIOC_REQBUFS, &req) < 0 || req.count != ctx->buf_count) {
        fprintf(stderr, "[vtx] VIDIOC_REQBUFS mismatch (got %u, expected %u)\n",
                req.count, ctx->buf_count);
        goto fail;
    }

    for (uint32_t i = 0; i < ctx->buf_count; ++i) {
        if (queue_v4l2_buf(ctx, i) < 0) goto fail;
    }

    ret = mpp_create(&ctx->mpp_ctx, &ctx->mpi);
    if (ret != MPP_OK) goto fail;

    /* Hardware interrupt output timeout (20ms) */
    RK_S64 out_timeout = 20;
    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_SET_OUTPUT_TIMEOUT, &out_timeout);
    if (ret != MPP_OK) goto fail;

    ret = mpp_init(ctx->mpp_ctx, MPP_CTX_ENC, (MppCodingType)ctx->cfg.codec_type);
    if (ret != MPP_OK) goto fail;

    ret = mpp_enc_cfg_init(&ctx->enc_cfg);
    if (ret != MPP_OK) goto fail;

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
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:gop", ctx->cfg.gop);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:max_reenc_times", 0);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:stats_time", 1);
    // mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:qp_init", 26);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:qp_min", 18);
    // mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:qp_max", 48);
    // mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:qp_min_i", 20);
    // mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:qp_max_i", 45);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:super_mode", MPP_ENC_RC_SUPER_FRM_NONE);

    /* P-frames only after initial/resumed IDR (no GIR) */
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:refresh_en", 0);

    mpp_enc_cfg_set_s32(ctx->enc_cfg, "codec:type", ctx->cfg.codec_type);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:profile", 1);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:scaling_list", 0);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:sao_luma_disable", 0);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:sao_chroma_disable", 0);

    mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:mode", MPP_ENC_SPLIT_NONE);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:arg", 0);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:out", 0);

    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_CFG, ctx->enc_cfg);
    if (ret != MPP_OK) {
        fprintf(stderr, "[vtx] MPP_ENC_SET_CFG failed: %d\n", ret);
        goto fail;
    }

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
    if (ret != MPP_OK) goto fail;

    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_REF_CFG, ctx->ref_cfg);
    if (ret != MPP_OK) goto fail;

    MppEncHeaderMode hdr_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
    ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_HEADER_MODE, &hdr_mode);

    MppEncSeiMode sei_mode = MPP_ENC_SEI_MODE_DISABLE;
    ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_SEI_CFG, &sei_mode);

    MppPacket hdr_pkt = NULL;
    ret = mpp_packet_init(&hdr_pkt, ctx->hdr_buf, sizeof(ctx->hdr_buf));
    if (ret != MPP_OK) goto fail;

    mpp_packet_set_length(hdr_pkt, 0);
    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_GET_HDR_SYNC, hdr_pkt);
    if (ret != MPP_OK) {
        ctx->hdr_len = 0;
    } else {
        ctx->hdr_len = mpp_packet_get_length(hdr_pkt);
        fprintf(stderr, ">>> H.265 sync header: %zu bytes <<<\n", ctx->hdr_len);
    }
    mpp_packet_deinit(&hdr_pkt);

    /* Configure sensor exposure BEFORE stream start */
    configure_sensor_exposure();

    enum v4l2_buf_type type = ctx->buf_type;
    if (xioctl(ctx->v4l2_fd, VIDIOC_STREAMON, &type) < 0) goto fail;

    atomic_store(&ctx->streaming, true);
    if (pthread_create(&ctx->stream_thd, NULL, vtx_stream_thread, ctx) != 0) {
        atomic_store(&ctx->streaming, false);
        goto fail;
    }
    return 0;

fail:
    cleanup_pipeline(ctx);
    return -1;
}

static void stop_pipeline(VtxContext *ctx) {
    if (!atomic_load(&ctx->streaming)) return;

    atomic_store(&ctx->streaming, false);
    atomic_store(&ctx->want_paused, false);
    wake_stream_thread(ctx);

    pthread_join(ctx->stream_thd, NULL);
    cleanup_pipeline(ctx);
}

int main(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigterm_handler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    signal(SIGHUP, SIG_IGN);
    signal(SIGPIPE, SIG_IGN);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
        fprintf(stderr, "Warning: mlockall failed: %s\n", strerror(errno));
    }

    VtxContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.v4l2_fd = -1;
    ctx.ep1_fd = -1;
    ctx.wake_efd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (ctx.wake_efd < 0) {
        fprintf(stderr, "Fatal: eventfd failed: %s\n", strerror(errno));
        return 1;
    }
    g_wake_efd = ctx.wake_efd;

    strncpy(ctx.cfg.v4l2_dev, "/dev/video12", sizeof(ctx.cfg.v4l2_dev) - 1);
    ctx.cfg.width = 1280;
    ctx.cfg.height = 720;
    ctx.cfg.fps = 60;
    ctx.cfg.bitrate_kbps = 2350;
    ctx.cfg.gop = 0x7FFFFFFF; /* Infinite GOP (P-frames only after IDR) */
    ctx.cfg.codec_type = MPP_VIDEO_CodingHEVC;

    /* 1. Initialize USB FunctionFS Vendor Endpoint */
    if (open_vtx_endpoint(&ctx) < 0) {
        fprintf(stderr, "Fatal: Failed to initialize FunctionFS\n");
        close(ctx.wake_efd);
        return 1;
    }

    /* 2. Open SARADC Pushbutton */
    const char *adc_path = "/sys/bus/iio/devices/iio:device0/in_voltage0_raw";
    int adc_fd = open(adc_path, O_RDONLY);
    if (adc_fd >= 0) {
        fprintf(stderr, ">>> Onboard Pushbutton Armed (SARADC) <<<\n");
    } else {
        fprintf(stderr, "Warning: Could not open '%s': %s\n", adc_path, strerror(errno));
    }

    fprintf(stderr, ">>> Starting VTX camera and encoder pipeline <<<\n");
    // Elevate temporarily so spawned mpp_h265e thread inherits SCHED_FIFO 80
    struct sched_param rt_sp = { .sched_priority = 80 };
    struct sched_param normal_sp = { .sched_priority = 0 };
    if (sched_setscheduler(0, SCHED_FIFO, &rt_sp) < 0) {
        fprintf(stderr, "Warning: main SCHED_FIFO failed: %s\n", strerror(errno));
    }
    int pipe_ret = start_pipeline(&ctx);
    sched_setscheduler(0, SCHED_OTHER, &normal_sp);
    if (pipe_ret < 0) {
        fprintf(stderr, "Fatal: start_pipeline failed!\n");
        if (adc_fd >= 0) close(adc_fd);
        close(ctx.ep1_fd);
        close(ctx.wake_efd);
        return 1;
    }

    int stdin_has_tty = isatty(STDIN_FILENO);
    int stdin_orig_flags = -1;
    if (stdin_has_tty) {
        stdin_orig_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (stdin_orig_flags >= 0) {
            fcntl(STDIN_FILENO, F_SETFL, stdin_orig_flags | O_NONBLOCK);
        }
    }

    struct timespec last_toggle_ts = { 0, 0 };
    bool btn_last_pressed = false;

    while (!atomic_load(&quit)) {
        struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN };
        int ret = poll(stdin_has_tty ? &pfd : NULL, stdin_has_tty ? 1 : 0, 100);

        bool toggle = false;

        /* Check ADC Pushbutton with hysteresis (<200 press, >500 release) */
        if (adc_fd >= 0) {
            char val_str[16];
            ssize_t n = pread(adc_fd, val_str, sizeof(val_str) - 1, 0);
            if (n > 0) {
                val_str[n] = '\0';
                int val = atoi(val_str);
                bool pressed = btn_last_pressed ? (val <= 500) : (val < 200);
                if (pressed && !btn_last_pressed) {
                    toggle = true;
                }
                btn_last_pressed = pressed;
            }
        }

        /* Check SSH keyboard (disable polling cleanly if TTY disconnects) */
        if (stdin_has_tty && ret > 0) {
            if (pfd.revents & POLLIN) {
                char dummy[64];
                ssize_t r;
                while ((r = read(STDIN_FILENO, dummy, sizeof(dummy))) > 0) {
                    toggle = true;
                }
                if (r == 0) {
                    stdin_has_tty = 0;
                }
            }
            if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
                stdin_has_tty = 0;
            }
        }

        if (toggle) {
            struct timespec now_ts;
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            long elapsed_ms = (now_ts.tv_sec - last_toggle_ts.tv_sec) * 1000L +
                              (now_ts.tv_nsec - last_toggle_ts.tv_nsec) / 1000000L;
            if (elapsed_ms < 300) {
                continue;
            }
            last_toggle_ts = now_ts;

            bool next_paused = !atomic_load(&ctx.want_paused);
            atomic_store(&ctx.want_paused, next_paused);
            wake_stream_thread(&ctx);

            fprintf(stderr, next_paused
                    ? ">>> VTX PAUSED: Stopping camera stream <<<\n"
                    : ">>> VTX RESUMED: Restarting camera stream & requesting IDR <<<\n");
        }
    }

    stop_pipeline(&ctx);

    if (adc_fd >= 0) close(adc_fd);
    if (ctx.ep1_fd >= 0) close(ctx.ep1_fd);
    if (ctx.wake_efd >= 0) close(ctx.wake_efd);
    if (stdin_has_tty && stdin_orig_flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, stdin_orig_flags);
    }

    return 0;
}