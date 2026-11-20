/*
 * Dedicated Low-Latency 720p60 USB Vendor Bulk Streamer (VTX)
 * Linux FunctionFS (Vendor Class 0xFF) Bulk IN Endpoint Pipeline
 * Pure N-1 Short-Term Reference Pipeline with GIR
 * Infinite GOP with periodic header resync for mid-stream decoders
 */

#define _GNU_SOURCE
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
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
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <linux/aio_abi.h>
#include <time.h>
#include <unistd.h>
#include <linux/dma-buf.h>
#include <linux/gpio.h>
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

#define MAX_V4L2_BUFFERS   3
#define FIFO_SIZE          32
#define MPP_ALIGN(x, a)    (((x) + (a) - 1) & ~((a) - 1))

#define FFS_EP0_PATH       "/dev/usb-ffs/vtx/ep0"
#define FFS_EP1_PATH       "/dev/usb-ffs/vtx/ep1"
#define GADGET_UDC_PATH    "/sys/kernel/config/usb_gadget/g1/UDC"

struct ffs_desc_full {
    struct usb_functionfs_descs_head_v2 header;
    __le32 fs_count;
    __le32 hs_count;
    struct {
        struct usb_interface_descriptor intf;
        struct {
            __u8  bLength;
            __u8  bDescriptorType;
            __u8  bEndpointAddress;
            __u8  bmAttributes;
            __le16 wMaxPacketSize;
            __u8  bInterval;
        } __attribute__((packed)) bulk_in;
    } __attribute__((packed)) fs_descs, hs_descs;
} __attribute__((packed));

struct ffs_strings_full {
    __le32 magic;
    __le32 length;
    __le32 str_count;
    __le32 lang_count;
    struct {
        __le16 code;
        char str1[18];
    } __attribute__((packed)) tab;
} __attribute__((packed));

typedef struct {
    char        v4l2_dev[64];
    uint32_t    width;
    uint32_t    height;
    uint32_t    fps;
    uint32_t    bitrate_kbps;
    uint32_t    gop;
    int         codec_type;
    bool        autostart;
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

    int             ep0_fd;
    int             ep1_fd;
    pthread_t       ep0_thd;
    pthread_t       cap_thd;
    pthread_t       tx_thd;

    atomic_bool     streaming;
    atomic_bool     paused;
    atomic_bool     request_idr;
    pthread_mutex_t pause_lock;
    pthread_cond_t  pause_cond;
    bool            cap_parked;
    bool            tx_parked;
} VtxContext;

static atomic_bool quit = false;

static void sigterm_handler(int sig) {
    (void)sig;
    atomic_store(&quit, true);
}

static int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}


#define USB_TX_SLOTS     2
#define USB_TX_BUF_SIZE  (256 * 1024)

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
        /* Give any in-flight USB transfer up to 50ms to finish cleanly */
        struct io_event events[USB_TX_SLOTS];
        struct aio_old_timespec ts = { .tv_sec = 0, .tv_nsec = 50000000L };
        for (int i = 0; i < USB_TX_SLOTS; i++) {
            if (s_usb_aio.busy[i]) {
                syscall(SYS_io_getevents, s_usb_aio.aio_ctx, 1, USB_TX_SLOTS, events, &ts);
                break;
            }
        }
        syscall(SYS_io_destroy, s_usb_aio.aio_ctx);
        s_usb_aio.aio_ctx = 0;
    }
}

static void usb_aio_reap(long wait_ns) {
    struct io_event events[USB_TX_SLOTS];
    struct aio_old_timespec ts = { .tv_sec = 0, .tv_nsec = wait_ns };
    long min_nr = (wait_ns > 0) ? 1 : 0;

    int n;
    do {
        n = (int)syscall(SYS_io_getevents, s_usb_aio.aio_ctx,
                         min_nr, USB_TX_SLOTS, events, &ts);
    } while (n < 0 && errno == EINTR &&
             !atomic_load(&quit));

    for (int i = 0; i < n; i++) {
        int slot = (int)events[i].data;
        if (events[i].res < 0) {
            fprintf(stderr, "[vtx] USB AIO completion err: %lld\n", (long long)events[i].res);
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

    static const uint8_t hevc_aud[7] = { 0x00, 0x00, 0x00, 0x01, 0x46, 0x01, 0x50 };
    size_t total = hdr_len + slice_len + sizeof(hevc_aud);
    if (total > USB_TX_BUF_SIZE - 2)
        return -1;

    /* 1. Non-blocking reap of any finished USB transfers */
    usb_aio_reap(0);

    int slot = -1;
    for (int i = 0; i < USB_TX_SLOTS; i++) {
        if (!s_usb_aio.busy[i]) {
            slot = i;
            break;
        }
    }

    /* 2. If all slots are busy, wait up to 100ms for 1 slot to finish */
    if (slot < 0) {
        usb_aio_reap(100000000L);
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

    /* 3. Stage packet into slot buffer and enforce short-packet termination */
    uint8_t *dst = s_usb_aio.buf[slot];
    if (hdr && hdr_len > 0)
        memcpy(dst, hdr, hdr_len);
    memcpy(dst + hdr_len, slice, slice_len);
    memcpy(dst + hdr_len + slice_len, hevc_aud, sizeof(hevc_aud));

    //if ((total % 64) == 0)
    //    dst[total++] = 0x00;

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
        fprintf(stderr, "[vtx] io_submit failed: %s\n", strerror(errno));
        return -1;
    }

    s_usb_aio.busy[slot] = true;
    return (int)total;
}




static int open_vtx_endpoint(VtxContext *ctx) {
    ctx->ep1_fd = open(FFS_EP1_PATH, O_RDWR | O_NONBLOCK);

    if (ctx->ep1_fd < 0) {
        fprintf(stderr,
                "ERROR: Cannot open '%s': %s\n",
                FFS_EP1_PATH, strerror(errno));
        return -1;
    }

    // Ask FunctionFS for the actual endpoint descriptor.
    struct usb_endpoint_descriptor ep_desc;
    memset(&ep_desc, 0, sizeof(ep_desc));

    if (ioctl(ctx->ep1_fd, FUNCTIONFS_ENDPOINT_DESC, &ep_desc) == 0) {

        uint16_t max_packet = le16toh(ep_desc.wMaxPacketSize);

        fprintf(stderr,
                "[USB DIAG] FunctionFS ep1 actual address: 0x%02X\n",
                ep_desc.bEndpointAddress);

        fprintf(stderr,
                "[USB DIAG] Transfer type: %u, Max packet: %u\n",
                ep_desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK,
                max_packet);

        if (ep_desc.bEndpointAddress != 0x85) {
            fprintf(stderr,
                    "[USB DIAG] WARNING: RP2350 currently expects 0x85!\n");
        } else {
            fprintf(stderr,
                    "[USB DIAG] Endpoint matches RP2350 expectation.\n");
        }

    } else {
        fprintf(stderr,
                "[USB DIAG] Cannot query endpoint descriptor: %s\n",
                strerror(errno));
    }

    fprintf(stderr, ">>> VTX connected to USB Bulk Endpoint <<<\n");

    return 0;
}


#include <linux/i2c-dev.h>

static int set_sensor_standby(bool standby) {
    int fd = open("/dev/i2c-4", O_RDWR);
    if (fd < 0) return -1;

    if (ioctl(fd, I2C_SLAVE_FORCE, 0x1a) < 0) {
        close(fd);
        return -1;
    }

    /* 16-bit register writes: [Addr_MSB, Addr_LSB, Value] */
    uint8_t xmsta_cmd[3]   = { 0x30, 0x02, standby ? 0x01 : 0x00 };
    uint8_t standby_cmd[3] = { 0x30, 0x00, standby ? 0x01 : 0x00 };

    if (standby) {
        /* Stop clock first, wait > 1 frame (16.7ms @ 60fps), then enter standby */
        if (write(fd, xmsta_cmd, 3) != 3) { close(fd); return -1; }
        usleep(18000);
        if (write(fd, standby_cmd, 3) != 3) { close(fd); return -1; }
    } else {
        /* Exit standby first, wait for sensor PLL, then start clock */
        if (write(fd, standby_cmd, 3) != 3) { close(fd); return -1; }
        usleep(30000);
        if (write(fd, xmsta_cmd, 3) != 3) { close(fd); return -1; }
    }

    close(fd);
    return 0;
}


/* ----------------------------------------------------------------------------
 * THREAD 1: Full-Speed Camera Capture (60 FPS)
 * ---------------------------------------------------------------------------- */
static void *camera_capture_thread(void *arg) {
    VtxContext *ctx = (VtxContext *)arg;
    prctl(PR_SET_NAME, "vtx_cap", 0, 0, 0);

    struct sched_param sp = { .sched_priority = 85 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    uint32_t hor_stride = MPP_ALIGN(ctx->cfg.width, 16);
    uint32_t ver_stride = MPP_ALIGN(ctx->cfg.height, 16);
    uint32_t cap_cnt = 0;
    
    uint32_t last_seq = 0;
    bool first_frame = true;

    MppFrame frame = NULL;
    if (mpp_frame_init(&frame) != MPP_OK || !frame) {
        return NULL;
    }
    mpp_frame_set_width(frame, ctx->cfg.width);
    mpp_frame_set_height(frame, ctx->cfg.height);
    mpp_frame_set_hor_stride(frame, hor_stride);
    mpp_frame_set_ver_stride(frame, ver_stride);
    mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);

    while (!atomic_load(&quit) && atomic_load(&ctx->streaming)) {
        /* When paused, park completely (0 wakeups/sec) until resumed or stopped */
        if (atomic_load(&ctx->paused)) {
            first_frame = true;
            pthread_mutex_lock(&ctx->pause_lock);
            ctx->cap_parked = true;
            pthread_cond_broadcast(&ctx->pause_cond);
            while (atomic_load(&ctx->paused) &&
                   atomic_load(&ctx->streaming) &&
                   !atomic_load(&quit)) {
                pthread_cond_wait(&ctx->pause_cond, &ctx->pause_lock);
            }
            ctx->cap_parked = false;
            pthread_mutex_unlock(&ctx->pause_lock);
            continue;
        }

        struct pollfd pfd = { .fd = ctx->v4l2_fd, .events = POLLIN };
        int poll_ret = poll(&pfd, 1, 30);
        if (poll_ret <= 0) continue;

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            usleep(5000);
            continue;
        }

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

        if (cap_cnt == 0) {
          fprintf(stderr, "[vtx] first DQBUF ok (idx=%u)\n", idx);
      }

        /* If paused, return the buffer to the camera and do NOT pass to encoder */
        if (atomic_load(&ctx->paused)) {
            if (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
                planes[0].m.fd = ctx->buffers[idx].dma_fd;
                planes[0].length = ctx->buffers[idx].length;
            } else {
                buf.m.fd = ctx->buffers[idx].dma_fd;
            }
            xioctl(ctx->v4l2_fd, VIDIOC_QBUF, &buf);
            first_frame = true;
            continue;
        }

        mpp_frame_set_buffer(frame, ctx->buffers[idx].mpp_buf);
        mpp_frame_set_pts(frame, (uint64_t)cap_cnt * 1000000ULL / ctx->cfg.fps);

        /* Register buffer index into FIFO before submit */
        pthread_mutex_lock(&ctx->fifo_lock);
        ctx->fifo[ctx->fifo_head] = idx;
        ctx->fifo_head = (ctx->fifo_head + 1) % FIFO_SIZE;
        pthread_mutex_unlock(&ctx->fifo_lock);

        /* Submit frame to MPP encoder */
        MPP_RET put_ret = ctx->mpi->encode_put_frame(ctx->mpp_ctx, frame);
        if (put_ret != MPP_OK) {
            fprintf(stderr, "[vtx] encode_put_frame failed: %d\n", put_ret);
            /* Roll back FIFO and return buffer immediately to camera if rejected */
            pthread_mutex_lock(&ctx->fifo_lock);
            ctx->fifo_head = (ctx->fifo_head + FIFO_SIZE - 1) % FIFO_SIZE;
            pthread_mutex_unlock(&ctx->fifo_lock);

            if (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
                planes[0].m.fd = ctx->buffers[idx].dma_fd;
                planes[0].length = ctx->buffers[idx].length;
            } else {
                buf.m.fd = ctx->buffers[idx].dma_fd;
            }
            xioctl(ctx->v4l2_fd, VIDIOC_QBUF, &buf);
        }
        cap_cnt++;
    }
    mpp_frame_deinit(&frame);
    return NULL;
}

/* ----------------------------------------------------------------------------
 * THREAD 2: USB Bulk Output & Safe V4L2 Buffer Requeue
 * ---------------------------------------------------------------------------- */
static void *venc_tx_thread(void *arg) {
    VtxContext *ctx = (VtxContext *)arg;
    prctl(PR_SET_NAME, "vtx_tx", 0, 0, 0);

    struct sched_param sp = { .sched_priority = 80 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    uint32_t frame_cnt = 0;
    bool is_first_slice = true;
    size_t total_bytes = 0;
    struct timespec last_time, cur_time;
    clock_gettime(CLOCK_MONOTONIC, &last_time);

    if (usb_aio_init() < 0) {
        return NULL;
    }

    while (!atomic_load(&quit) && atomic_load(&ctx->streaming)) {
        /* Once capture is parked and FIFO is empty, park TX thread cleanly (0 wakeups/sec) */
        if (atomic_load(&ctx->paused)) {
            pthread_mutex_lock(&ctx->fifo_lock);
            bool fifo_empty = (ctx->fifo_head == ctx->fifo_tail);
            pthread_mutex_unlock(&ctx->fifo_lock);

            if (fifo_empty) {
                pthread_mutex_lock(&ctx->pause_lock);
                if (ctx->cap_parked) {
                    ctx->tx_parked = true;
                    pthread_cond_broadcast(&ctx->pause_cond);
                    while (atomic_load(&ctx->paused) &&
                           atomic_load(&ctx->streaming) &&
                           !atomic_load(&quit)) {
                        pthread_cond_wait(&ctx->pause_cond, &ctx->pause_lock);
                    }
                    ctx->tx_parked = false;
                    is_first_slice = true;
                    pthread_mutex_unlock(&ctx->pause_lock);
                    continue;
                }
                pthread_mutex_unlock(&ctx->pause_lock);
            }
        }
        MppPacket packet = NULL;
        MPP_RET ret = ctx->mpi->encode_get_packet(ctx->mpp_ctx, &packet);

        if (ret != MPP_OK || !packet) {
          static int err_logged = 0;
          if (!err_logged) {
              fprintf(stderr, "[vtx] encode_get_packet ret=%d pkt=%p\n", ret, (void *)packet);
              err_logged = 1;
          }
          if (ret != MPP_OK && ret != MPP_ERR_TIMEOUT) {
              usleep(5000);
          }
          continue;
      }

        void *ptr = mpp_packet_get_pos(packet);
        size_t len = mpp_packet_get_length(packet);
        int partition = mpp_packet_is_partition(packet);
        int eoi = mpp_packet_is_eoi(packet);
        int frame_complete = partition ? eoi : 1;

        /* 1. Requeue V4L2 input buffer BEFORE blocking on USB I/O */
        if (frame_complete) {
            pthread_mutex_lock(&ctx->fifo_lock);
            if (ctx->fifo_head != ctx->fifo_tail) {
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
            } else {
                pthread_mutex_unlock(&ctx->fifo_lock);
            }
        }

        /* 2. Transmit packet over USB while MppPacket is still valid */
        if (ptr && len > 0 && !atomic_load(&ctx->paused)) {
            const uint8_t *u8 = (const uint8_t *)ptr;
            const uint8_t *send_hdr = NULL;
            size_t send_hdr_len = 0;

            /* Check if MPP already included HEVC VPS (NAL type 32) via EACH_IDR */
            bool has_vps = (len >= 5 &&
                            u8[0] == 0x00 && u8[1] == 0x00 &&
                            u8[2] == 0x00 && u8[3] == 0x01 &&
                            (((u8[4] >> 1) & 0x3F) == 32));

            if (is_first_slice) {
                bool want_idr_hdr = atomic_exchange(&ctx->request_idr, false);
                if (ctx->hdr_len > 0 && !has_vps &&
                    (frame_cnt == 0 || (frame_cnt % 60 == 0) || want_idr_hdr)) {
                    send_hdr = ctx->hdr_buf;
                    send_hdr_len = ctx->hdr_len;
                }
                is_first_slice = false;
            }

            int tx_bytes = bulk_write_frame(ctx, send_hdr, send_hdr_len, u8, len);
            if (frame_cnt < 2) {
                fprintf(stderr, "[vtx] pkt frm=%u part=%d eoi=%d len=%zu hdr=%zu tx=%d\n",
                        frame_cnt, partition, eoi, len, send_hdr_len, tx_bytes);
            }
            if (tx_bytes > 0) {
                total_bytes += (size_t)tx_bytes;
            } else if (tx_bytes < 0) {
                fprintf(stderr, "[vtx] bulk_write_frame err: %s\n", strerror(errno));
            }
        }

        mpp_packet_deinit(&packet);

        if (frame_complete) {
            frame_cnt++;
            is_first_slice = true;
        }
    }
    usb_aio_deinit();
    return NULL;
}

static void configure_sensor_exposure(void) {
    int fd = open("/dev/v4l-subdev2", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Warning: Could not open /dev/v4l-subdev2: %s\n", strerror(errno));
        return;
    }

    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));

    ctrl.id = V4L2_CID_EXPOSURE;
    ctrl.value = 250;
    int ret_exp = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

    /* Read back active exposure to verify */
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_EXPOSURE;
    int ret_get = ioctl(fd, VIDIOC_G_CTRL, &ctrl);

    close(fd);
    fprintf(stderr, "[sensor-diag] Set exposure ret=%d, readback active exposure=%d (ret=%d)\n",
            ret_exp, ctrl.value, ret_get);
}

static int start_pipeline(VtxContext *ctx) {
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


    ret = mpp_create(&ctx->mpp_ctx, &ctx->mpi);
    if (ret != MPP_OK) return -1;

    /* 1. Configure sensor low-latency exposure automatically */

    /* 2. Hardware interrupt output timeout (20ms) - Zero CPU usage while encoding */
    RK_S64 out_timeout = 20;
    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_SET_OUTPUT_TIMEOUT, &out_timeout);
    if (ret != MPP_OK) return -1;

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
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:gop", ctx->cfg.gop);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:max_reenc_times", 0);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:stats_time", 1);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "rc:qp_min", 18);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:super_mode", MPP_ENC_RC_SUPER_FRM_NONE);

    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:refresh_en", 0);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:refresh_mode", 0);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "rc:refresh_num", 11);

    mpp_enc_cfg_set_s32(ctx->enc_cfg, "codec:type", ctx->cfg.codec_type);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:profile", 1);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:scaling_list", 0);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:sao_luma_disable", 0);
    mpp_enc_cfg_set_s32(ctx->enc_cfg, "h265:sao_chroma_disable", 0);
    // mpp_enc_cfg_set_s32(ctx->enc_cfg, "base:low_delay", 1); 

    mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:mode", MPP_ENC_SPLIT_NONE);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:arg", 0);
    mpp_enc_cfg_set_u32(ctx->enc_cfg, "split:out", 0);


    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_CFG, ctx->enc_cfg);
    if (ret != MPP_OK) {
        fprintf(stderr, "[vtx] MPP_ENC_SET_CFG failed: %d\n", ret);
        return -1;
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
    if (ret != MPP_OK) return -1;

    ret = ctx->mpi->control(ctx->mpp_ctx, MPP_ENC_SET_REF_CFG, ctx->ref_cfg);
    if (ret != MPP_OK) return -1;

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
        fprintf(stderr, ">>> H.265 sync header: %zu bytes <<<\n", ctx->hdr_len);
    }
    mpp_packet_deinit(&hdr_pkt);

    /* Configure sensor exposure BEFORE stream start */
    configure_sensor_exposure();

    /* Start camera stream IMMEDIATELY before launching capture thread (NO SLEEP!) */
    enum v4l2_buf_type type = ctx->buf_type;
    if (xioctl(ctx->v4l2_fd, VIDIOC_STREAMON, &type) < 0) return -1;

    atomic_store(&ctx->streaming, true);
    if (pthread_create(&ctx->tx_thd, NULL, venc_tx_thread, ctx) != 0) {
        atomic_store(&ctx->streaming, false);
        xioctl(ctx->v4l2_fd, VIDIOC_STREAMOFF, &type);
        return -1;
    }
    if (pthread_create(&ctx->cap_thd, NULL, camera_capture_thread, ctx) != 0) {
        atomic_store(&ctx->streaming, false);
        pthread_kill(ctx->tx_thd, SIGUSR1);
        pthread_join(ctx->tx_thd, NULL);
        xioctl(ctx->v4l2_fd, VIDIOC_STREAMOFF, &type);
        return -1;
    }
    return 0;
}

static void stop_pipeline(VtxContext *ctx) {
    if (!atomic_load(&ctx->streaming)) return;
    atomic_store(&ctx->streaming, false);

    pthread_mutex_lock(&ctx->pause_lock);
    atomic_store(&ctx->paused, false);
    pthread_cond_broadcast(&ctx->pause_cond);
    pthread_mutex_unlock(&ctx->pause_lock);

    /* 1. Wake up worker threads */
    pthread_kill(ctx->cap_thd, SIGUSR1);
    pthread_kill(ctx->tx_thd, SIGUSR1);

    /* 2. Join threads cleanly */
    pthread_join(ctx->cap_thd, NULL);
    pthread_join(ctx->tx_thd, NULL);

    /* 3. Stop camera hardware stream */
    if (ctx->v4l2_fd >= 0) {
        enum v4l2_buf_type vtype = ctx->buf_type;
        xioctl(ctx->v4l2_fd, VIDIOC_STREAMOFF, &vtype);
        close(ctx->v4l2_fd);
        ctx->v4l2_fd = -1;
    }

    /* 4. Destroy MPP encoder */
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

    /* 5. Safely release camera buffers */
    for (uint32_t i = 0; i < ctx->buf_count; ++i) {
        if (ctx->buffers[i].mpp_buf) {
            mpp_buffer_put(ctx->buffers[i].mpp_buf);
            ctx->buffers[i].mpp_buf = NULL;
        }
    }
    if (ctx->buf_group) {
        mpp_buffer_group_put(ctx->buf_group);
        ctx->buf_group = NULL;
    }

    /* Reset FIFO */
    pthread_mutex_lock(&ctx->fifo_lock);
    ctx->fifo_head = 0;
    ctx->fifo_tail = 0;
    pthread_mutex_unlock(&ctx->fifo_lock);
}

static void sigusr1_handler(int sig) {
    (void)sig;
}

int main(int argc, char **argv) {
    signal(SIGINT, sigterm_handler);
    signal(SIGTERM, sigterm_handler);
    signal(SIGHUP, SIG_IGN);
    signal(SIGUSR1, sigusr1_handler);

    signal(SIGPIPE, SIG_IGN);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
        fprintf(stderr, "Warning: mlockall failed: %s\n", strerror(errno));
    }

    VtxContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.v4l2_fd = -1;
    ctx.ep1_fd = -1;
    pthread_mutex_init(&ctx.fifo_lock, NULL);
    pthread_mutex_init(&ctx.pause_lock, NULL);
    pthread_cond_init(&ctx.pause_cond, NULL);

    strncpy(ctx.cfg.v4l2_dev, "/dev/video11", sizeof(ctx.cfg.v4l2_dev) - 1);
    ctx.cfg.width = 1280;
    ctx.cfg.height = 720;
    ctx.cfg.fps = 60;
    ctx.cfg.bitrate_kbps = 2350;
    ctx.cfg.gop = 0x7FFFFFFF; // 65536
    ctx.cfg.codec_type = MPP_VIDEO_CodingHEVC;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-i") || !strcmp(argv[i], "--immediate"))
            ctx.cfg.autostart = true;
    }
    if (getenv("VTX_AUTOSTART") && atoi(getenv("VTX_AUTOSTART")) != 0)
        ctx.cfg.autostart = true;

    /* 1. Initialize USB FunctionFS Vendor Endpoint & Bind Gadget */
    if (open_vtx_endpoint(&ctx) < 0) {
        fprintf(stderr, "Fatal: Failed to initialize FunctionFS\n");
        return 1;
    }

    /* 2. Standby / Arming Loop (Onboard SARADC Button + STDIN keyboard trigger) */
    const char *adc_path = "/sys/bus/iio/devices/iio:device0/in_voltage0_raw";
    int adc_fd = open(adc_path, O_RDONLY);
    if (adc_fd >= 0) {
        fprintf(stderr, ">>> Onboard Pushbutton Armed (SARADC) <<<\n");
    } else {
        fprintf(stderr, "Warning: Could not open '%s': %s\n", adc_path, strerror(errno));
    }

    fprintf(stderr, ">>> Starting VTX camera and encoder pipeline <<<\n");
    if (start_pipeline(&ctx) < 0) {
        fprintf(stderr, "Fatal: start_pipeline failed!\n");
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
        struct pollfd pfd;
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;

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

        /* Check SSH keyboard */
        if (stdin_has_tty && ret > 0 && (pfd.revents & POLLIN)) {
            char dummy[64];
            while (read(STDIN_FILENO, dummy, sizeof(dummy)) > 0) {}
            toggle = true;
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
          bool next_paused = !atomic_load(&ctx.paused);
          if (next_paused) {
              atomic_store(&ctx.paused, true);
              fprintf(stderr, ">>> VTX PAUSED: Parking threads & stopping camera stream <<<\n");

              /* Wait for both threads to park with an empty FIFO (bounded 50ms check) */
              pthread_mutex_lock(&ctx.pause_lock);
              while (!(ctx.cap_parked && ctx.tx_parked) && !atomic_load(&quit)) {
                  struct timespec ts;
                  clock_gettime(CLOCK_REALTIME, &ts);
                  ts.tv_nsec += 50000000L;
                  if (ts.tv_nsec >= 1000000000L) {
                      ts.tv_sec += 1;
                      ts.tv_nsec -= 1000000000L;
                  }
                  pthread_cond_timedwait(&ctx.pause_cond, &ctx.pause_lock, &ts);
              }
              pthread_mutex_unlock(&ctx.pause_lock);

              if (!atomic_load(&quit) && ctx.v4l2_fd >= 0) {
                  enum v4l2_buf_type vtype = ctx.buf_type;
                  if (xioctl(ctx.v4l2_fd, VIDIOC_STREAMOFF, &vtype) < 0) {
                      fprintf(stderr, "[vtx] Pause STREAMOFF failed: %s\n", strerror(errno));
                  }
              }
          } else {
              fprintf(stderr, ">>> VTX RESUMED: Restarting camera stream & requesting IDR <<<\n");
              if (ctx.v4l2_fd >= 0) {
                  /* V4L2 STREAMOFF empties the driver queue, so re-queue all buffers before STREAMON */
                  for (uint32_t i = 0; i < ctx.buf_count; ++i) {
                      struct v4l2_buffer buf;
                      struct v4l2_plane planes[1];
                      memset(&buf, 0, sizeof(buf));
                      memset(planes, 0, sizeof(planes));

                      buf.type = ctx.buf_type;
                      buf.memory = V4L2_MEMORY_DMABUF;
                      buf.index = i;

                      if (ctx.buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
                          planes[0].m.fd = ctx.buffers[i].dma_fd;
                          planes[0].length = ctx.buffers[i].length;
                          buf.m.planes = planes;
                          buf.length = 1;
                      } else {
                          buf.m.fd = ctx.buffers[i].dma_fd;
                          buf.length = ctx.buffers[i].length;
                      }

                      if (xioctl(ctx.v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
                          fprintf(stderr, "[vtx] Resume QBUF[%u] failed: %s\n", i, strerror(errno));
                      }
                  }

                  enum v4l2_buf_type vtype = ctx.buf_type;
                  if (xioctl(ctx.v4l2_fd, VIDIOC_STREAMON, &vtype) < 0) {
                      fprintf(stderr, "[vtx] Resume STREAMON failed: %s\n", strerror(errno));
                  }
              }

              atomic_store(&ctx.request_idr, true);
              if (ctx.mpp_ctx && ctx.mpi) {
                  ctx.mpi->control(ctx.mpp_ctx, MPP_ENC_SET_IDR_FRAME, NULL);
              }

              pthread_mutex_lock(&ctx.pause_lock);
              atomic_store(&ctx.paused, false);
              pthread_cond_broadcast(&ctx.pause_cond);
              pthread_mutex_unlock(&ctx.pause_lock);
          }
      }
    }

    if (atomic_load(&ctx.streaming)) stop_pipeline(&ctx);
    if (adc_fd >= 0) close(adc_fd);
    if (ctx.ep1_fd >= 0) close(ctx.ep1_fd);
    if (stdin_has_tty && stdin_orig_flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, stdin_orig_flags);
    }

    pthread_mutex_destroy(&ctx.fifo_lock);
    pthread_cond_destroy(&ctx.pause_cond);
    pthread_mutex_destroy(&ctx.pause_lock);
    return 0;
}