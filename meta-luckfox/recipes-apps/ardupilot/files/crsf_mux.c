// crsf_mux.c  (compile with: arm-rockchip830-linux-uclibcgnueabihf-gcc -O2 -pthread crsf_mux.c -o crsf_mux)
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define CRSF_MAX_FRAME              64
#define CRSF_SYNC_BYTE              0xC8
#define CRSF_SYNC_EDGETX            0xEA
#define CRSF_SYNC_FC                0xEC
#define CRSF_FRAMETYPE_TARGET_CMD   0x81
#define CRSF_FRAMETYPE_TARGET_TELEM 0x82

#define FFS_CRSF_IN_EP              "/dev/usb-ffs/crsf/ep1" // IN  to Host
#define FFS_CRSF_OUT_EP             "/dev/usb-ffs/crsf/ep2" // OUT from Host

static uint8_t crc8_dvb_s2(const uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

static int set_raw_tty(int fd) {
    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) return -1;
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    return tcsetattr(fd, TCSANOW, &tio);
}

/* --- Non-blocking Drop-Oldest Ring Queue for USB IN --- */
#define TX_Q_SIZE 32
typedef struct {
    uint8_t data[CRSF_MAX_FRAME];
    int     len;
} crsf_pkt_t;

static crsf_pkt_t      tx_q[TX_Q_SIZE];
static int             tx_head = 0, tx_tail = 0, tx_count = 0;
static pthread_mutex_t tx_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  tx_cv  = PTHREAD_COND_INITIALIZER;

static void enqueue_usb_in(const uint8_t *frame, int len) {
    if (len <= 0 || len > CRSF_MAX_FRAME) return;
    pthread_mutex_lock(&tx_mtx);
    if (tx_count == TX_Q_SIZE) { // Drop oldest if USB disconnected
        tx_tail = (tx_tail + 1) % TX_Q_SIZE;
        tx_count--;
    }
    memcpy(tx_q[tx_head].data, frame, (size_t)len);
    tx_q[tx_head].len = len;
    tx_head = (tx_head + 1) % TX_Q_SIZE;
    tx_count++;
    pthread_cond_signal(&tx_cv);
    pthread_mutex_unlock(&tx_mtx);
}

/* --- CRSF Stream Parser --- */
typedef struct {
    uint8_t buf[CRSF_MAX_FRAME];
    int     pos;
    int     expected_len;
} crsf_rx_t;

static void feed_crsf_bytes(crsf_rx_t *rx, const uint8_t *in, int n,
                            void (*on_frame)(const uint8_t *, int, void *), void *ctx) {
    for (int i = 0; i < n; i++) {
        uint8_t b = in[i];
        if (rx->pos == 0) {
            if (b == CRSF_SYNC_BYTE || b == CRSF_SYNC_FC || b == CRSF_SYNC_EDGETX) {
                rx->buf[rx->pos++] = b;
                rx->expected_len = 0;
            }
        } else if (rx->pos == 1) {
            if (b >= 2 && b <= (CRSF_MAX_FRAME - 2)) {
                rx->buf[rx->pos++] = b;
                rx->expected_len = b + 2;
            } else {
                rx->pos = 0;
            }
        } else {
            rx->buf[rx->pos++] = b;
            if (rx->pos == rx->expected_len) {
                if (crc8_dvb_s2(&rx->buf[2], rx->expected_len - 3) == rx->buf[rx->expected_len - 1])
                    on_frame(rx->buf, rx->expected_len, ctx);
                rx->pos = 0;
            }
        }
    }
}

static int                pty_master_fd = -1;
static int                udp_click_tx  = -1;
static struct sockaddr_in tracker_addr;

static void on_ap_frame(const uint8_t *frame, int len, void *ctx) {
    (void)ctx;
    enqueue_usb_in(frame, len);
}

static void on_usb_out_frame(const uint8_t *frame, int len, void *ctx) {
    (void)ctx;
    if (frame[2] == CRSF_FRAMETYPE_TARGET_CMD) {
        sendto(udp_click_tx, &frame[3], (size_t)(len - 4), 0,
               (struct sockaddr *)&tracker_addr, sizeof(tracker_addr));
        return;
    }
    int off = 0;
    while (off < len) {
        ssize_t w = write(pty_master_fd, frame + off, (size_t)(len - off));
        if (w > 0) off += (int)w;
        else if (errno == EAGAIN || errno == EINTR) continue;
        else break;
    }
}

static void *usb_in_thread(void *arg) {
    (void)arg;
    int ep_fd = -1;
    while (1) {
        while (ep_fd < 0) {
            ep_fd = open(FFS_CRSF_IN_EP, O_WRONLY);
            if (ep_fd < 0) usleep(50000);
        }
        crsf_pkt_t pkt;
        pthread_mutex_lock(&tx_mtx);
        while (tx_count == 0) pthread_cond_wait(&tx_cv, &tx_mtx);
        pkt = tx_q[tx_tail];
        tx_tail = (tx_tail + 1) % TX_Q_SIZE;
        tx_count--;
        pthread_mutex_unlock(&tx_mtx);

        ssize_t w = write(ep_fd, pkt.data, (size_t)pkt.len);
        if (w < 0) {
            if (errno == ESHUTDOWN || errno == EPIPE || errno == EAGAIN) {
                usleep(10000);
            } else if (errno == ENODEV) {
                close(ep_fd);
                ep_fd = -1;
            }
        }
    }
    return NULL;
}

static void *usb_out_thread(void *arg) {
    (void)arg;
    int ep_fd = -1;
    crsf_rx_t rx = {0};
    uint8_t buf[512]; // >= HS wMaxPacketSize (128)
    while (1) {
        while (ep_fd < 0) {
            ep_fd = open(FFS_CRSF_OUT_EP, O_RDONLY);
            if (ep_fd < 0) usleep(50000);
        }
        ssize_t n = read(ep_fd, buf, sizeof(buf));
        if (n > 0) {
            feed_crsf_bytes(&rx, buf, (int)n, on_usb_out_frame, NULL);
        } else if (n < 0) {
            if (errno == ESHUTDOWN || errno == EPIPE || errno == EAGAIN || errno == EINTR) {
                rx.pos = 0;
                usleep(20000);
            } else {
                close(ep_fd);
                ep_fd = -1;
                usleep(50000);
            }
        }
    }
    return NULL;
}

int main(void) {
    pty_master_fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    grantpt(pty_master_fd);
    unlockpt(pty_master_fd);
    char slave_path[128];
    ptsname_r(pty_master_fd, slave_path, sizeof(slave_path));
    set_raw_tty(pty_master_fd);
    unlink("/tmp/ttyAP_crsf");
    symlink(slave_path, "/tmp/ttyAP_crsf");
    printf("[crsf_mux] PTY ready: /tmp/ttyAP_crsf -> %s\n", slave_path);

    int udp_telem_rx = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET, .sin_port = htons(14601),
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK)
    };
    bind(udp_telem_rx, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

    udp_click_tx = socket(AF_INET, SOCK_DGRAM, 0);
    tracker_addr = (struct sockaddr_in){
        .sin_family = AF_INET, .sin_port = htons(14602),
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK)
    };

    pthread_t th_in, th_out;
    pthread_create(&th_in, NULL, usb_in_thread, NULL);
    pthread_create(&th_out, NULL, usb_out_thread, NULL);

    crsf_rx_t rx_ap = {0};
    uint8_t buf[256];
    while (1) {
        struct pollfd fds[2] = {
            { .fd = pty_master_fd, .events = POLLIN },
            { .fd = udp_telem_rx,  .events = POLLIN },
        };
        if (poll(fds, 2, 50) <= 0) continue;

        if (fds[0].revents & POLLIN) {
            ssize_t n = read(pty_master_fd, buf, sizeof(buf));
            if (n > 0) feed_crsf_bytes(&rx_ap, buf, (int)n, on_ap_frame, NULL);
        } else if (fds[0].revents & (POLLHUP | POLLERR)) {
            usleep(10000);
        }

        if (fds[1].revents & POLLIN) {
            uint8_t payload[58];
            ssize_t plen = recv(udp_telem_rx, payload, sizeof(payload), 0);
            if (plen > 0) {
                uint8_t frame[CRSF_MAX_FRAME];
                frame[0] = CRSF_SYNC_BYTE; // 0xC8 (or 0xEC if your Mac parser requires)
                frame[1] = (uint8_t)(plen + 2);
                frame[2] = CRSF_FRAMETYPE_TARGET_TELEM;
                memcpy(&frame[3], payload, (size_t)plen);
                frame[3 + plen] = crc8_dvb_s2(&frame[2], (int)plen + 1);
                enqueue_usb_in(frame, (int)plen + 4);
            }
        }
    }
}