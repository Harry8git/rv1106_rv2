/*
 * Dedicated Persistent USB Gadget FunctionFS Daemon
 * Keeps VTX Video (0xFF/0x00) and CRSF Telemetry (0xFF/0x01) active 24/7.
 */

#define _GNU_SOURCE
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <linux/usb/ch9.h>
#include <linux/usb/functionfs.h>

#define VTX_EP0_PATH    "/dev/usb-ffs/vtx/ep0"
#define CRSF_EP0_PATH   "/dev/usb-ffs/crsf/ep0"
#define GADGET_UDC_PATH "/sys/kernel/config/usb_gadget/g1/UDC"

struct ffs_ep_desc_no_audio {
    __u8   bLength;
    __u8   bDescriptorType;
    __u8   bEndpointAddress;
    __u8   bmAttributes;
    __le16 wMaxPacketSize;
    __u8   bInterval;
} __attribute__((packed));

/* --- VTX Descriptors (1 Bulk IN) --- */
struct ffs_vtx_descs {
    struct usb_functionfs_descs_head_v2 header;
    __le32 fs_count;
    __le32 hs_count;
    struct {
        struct usb_interface_descriptor intf;
        struct ffs_ep_desc_no_audio bulk_in;
    } __attribute__((packed)) fs_descs, hs_descs;
} __attribute__((packed));

struct ffs_vtx_strings {
    __le32 magic;
    __le32 length;
    __le32 str_count;
    __le32 lang_count;
    struct {
        __le16 code;
        char str1[18]; // "Luckfox VTX Video\0" = exact 18 bytes
    } __attribute__((packed)) tab;
} __attribute__((packed));

/* --- CRSF Descriptors (1 Interrupt IN + 1 Interrupt OUT) --- */
struct ffs_crsf_descs {
    struct usb_functionfs_descs_head_v2 header;
    __le32 fs_count;
    __le32 hs_count;
    struct {
        struct usb_interface_descriptor intf;
        struct ffs_ep_desc_no_audio int_in;
        struct ffs_ep_desc_no_audio int_out;
    } __attribute__((packed)) fs_descs, hs_descs;
} __attribute__((packed));

struct ffs_crsf_strings {
    __le32 magic;
    __le32 length;
    __le32 str_count;
    __le32 lang_count;
    struct {
        __le16 code;
        char str1[19]; // "Luckfox CRSF Telem\0" = exact 19 bytes (no extra trailing pad!)
    } __attribute__((packed)) tab;
} __attribute__((packed));

static volatile bool quit = false;
static void sig_handler(int sig) { (void)sig; quit = true; }

static void handle_ep0_events(int fd) {
    struct usb_functionfs_event events[4];
    ssize_t n = read(fd, events, sizeof(events));
    if (n <= 0) return;

    size_t count = (size_t)n / sizeof(events[0]);
    for (size_t i = 0; i < count; i++) {
        if (events[i].type == FUNCTIONFS_SETUP) {
            if (events[i].u.setup.bRequestType & USB_DIR_IN)
                write(fd, NULL, 0);
            else
                read(fd, NULL, 0);
        }
    }
}

int main(void) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGHUP, SIG_IGN);
    prctl(PR_SET_NAME, "usb_ffs_daemon", 0, 0, 0);

    /* 1. Prepare & Write VTX Descriptors */
    struct ffs_vtx_descs vtx_descs;
    memset(&vtx_descs, 0, sizeof(vtx_descs));
    vtx_descs.header.magic  = htole32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2);
    vtx_descs.header.flags  = htole32(FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC);
    vtx_descs.header.length = htole32(sizeof(vtx_descs));
    vtx_descs.fs_count = htole32(2);
    vtx_descs.hs_count = htole32(2);

    vtx_descs.fs_descs.intf.bLength            = sizeof(struct usb_interface_descriptor);
    vtx_descs.fs_descs.intf.bDescriptorType    = USB_DT_INTERFACE;
    vtx_descs.fs_descs.intf.bInterfaceNumber   = 0;
    vtx_descs.fs_descs.intf.bNumEndpoints      = 1;
    vtx_descs.fs_descs.intf.bInterfaceClass    = USB_CLASS_VENDOR_SPEC;
    vtx_descs.fs_descs.intf.bInterfaceSubClass = 0x00;
    vtx_descs.fs_descs.intf.bInterfaceProtocol = 0x00;
    vtx_descs.fs_descs.intf.iInterface         = 1;

    vtx_descs.fs_descs.bulk_in.bLength          = 7;
    vtx_descs.fs_descs.bulk_in.bDescriptorType  = USB_DT_ENDPOINT;
    vtx_descs.fs_descs.bulk_in.bEndpointAddress = USB_DIR_IN | 1;
    vtx_descs.fs_descs.bulk_in.bmAttributes     = USB_ENDPOINT_XFER_BULK;
    vtx_descs.fs_descs.bulk_in.wMaxPacketSize   = htole16(64);
    vtx_descs.fs_descs.bulk_in.bInterval        = 0;

    vtx_descs.hs_descs = vtx_descs.fs_descs;
    vtx_descs.hs_descs.bulk_in.wMaxPacketSize = htole16(512);

    struct ffs_vtx_strings vtx_strings;
    memset(&vtx_strings, 0, sizeof(vtx_strings));
    vtx_strings.magic      = htole32(FUNCTIONFS_STRINGS_MAGIC);
    vtx_strings.length     = htole32(sizeof(vtx_strings));
    vtx_strings.str_count  = htole32(1);
    vtx_strings.lang_count = htole32(1);
    vtx_strings.tab.code   = htole16(0x0409);
    memcpy(vtx_strings.tab.str1, "Luckfox VTX Video", 18);

    int vtx_ep0 = open(VTX_EP0_PATH, O_RDWR);
    if (vtx_ep0 < 0) { perror("open vtx ep0"); return 1; }
    if (write(vtx_ep0, &vtx_descs, sizeof(vtx_descs)) < 0) { perror("write vtx descs"); return 1; }
    if (write(vtx_ep0, &vtx_strings, sizeof(vtx_strings)) < 0) { perror("write vtx strings"); return 1; }

    /* 2. Prepare & Write CRSF Descriptors (SubClass 0x01, Interrupt IN/OUT) */
    struct ffs_crsf_descs crsf_descs;
    memset(&crsf_descs, 0, sizeof(crsf_descs));
    crsf_descs.header.magic  = htole32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2);
    crsf_descs.header.flags  = htole32(FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC);
    crsf_descs.header.length = htole32(sizeof(crsf_descs));
    crsf_descs.fs_count = htole32(3);
    crsf_descs.hs_count = htole32(3);

    crsf_descs.fs_descs.intf.bLength            = sizeof(struct usb_interface_descriptor);
    crsf_descs.fs_descs.intf.bDescriptorType    = USB_DT_INTERFACE;
    crsf_descs.fs_descs.intf.bInterfaceNumber   = 0;
    crsf_descs.fs_descs.intf.bNumEndpoints      = 2;
    crsf_descs.fs_descs.intf.bInterfaceClass    = USB_CLASS_VENDOR_SPEC;
    crsf_descs.fs_descs.intf.bInterfaceSubClass = 0x01; // Distinguishes CRSF from Video (0x00)
    crsf_descs.fs_descs.intf.bInterfaceProtocol = 0x00;
    crsf_descs.fs_descs.intf.iInterface         = 1;

    /* ep1 -> Interrupt IN (Drone -> RP2350) */
    crsf_descs.fs_descs.int_in.bLength          = 7;
    crsf_descs.fs_descs.int_in.bDescriptorType  = USB_DT_ENDPOINT;
    crsf_descs.fs_descs.int_in.bEndpointAddress = USB_DIR_IN | 1;
    crsf_descs.fs_descs.int_in.bmAttributes     = USB_ENDPOINT_XFER_INT;
    crsf_descs.fs_descs.int_in.wMaxPacketSize   = htole16(64);
    crsf_descs.fs_descs.int_in.bInterval        = 1; // 1 ms on FS

    /* ep2 -> Interrupt OUT (RP2350 -> Drone) */
    crsf_descs.fs_descs.int_out.bLength          = 7;
    crsf_descs.fs_descs.int_out.bDescriptorType  = USB_DT_ENDPOINT;
    crsf_descs.fs_descs.int_out.bEndpointAddress = USB_DIR_OUT | 2;
    crsf_descs.fs_descs.int_out.bmAttributes     = USB_ENDPOINT_XFER_INT;
    crsf_descs.fs_descs.int_out.wMaxPacketSize   = htole16(64);
    crsf_descs.fs_descs.int_out.bInterval        = 1; // 1 ms on FS

    /* HS Descriptors: 128B avoids 64B ZLP edge cases; bInterval=4 is 1 ms (2^(4-1)*125us) */
    crsf_descs.hs_descs = crsf_descs.fs_descs;
    crsf_descs.hs_descs.int_in.wMaxPacketSize  = htole16(128);
    crsf_descs.hs_descs.int_in.bInterval       = 4;
    crsf_descs.hs_descs.int_out.wMaxPacketSize = htole16(128);
    crsf_descs.hs_descs.int_out.bInterval      = 4;

    struct ffs_crsf_strings crsf_strings;
    memset(&crsf_strings, 0, sizeof(crsf_strings));
    crsf_strings.magic      = htole32(FUNCTIONFS_STRINGS_MAGIC);
    crsf_strings.length     = htole32(sizeof(crsf_strings));
    crsf_strings.str_count  = htole32(1);
    crsf_strings.lang_count = htole32(1);
    crsf_strings.tab.code   = htole16(0x0409);
    memcpy(crsf_strings.tab.str1, "Luckfox CRSF Telem", 19);

    int crsf_ep0 = open(CRSF_EP0_PATH, O_RDWR);
    if (crsf_ep0 < 0) { perror("open crsf ep0"); return 1; }
    if (write(crsf_ep0, &crsf_descs, sizeof(crsf_descs)) < 0) { perror("write crsf descs"); return 1; }
    if (write(crsf_ep0, &crsf_strings, sizeof(crsf_strings)) < 0) { perror("write crsf strings"); return 1; }

    /* 3. Bind UDC only AFTER both FunctionFS instances are registered */
    FILE *f_udc = fopen(GADGET_UDC_PATH, "w");
    if (f_udc) {
        fputs("ffb00000.usb\n", f_udc);
        fclose(f_udc);
    }

    system("if ip link show usb0 >/dev/null 2>&1; then "
           "  ip link set usb0 up && "
           "  ip addr add 192.168.2.100/24 dev usb0 2>/dev/null || true; "
           "fi");

    fprintf(stderr, ">>> Luckfox USB Daemon: NCM + VTX + CRSF Vendor Ready <<<\n");

    while (!quit) {
        struct pollfd pfds[2] = {
            { .fd = vtx_ep0,  .events = POLLIN },
            { .fd = crsf_ep0, .events = POLLIN },
        };
        int ret = poll(pfds, 2, 1000);
        if (ret <= 0) continue;
        if (pfds[0].revents & POLLIN) handle_ep0_events(vtx_ep0);
        if (pfds[1].revents & POLLIN) handle_ep0_events(crsf_ep0);
    }

    close(crsf_ep0);
    close(vtx_ep0);
    return 0;
}