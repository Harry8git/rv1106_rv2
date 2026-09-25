/*
 * Dedicated Persistent USB Gadget FunctionFS Daemon
 * Keeps USB descriptors active 24/7 so composite interfaces
 * (CDC-NCM, CDC-ACM CRSF, and Vendor Bulk) never drop.
 */

#define _GNU_SOURCE
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
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

#define FFS_EP0_PATH    "/dev/usb-ffs/vtx/ep0"
#define GADGET_UDC_PATH "/sys/kernel/config/usb_gadget/g1/UDC"

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

static volatile bool quit = false;
static void sig_handler(int sig) { (void)sig; quit = true; }

int main(void) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGHUP, SIG_IGN);
    prctl(PR_SET_NAME, "usb_ffs_daemon", 0, 0, 0);

    /* Prepare Descriptors (Vendor Class 0xFF, Bulk IN Endpoint 0x81) */
    struct ffs_desc_full descs;
    memset(&descs, 0, sizeof(descs));
    descs.header.magic = htole32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2);
    descs.header.flags = htole32(FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC);
    descs.header.length = htole32(sizeof(descs));
    descs.fs_count = htole32(2);
    descs.hs_count = htole32(2);

    /* FS Descriptors */
    descs.fs_descs.intf.bLength = sizeof(struct usb_interface_descriptor);
    descs.fs_descs.intf.bDescriptorType = USB_DT_INTERFACE;
    descs.fs_descs.intf.bInterfaceNumber = 0;
    descs.fs_descs.intf.bAlternateSetting = 0;
    descs.fs_descs.intf.bNumEndpoints = 1;
    descs.fs_descs.intf.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
    descs.fs_descs.intf.bInterfaceSubClass = 0x00;
    descs.fs_descs.intf.bInterfaceProtocol = 0x00;
    descs.fs_descs.intf.iInterface = 1;

    descs.fs_descs.bulk_in.bLength = 7;
    descs.fs_descs.bulk_in.bDescriptorType = USB_DT_ENDPOINT;
    descs.fs_descs.bulk_in.bEndpointAddress = USB_DIR_IN | 1;
    descs.fs_descs.bulk_in.bmAttributes = USB_ENDPOINT_XFER_BULK;
    descs.fs_descs.bulk_in.wMaxPacketSize = htole16(64);
    descs.fs_descs.bulk_in.bInterval = 0;

    /* HS Descriptors */
    descs.hs_descs.intf = descs.fs_descs.intf;
    descs.hs_descs.bulk_in = descs.fs_descs.bulk_in;
    descs.hs_descs.bulk_in.wMaxPacketSize = htole16(512);

    /* Strings */
    struct ffs_strings_full strings;
    memset(&strings, 0, sizeof(strings));
    strings.magic = htole32(FUNCTIONFS_STRINGS_MAGIC);
    strings.length = htole32(sizeof(strings));
    strings.str_count = htole32(1);
    strings.lang_count = htole32(1);
    strings.tab.code = htole16(0x0409);
    strncpy(strings.tab.str1, "Luckfox VTX Video", sizeof(strings.tab.str1) - 1);

    int ep0_fd = open(FFS_EP0_PATH, O_RDWR);
    if (ep0_fd < 0) {
        perror("Cannot open " FFS_EP0_PATH);
        return 1;
    }

    if (write(ep0_fd, &descs, sizeof(descs)) < 0) {
        perror("Failed writing descriptors to ep0");
        close(ep0_fd);
        return 1;
    }

    if (write(ep0_fd, &strings, sizeof(strings)) < 0) {
        perror("Failed writing strings to ep0");
        close(ep0_fd);
        return 1;
    }

    /* Bind USB Device Controller */
    FILE *f_udc = fopen(GADGET_UDC_PATH, "w");
    if (f_udc) {
        fputs("ffb00000.usb\n", f_udc);
        fclose(f_udc);
    }

    /* Bring up Ethernet usb0 */
    system("if ip link show usb0 >/dev/null 2>&1; then "
           "  ip link set usb0 up && "
           "  ip addr add 169.254.100.1/16 dev usb0 2>/dev/null || true; "
           "fi");

    fprintf(stderr, ">>> Luckfox USB Daemon: Composite Gadget Ready 24/7 <<<\n");

    /* Event loop keeps ep0 open forever */
    struct usb_functionfs_event events[4];
    while (!quit) {
        struct pollfd pfd = { .fd = ep0_fd, .events = POLLIN };
        int ret = poll(&pfd, 1, 1000);
        if (ret <= 0) continue;

        ssize_t n = read(ep0_fd, events, sizeof(events));
        if (n <= 0) continue;

        size_t count = (size_t)n / sizeof(events[0]);
        for (size_t i = 0; i < count; i++) {
            if (events[i].type == FUNCTIONFS_SETUP) {
                if (events[i].u.setup.bRequestType & USB_DIR_IN)
                    write(ep0_fd, NULL, 0);
                else
                    read(ep0_fd, NULL, 0);
            }
        }
    }

    close(ep0_fd);
    return 0;
}
