#!/bin/sh

mountpoint -q /sys/kernel/config || mount -t configfs none /sys/kernel/config 2>/dev/null || true

GADGET_DIR="/sys/kernel/config/usb_gadget/g1"
mkdir -p "$GADGET_DIR"
cd "$GADGET_DIR" || exit 1

if [ -f UDC ] && [ -n "$(cat UDC 2>/dev/null)" ]; then
    echo "" > UDC 2>/dev/null || true
fi

echo 0x2207 > idVendor
echo 0x0011 > idProduct

# Composite Device Class with IAD (Required for macOS NCM + ACM)
echo 0xEF > bDeviceClass
echo 0x02 > bDeviceSubClass
echo 0x01 > bDeviceProtocol

mkdir -p strings/0x409
SERIAL=$(grep Serial /proc/cpuinfo 2>/dev/null | awk '{print $3}')
[ -z "$SERIAL" ] && SERIAL="0123456789"
echo "$SERIAL" > strings/0x409/serialnumber
echo "Luckfox" > strings/0x409/manufacturer
echo "Pico Zero" > strings/0x409/product

mkdir -p functions/ncm.usb0
mkdir -p functions/acm.GS0

# Consistent MAC address derived from board serial number
HASH=$(echo "$SERIAL" | md5sum | head -c 8)
DEV_MAC="12:22:$(echo $HASH | cut -c1-2):$(echo $HASH | cut -c3-4):$(echo $HASH | cut -c5-6):01"
HOST_MAC="12:22:$(echo $HASH | cut -c1-2):$(echo $HASH | cut -c3-4):$(echo $HASH | cut -c5-6):02"

echo "$DEV_MAC" > functions/ncm.usb0/dev_addr
echo "$HOST_MAC" > functions/ncm.usb0/host_addr

mkdir -p configs/c.1/strings/0x409
echo "CDC-NCM + ACM" > configs/c.1/strings/0x409/configuration
ln -sf functions/ncm.usb0 configs/c.1/
ln -sf functions/acm.GS0 configs/c.1/

# Bind directly to UDC
UDC_NAME=$(ls /sys/class/udc 2>/dev/null | head -n 1)
[ -n "$UDC_NAME" ] && echo "$UDC_NAME" > UDC

# Bring interface up and assign static link-local address
if ip link show usb0 >/dev/null 2>&1; then
    ip link set usb0 up
    ip addr add 169.254.100.1/16 dev usb0 2>/dev/null || true
fi
