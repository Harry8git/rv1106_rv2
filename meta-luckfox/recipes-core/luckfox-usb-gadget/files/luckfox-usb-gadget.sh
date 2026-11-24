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

# Composite Device Class with IAD (macOS NCM + Vendor VTX + Vendor CRSF)
echo 0xEF > bDeviceClass
echo 0x02 > bDeviceSubClass
echo 0x01 > bDeviceProtocol

mkdir -p strings/0x409
SERIAL=$(grep Serial /proc/cpuinfo 2>/dev/null | awk '{print $3}')
[ -z "$SERIAL" ] && SERIAL="0123456789"
echo "$SERIAL" > strings/0x409/serialnumber
echo "Luckfox" > strings/0x409/manufacturer
echo "Pico Zero VTX" > strings/0x409/product

# 1. CDC-NCM (Ethernet for SSH / Config)
mkdir -p functions/ncm.usb0
HASH=$(echo "$SERIAL" | md5sum | cut -c 1-8)
DEV_MAC="12:22:$(echo $HASH | cut -c1-2):$(echo $HASH | cut -c3-4):$(echo $HASH | cut -c5-6):01"
HOST_MAC="12:22:$(echo $HASH | cut -c1-2):$(echo $HASH | cut -c3-4):$(echo $HASH | cut -c5-6):02"
echo "$DEV_MAC" > functions/ncm.usb0/dev_addr
echo "$HOST_MAC" > functions/ncm.usb0/host_addr

# 2. FunctionFS (Vendor Class Bulk Endpoint for Video)
mkdir -p functions/ffs.vtx
mkdir -p /dev/usb-ffs/vtx
mountpoint -q /dev/usb-ffs/vtx || mount -t functionfs vtx /dev/usb-ffs/vtx

# 3. FunctionFS (Vendor Class Interrupt Endpoints for CRSF + Target)
mkdir -p functions/ffs.crsf
mkdir -p /dev/usb-ffs/crsf
mountpoint -q /dev/usb-ffs/crsf || mount -t functionfs crsf /dev/usb-ffs/crsf

mkdir -p configs/c.1/strings/0x409
echo "CDC-NCM + VTX + CRSF Vendor" > configs/c.1/strings/0x409/configuration

rm -f configs/c.1/acm.GS0
ln -sf functions/ncm.usb0 configs/c.1/
ln -sf functions/ffs.vtx  configs/c.1/
ln -sf functions/ffs.crsf configs/c.1/

# Launch persistent USB daemon (writes BOTH ep0s before binding UDC)
killall luckfox-usb-daemon 2>/dev/null || true
luckfox-usb-daemon &