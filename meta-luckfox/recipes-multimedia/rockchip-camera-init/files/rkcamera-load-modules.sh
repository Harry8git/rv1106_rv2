#!/bin/sh
udevadm control --stop-exec-queue
trap 'udevadm control --start-exec-queue' EXIT

set -e

for sensor in imx290; do
	modprobe "$sensor" || true
done

modprobe phy-rockchip-csi2-dphy-hw
modprobe phy-rockchip-csi2-dphy

modprobe video_rkcif
[ -w /sys/module/video_rkcif/parameters/clr_unready_dev ] && \
	echo 1 > /sys/module/video_rkcif/parameters/clr_unready_dev

modprobe video_rkisp
[ -w /sys/module/video_rkisp/parameters/clr_unready_dev ] && \
	echo 1 > /sys/module/video_rkisp/parameters/clr_unready_dev

exit 0
