SUMMARY = "Dedicated Low-Latency H.265 VTX Streamer for Luckfox Pico (RV1106)"
DESCRIPTION = "Direct Zero-Copy V4L2 to Rockchip MPP H.265 hardware encoder streaming to USB Vendor Bulk"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

DEPENDS = "rockchip-mpp"
RDEPENDS:${PN} = "rockchip-rkaiq-server luckfox-iqfiles"

SRC_URI = " \
    file://vtx.c \
    file://luckfox-vtx.service \
"

S = "${UNPACKDIR}"

inherit systemd

SYSTEMD_SERVICE:${PN} = "luckfox-vtx.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

CFLAGS:append:arm = " -fno-tree-vectorize -mno-unaligned-access"

do_compile() {
    ${CC} ${CFLAGS} ${LDFLAGS} \
        -I${STAGING_INCDIR} \
        -I${STAGING_INCDIR}/rockchip \
        ${S}/vtx.c \
        -lrockchip_mpp -lpthread \
        -o ${B}/vtx
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${B}/vtx ${D}${bindir}/vtx

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${S}/luckfox-vtx.service ${D}${systemd_system_unitdir}/luckfox-vtx.service
}

FILES:${PN} = " \
    ${bindir}/vtx \
    ${systemd_system_unitdir}/luckfox-vtx.service \
"
