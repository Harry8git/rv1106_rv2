SUMMARY = "Low-Power Motion-Assisted Visual Tracker for Luckfox RV1106"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

DEPENDS = "rockchip-librga"
RDEPENDS:${PN} = "rockchip-librga"

SRC_URI = " \
    file://tracker.c \
    file://luckfox-tracker.service \
"

inherit systemd

SYSTEMD_SERVICE:${PN} = "luckfox-tracker.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

do_compile() {
    ${CC} ${CFLAGS} ${LDFLAGS} -O3 \
        -I${STAGING_INCDIR} \
        -I${STAGING_INCDIR}/rga \
        ${THISDIR}/files/tracker.c \
        -lrga -lm -lpthread \
        -o ${WORKDIR}/luckfox_tracker
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/luckfox_tracker ${D}${bindir}/luckfox_tracker

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${THISDIR}/files/luckfox-tracker.service ${D}${systemd_system_unitdir}/luckfox-tracker.service
}

FILES:${PN} += " \
    ${bindir}/luckfox_tracker \
    ${systemd_system_unitdir}/luckfox-tracker.service \
"