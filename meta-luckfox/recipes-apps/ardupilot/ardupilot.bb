SUMMARY = "ArduPilot Plane for Luckfox RV1106"
HOMEPAGE = "https://github.com/Harry8git/Ardupilot-rv1106"
LICENSE = "GPL-3.0-only"
LIC_FILES_CHKSUM = "file://COPYING.txt;md5=d32239bcb673463ab874e80d47fae504"

inherit externalsrc

EXTERNALSRC = "${TOPDIR}/../apps/ardupilot"
EXTERNALSRC_BUILD = "${EXTERNALSRC}"

DEPENDS += "python3-native"

ARDUPILOT_BOARD = "luckfox"
ARDUPILOT_VEHICLE = "plane"

do_configure() {
    cd ${S}
    /usr/bin/python3 ./waf configure --board=${ARDUPILOT_BOARD}
}

do_compile() {
    cd ${S}
    /usr/bin/python3 ./waf ${ARDUPILOT_VEHICLE}
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${S}/build/${ARDUPILOT_BOARD}/bin/ardu${ARDUPILOT_VEHICLE} ${D}${bindir}/
}

FILES:${PN} += "${bindir}/ardu${ARDUPILOT_VEHICLE}"

do_buildclean[noexec] = "1"