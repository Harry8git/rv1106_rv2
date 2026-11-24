SUMMARY = "ArduPilot Plane for Luckfox RV1106"
HOMEPAGE = "https://github.com/Harry8git/Ardupilot-rv1106"
LICENSE = "GPL-3.0-only"
LIC_FILES_CHKSUM = "file://COPYING.txt;md5=d32239bcb673463ab874e80d47fae504"

inherit externalsrc systemd

EXTERNALSRC = "${TOPDIR}/../apps/ardupilot"
EXTERNALSRC_BUILD = "${EXTERNALSRC}"

DEPENDS += "python3-native"

ARDUPILOT_BOARD = "luckfox"
ARDUPILOT_VEHICLE = "plane"

SYSTEMD_SERVICE:${PN} = "crsf-mux.service ardupilot.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

do_configure() {
    cd ${S}
    /usr/bin/python3 ./waf configure --board=${ARDUPILOT_BOARD}
}

do_compile() {
    cd ${S}
    /usr/bin/python3 ./waf ${ARDUPILOT_VEHICLE}

    # Build crsf_mux into ${WORKDIR} using Yocto's CC/CFLAGS/LDFLAGS (keeps EXTERNALSRC clean)
    ${CC} ${CFLAGS} ${LDFLAGS} -O2 -pthread ${THISDIR}/files/crsf_mux.c -o ${WORKDIR}/crsf_mux
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${S}/build/${ARDUPILOT_BOARD}/bin/ardu${ARDUPILOT_VEHICLE} ${D}${bindir}/
    install -m 0755 ${WORKDIR}/crsf_mux ${D}${bindir}/crsf_mux

    install -d ${D}${sysconfdir}
    install -m 0644 ${S}/libraries/AP_HAL_Linux/hwdef/${ARDUPILOT_BOARD}/defaults.parm ${D}${sysconfdir}/ardupilot.parm

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${THISDIR}/files/crsf-mux.service ${D}${systemd_system_unitdir}/crsf-mux.service
    install -m 0644 ${THISDIR}/files/ardupilot.service ${D}${systemd_system_unitdir}/ardupilot.service
}

FILES:${PN} += " \
    ${bindir}/ardu${ARDUPILOT_VEHICLE} \
    ${bindir}/crsf_mux \
    ${sysconfdir}/ardupilot.parm \
    ${systemd_system_unitdir}/crsf-mux.service \
    ${systemd_system_unitdir}/ardupilot.service \
"

do_buildclean[noexec] = "1"