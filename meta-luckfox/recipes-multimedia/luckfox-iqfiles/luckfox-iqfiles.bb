SUMMARY = "Luckfox vendor ISP IQ calibration files for RV1106"
DESCRIPTION = "Sensor IQ JSON files and CAC binary calibration data for \
cameras supported by Luckfox Pico boards (sc4336, sc3336, mis5001, imx462/imx327). \
These files are taken directly from the Luckfox vendor SDK."

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = "file://NOTICE;md5=afe667c79b10e173904da1ed65460a49"

PACKAGE_ARCH = "${MACHINE_ARCH}"

S = "${UNPACKDIR}"

SRC_URI = " \
    file://imx462_imx462_default.json \
"

do_configure[noexec] = "1"
do_compile[noexec] = "1"

do_install() {
    install -d ${D}${sysconfdir}/iqfiles
    install -m 0644 ${UNPACKDIR}/imx462_imx462_default.json ${D}${sysconfdir}/iqfiles/
}

FILES:${PN} = "${sysconfdir}/iqfiles/"
