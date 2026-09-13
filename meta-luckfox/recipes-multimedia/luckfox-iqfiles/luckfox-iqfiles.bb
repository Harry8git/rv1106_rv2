SUMMARY = "Original IMX462 ISP IQ configuration for RV1106"
DESCRIPTION = "Original IMX462 ISP IQ configuration authored for the RV1106 VTX project."

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=8e885c58a55a2612e5b59cc803efeecf"

PACKAGE_ARCH = "${MACHINE_ARCH}"

S = "${UNPACKDIR}"

SRC_URI = " \
    file://imx462_imx462_default.json \
    file://LICENSE \
"

do_configure[noexec] = "1"
do_compile[noexec] = "1"

do_install() {
    install -d ${D}${sysconfdir}/iqfiles
    install -m 0644 ${UNPACKDIR}/imx462_imx462_default.json ${D}${sysconfdir}/iqfiles/
}

FILES:${PN} = "${sysconfdir}/iqfiles/"
