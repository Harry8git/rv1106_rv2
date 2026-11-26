SUMMARY = "Rockchip RKNPU2 Runtime (librknnmrt) for RV1106 glibc"
LICENSE = "CLOSED"

BASE_URL = "https://raw.githubusercontent.com/airockchip/rknn-toolkit2/v2.5.0/rknpu2/runtime/Linux/librknn_api"

SRC_URI = " \
    ${BASE_URL}/armhf/librknnmrt.so;name=lib \
    ${BASE_URL}/include/rknn_api.h;name=hdr1 \
    ${BASE_URL}/include/rknn_custom_op.h;name=hdr2 \
    ${BASE_URL}/include/rknn_matmul_api.h;name=hdr3 \
"

SRC_URI[lib.sha256sum]  = "5cb5466c8136ef1b0eb5ef4f0b4558249cf1e95467e83f8ab99873ec8c40c8ab"
SRC_URI[hdr1.sha256sum] = "2d4160cd868efd7b83a72587e411c16d6972da5876e067bd73665cea778ce666"
SRC_URI[hdr2.sha256sum] = "af5983da0ca244ca31dc3162aa683322b0285531196c7a770f29cd2e3b8ccaa6"
SRC_URI[hdr3.sha256sum] = "aaadd9a7118de30a06b222996b6731db77095d00f5931a7a98c83a67f14a4d42"

S = "${UNPACKDIR}"

do_configure[noexec] = "1"
do_compile[noexec] = "1"

do_install() {
    install -d ${D}${libdir}
    install -m 0755 ${S}/librknnmrt.so ${D}${libdir}/librknnmrt.so

    install -d ${D}${includedir}
    install -m 0644 ${S}/rknn_api.h        ${D}${includedir}/rknn_api.h
    install -m 0644 ${S}/rknn_custom_op.h  ${D}${includedir}/rknn_custom_op.h
    install -m 0644 ${S}/rknn_matmul_api.h ${D}${includedir}/rknn_matmul_api.h
}

SOLIBS = ".so"
FILES_SOLIBSDEV = ""

FILES:${PN} = "${libdir}/librknnmrt.so"
FILES:${PN}-dev = "${includedir}/*.h"

INSANE_SKIP:${PN} += "already-stripped ldflags dev-so"