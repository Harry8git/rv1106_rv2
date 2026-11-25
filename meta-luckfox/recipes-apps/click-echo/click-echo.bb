SUMMARY = "CRSF Target Click Echo Test"
LICENSE = "CLOSED"

do_compile() {
    ${CC} ${CFLAGS} ${LDFLAGS} -O2 ${THISDIR}/files/click_echo.c -o ${WORKDIR}/click_echo
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/click_echo ${D}${bindir}/click_echo
}