SUMMARY = "AIC8800DC SDIO WiFi & Bluetooth kernel module driver"
DESCRIPTION = "AIC8800DC wireless LAN and Bluetooth driver modules for Rockchip RV1106"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module update-rc.d

PV = "1.0+git${SRCPV}"

SRC_URI = "git://github.com/LYU4662/aic8800-sdio-linux-1.0.git;protocol=https;branch=master \
           file://Makefile \
           file://aic8800_bsp-Kbuild \
           file://aic8800_fdrv-Kbuild \
           file://aic8800_btlpm-Kbuild \
           file://aic8800dc-wifi.init \
           file://aic8800-modules.conf \
           file://25-wlan.network \
"

SRCREV = "e61a54225e3c3c6daccd65366fd5064f941a961f"

# Prevent buildpaths QA check failure for debug symbols
INSANE_SKIP:${PN}-dbg += "buildpaths"

EXTRA_OEMAKE += " \
    KDIR=${STAGING_KERNEL_BUILDDIR} \
    CONFIG_AIC_FW_PATH=\"/lib/firmware/aic8800\" \
"

do_configure() {
    # Replace top Makefile with Yocto-compatible Makefile
    cp -f ${UNPACKDIR}/Makefile ${S}/Makefile

    # Install Kbuild files and remove vendor Makefiles to avoid recursion conflict
    for mod in aic8800_bsp aic8800_fdrv aic8800_btlpm; do
        rm -f ${S}/${mod}/Makefile
        cp -f ${UNPACKDIR}/${mod}-Kbuild ${S}/${mod}/Kbuild
    done
}

do_install:append() {
    # Install SysVinit script (if SysVinit is active)
    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${UNPACKDIR}/aic8800dc-wifi.init ${D}${sysconfdir}/init.d/aic8800dc-wifi

    # Install module autoload config for systemd
    install -d ${D}${sysconfdir}/modules-load.d
    install -m 0644 ${UNPACKDIR}/aic8800-modules.conf ${D}${sysconfdir}/modules-load.d/aic8800.conf

    # Install systemd network configuration for wlan0 DHCP
    install -d ${D}${sysconfdir}/systemd/network
    install -m 0644 ${UNPACKDIR}/25-wlan.network ${D}${sysconfdir}/systemd/network/25-wlan.network

    # Install AIC8800DC firmware binaries
    install -d ${D}${nonarch_base_libdir}/firmware/aic8800
    install -m 0644 ${S}/firmware/aic8800_sdio/aic8800DC/* ${D}${nonarch_base_libdir}/firmware/aic8800/
}

INITSCRIPT_NAME = "aic8800dc-wifi"
INITSCRIPT_PARAMS = "defaults 90"

FILES:${PN} += " \
    ${sysconfdir}/init.d/aic8800dc-wifi \
    ${sysconfdir}/modules-load.d/aic8800.conf \
    ${sysconfdir}/systemd/network/25-wlan.network \
    ${nonarch_base_libdir}/firmware/aic8800/* \
"

RPROVIDES:${PN} += "kernel-module-aic8800-bsp kernel-module-aic8800-fdrv kernel-module-aic8800-btlpm"
AUTOLOAD += "aic8800_bsp aic8800_fdrv"

COMPATIBLE_MACHINE = "luckfox-pico-zero"
