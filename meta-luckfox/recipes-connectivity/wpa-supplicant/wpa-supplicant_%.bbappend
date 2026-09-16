FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

PACKAGECONFIG = ""

SRC_URI += "file://wpa_supplicant-wlan0.conf"

do_configure:append() {
    echo "CONFIG_TLS=internal" >> ${S}/wpa_supplicant/.config
    echo "CONFIG_CRYPTO=internal" >> ${S}/wpa_supplicant/.config
    echo "CONFIG_INTERNAL_LIBTOMMATH=y" >> ${S}/wpa_supplicant/.config
    echo "CONFIG_INTERNAL_LIBTOMMATH_FAST=y" >> ${S}/wpa_supplicant/.config
    sed -i -e 's/^CONFIG_CTRL_IFACE_DBUS/#CONFIG_CTRL_IFACE_DBUS/' \
           -e 's/\(^CONFIG_SAE=\)/#\1/' \
           -e 's/\(^CONFIG_OWE=\)/#\1/' \
           -e 's/\(^CONFIG_DPP=\)/#\1/' \
           -e 's/\(^CONFIG_EAP_PWD=\)/#\1/' \
           ${S}/wpa_supplicant/.config
}

do_install:append() {
    install -d ${D}${sysconfdir}/wpa_supplicant
    install -m 0600 ${UNPACKDIR}/wpa_supplicant-wlan0.conf ${D}${sysconfdir}/wpa_supplicant/wpa_supplicant-wlan0.conf
}

SYSTEMD_SERVICE:${PN} = "wpa_supplicant@wlan0.service"
SYSTEMD_AUTO_ENABLE = "enable"
DEPENDS:remove = "dbus"
