#!/usr/bin/env bash
# One-time Pi setup: packages, build, systemd service, Wi-Fi hotspot.
# Run on the Pi from the project folder:
#   sudo HOTSPOT_SSID=pothole-pi HOTSPOT_PSK=potholecam ./setup/setup_pi.sh
# Run it over Ethernet SSH (or a keyboard): the hotspot takes over wlan0, so the
# Pi drops off your home Wi-Fi (and loses internet) when it comes up.
set -euo pipefail

SSID="${HOTSPOT_SSID:-pothole-pi}"
PSK="${HOTSPOT_PSK:-potholecam}"          # WPA2 needs 8-63 characters
COUNTRY="${WIFI_COUNTRY:-ZA}"             # Wi-Fi stays blocked until a country is set
APP_DIR="$(cd "$(dirname "$0")/.." && pwd)"
APP_USER="${SUDO_USER:?run with sudo from your normal user}"

[ ${#PSK} -ge 8 ] || { echo "HOTSPOT_PSK must be at least 8 characters"; exit 1; }

echo "== packages (needs internet - do this before the hotspot is up)"
apt-get update
apt-get install -y build-essential pkg-config libcamera-dev libturbojpeg0-dev libcurl4-openssl-dev rpicam-apps dkms hailo-all python3-opencv

echo "== camera and serial-port (4G HAT / GPS) access for $APP_USER"
usermod -aG video,dialout "$APP_USER"

# ModemManager would grab the SIM7600's AT ports; the HAT runs in RNDIS mode
# (a plain network interface, usb0) and build/pothole talks to it directly.
if systemctl list-unit-files ModemManager.service >/dev/null 2>&1; then
    systemctl disable --now ModemManager.service || true
fi
sudo -u "$APP_USER" cp -n "$APP_DIR/uplink.conf.example" "$APP_DIR/uplink.conf"

echo "== build"
sudo -u "$APP_USER" make -C "$APP_DIR"

echo "== systemd service"
sed -e "s|^User=.*|User=$APP_USER|" \
    -e "s|^WorkingDirectory=.*|WorkingDirectory=$APP_DIR|" \
    -e "s|^ExecStart=.*|ExecStart=$APP_DIR/build/pothole|" \
    "$APP_DIR/setup/pothole.service" > /etc/systemd/system/pothole.service
systemctl daemon-reload
systemctl enable pothole
systemctl restart pothole

echo "== hotspot '$SSID' on wlan0 (Pi = 10.42.0.1)"
raspi-config nonint do_wifi_country "$COUNTRY"
rfkill unblock wifi
nmcli con delete pothole-hotspot >/dev/null 2>&1 || true
nmcli con add type wifi ifname wlan0 con-name pothole-hotspot ssid "$SSID" \
    802-11-wireless.mode ap 802-11-wireless.band bg \
    ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method disabled \
    wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
    wifi-sec.psk "$PSK" \
    connection.autoconnect yes connection.autoconnect-priority 100
nmcli con up pothole-hotspot

echo
echo "Done. Join Wi-Fi '$SSID' (password '$PSK') and open http://10.42.0.1:8000"
echo "SSH over the hotspot: ssh $APP_USER@10.42.0.1"
echo "Logs: journalctl -u pothole -f"
