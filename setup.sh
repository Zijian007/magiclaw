#!/bin/bash
# -*- coding: utf-8 -*-

set -e
set -x

### Update packages
echo "=========================================="
echo "Update packages"
echo "=========================================="

# Update package lists and upgrade installed packages
echo "Updating package lists and upgrading installed packages..."
sudo systemctl stop packagekit
sudo apt update && sudo apt upgrade -y

### Install Miniconda
echo "=========================================="
echo "Install Miniconda"
echo "=========================================="
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
eval "$(~/miniconda3/bin/conda shell.bash hook)"
~/miniconda3/bin/conda init --all


### Set hostname
echo "=========================================="
echo "Set hostname"
echo "=========================================="

# Get the last 6 characters of the MAC address of eth0
echo "Setting hostname based on MAC address of eth0..."
MAC_SUFFIX=$(ip link show eth0 | grep ether | awk '{print $2}' | tail -c 6 | tr -d ':')
NEW_HOSTNAME="rpi-$MAC_SUFFIX"
echo "New hostname will be: $NEW_HOSTNAME"
echo "$NEW_HOSTNAME" | sudo tee /etc/hostname
sudo sed -i "s/127.0.1.1.*/127.0.1.1\t$NEW_HOSTNAME/" /etc/hosts
sudo hostnamectl set-hostname "$NEW_HOSTNAME"


### Configure Wi-Fi Access Point
echo "=========================================="
echo "Configure Wi-Fi Access Point"
echo "=========================================="

# Install necessary packages for Wi-Fi Access Point
echo "Installing hostapd, dhcpcd5, dnsmasq, and network-manager..."
sudo apt install -y hostapd dhcpcd5 dnsmasq network-manager

SSID="$NEW_HOSTNAME"
echo "---------------------------------------------------------"
echo "Interface             wlan0"
echo "SSID                  $SSID"
echo "Passphrase            12345678"
echo "Hardware mode         802.11ac"
echo "Channel               149"
echo "Country code          CN"
echo "HT Capabilities       HT40- HT40+ SHORT-GI-40 DSSS_CCK-40"
echo "WPA version           2"
echo "WPA key management    WPA-PSK"
echo "WPA pairwise          CCMP"
echo "RSN pairwise          CCMP"
echo "---------------------------------------------------------"
sudo tee /etc/hostapd/hostapd.conf > /dev/null <<EOF
interface=wlan0
driver=nl80211
ssid=$SSID
hw_mode=a
ieee80211n=1
ieee80211ac=1
require_ht=1
require_vht=1
wmm_enabled=1
country_code=CN
ht_capab=[HT40-][HT40+][SHORT-GI-40][DSSS_CCK-40]
channel=149
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=12345678
wpa_key_mgmt=WPA-PSK
wpa_pairwise=CCMP
rsn_pairwise=CCMP
EOF

# Link hostapd to the configuration file
echo "Linking hostapd to the configuration file..."
sudo sed -i '/^#DAEMON_CONF/c\DAEMON_CONF="/etc/hostapd/hostapd.conf"' /etc/default/hostapd

# Static IP for wlan0
echo "Configuring static IP for wlan0..."
echo "WiFi IP address will be set to 192.168.4.1"
sudo tee -a /etc/dhcpcd.conf > /dev/null <<EOF

interface=wlan0
static ip_address=192.168.4.1/24
EOF

# DHCP server config
echo "Configuring DHCP server for wlan0..."
sudo tee /etc/dnsmasq.conf > /dev/null <<EOF
interface=wlan0
dhcp-range=192.168.4.2,192.168.4.254,255.255.255.0,24h
EOF

# Make NetworkManager ignore wlan0
echo "Configuring NetworkManager to ignore wlan0..."
sudo mkdir -p /etc/NetworkManager/conf.d
sudo tee /etc/NetworkManager/conf.d/10-ignore-wlan0.conf > /dev/null <<EOF
[keyfile]
unmanaged-devices=interface-name:wlan0
EOF

# Bring up wlan0
# Note that before bringing up wlan0, wifi country must be set
echo "Bringing up wlan0..."
sudo ip link set wlan0 up

# Enable and start Wi-Fi services
echo "Enabling and starting hostapd, dhcpcd, and dnsmasq services..."
sudo systemctl unmask hostapd dhcpcd dnsmasq
sudo systemctl enable hostapd dhcpcd dnsmasq
sudo systemctl start hostapd dhcpcd dnsmasq

# Restart NetworkManager to apply changes
echo "Restarting NetworkManager to apply changes..."
sudo systemctl restart NetworkManager

# Ensure UI is installed in case of graphical fallback
echo "Installing Raspberry Pi UI mods..."
sudo apt install -y raspberrypi-ui-mods


### Configure CAN bus
echo "=========================================="
echo "Configure CAN bus"
echo "=========================================="
# Install necessary packages for CAN bus
echo "Installing CAN bus packages..."
sudo apt install -y can-utils

# Enable SPI and configure CAN interfaces
echo "Enabling SPI and configuring CAN interfaces..."
# Note: This assumes the MCP2518FD or MCP2517FD CAN controller is used
# Adjust the device tree overlays as necessary for your hardware setup
CONFIG_TXT="/boot/firmware/config.txt"
sudo tee -a $CONFIG_TXT > /dev/null <<EOF

# CAN configuration
dtparam=spi=on
dtoverlay=spi1-3cs
dtoverlay=mcp251xfd,spi0-0,interrupt=25
dtoverlay=mcp251xfd,spi1-0,interrupt=24
EOF

# Prevent DHCP from managing CAN interfaces
echo "Preventing DHCP from managing CAN interfaces..."
sudo tee -a /etc/dhcpcd.conf > /dev/null <<EOF
denyinterfaces can0
denyinterfaces can1
EOF

# Create setup script for CAN interfaces
echo "Creating setup script for CAN interfaces..."
echo "------------------------------------------------------------"
echo "Interface  Bitrate  DBitrate  Restart MS  Berr Reporting  FD"
echo "------------------------------------------------------------"
echo "can0       1000000  8000000   1000        on              on"
echo "can1       1000000  8000000   1000        on              on"
echo "------------------------------------------------------------"
# Create a script to set up CAN interfaces
sudo tee /usr/local/bin/setup_can.sh > /dev/null <<'EOF'
#!/bin/bash
# Setup CAN0
sudo ip link set can0 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
sudo ifconfig can0 txqueuelen 65536

# Setup CAN1
sudo ip link set can1 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
sudo ifconfig can1 txqueuelen 65536
EOF

# Make it executable
sudo chmod +x /usr/local/bin/setup_can.sh

# Register systemd service to run CAN setup on boot
echo "Registering systemd service to run CAN setup on boot..."
sudo tee /etc/systemd/system/can-setup.service > /dev/null <<EOF
[Unit]
Description=Setup CAN interfaces on boot
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup_can.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

echo "Enabling and starting CAN setup service..."
sudo systemctl enable can-setup.service


### Step 6: Configure cooling fan
echo "=========================================="
echo "❄️ Step 6: Configure cooling fan"
echo "=========================================="
echo "Configuring cooling fan settings..."
# Ensure the cooling fan is enabled and configured
# This assumes the cooling fan is connected to the GPIO pins and uses the device tree overlay
# Adjust the fan parameters as necessary for your cooling requirements
echo "Cooling fan configuration:"
echo "---------------------"
echo "Temperature   Speed"
echo "---------------------"
echo "36°C          90/255"
echo "40°C          150/255"
echo "52°C          200/255"
echo "58°C          255/255"
echo "---------------------"
sudo tee -a $CONFIG_TXT > /dev/null <<EOF

# Cooling fan configuration
dtparam=cooling_fan=on
dtparam=fan_temp0=36000,fan_temp0_hyst=2000,fan_temp0_speed=90
dtparam=fan_temp1=40000,fan_temp1_hyst=3000,fan_temp1_speed=150
dtparam=fan_temp2=52000,fan_temp2_hyst=4000,fan_temp2_speed=200
dtparam=fan_temp3=58000,fan_temp3_hyst=5000,fan_temp3_speed=255
EOF


### Final message
echo "=========================================="
echo "Setup completed successfully."
echo "Reboot the system to apply all changes."
echo "=========================================="
