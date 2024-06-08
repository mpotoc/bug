#!/bin/bash

#MAC_AD="$(sed -e 's/://g' /sys/class/net/eth0/address 2>/dev/null || sed -e 's/://g' /sys/class/net/wlan0/address 2>/dev/null)"
SER_NR="$(cat /proc/cpuinfo | grep Serial | cut -d ' ' -f 2)"
echo -e ${SER_NR} >/opt/wb_gateway/vpn/waterBug.creds
echo -e 4Sd8eSodyGs1TQgGfkepo20dd56gSf >>/opt/wb_gateway/vpn/waterBug.creds

NEWHOSTNAME=rpi-${SER_NR}
echo -e '\n\n\t   updating hostname ...\n'
    sed -i -e "s/raspberrypi/${NEWHOSTNAME}/g" /etc/hosts
    echo ${NEWHOSTNAME}>/etc/hostname

cd /tmp/
# screen -dmS mavproxy mavproxy.py --master=/dev/ttyAMA0 --baudrate=115200 --force-connected --aircraft=WaterBug --out=udpin:0.0.0.0:14550
# sleep 5
screen -dmS wb_gateway python3 /opt/wb_gateway/wbgw.py