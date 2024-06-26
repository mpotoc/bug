#!/bin/bash

SER_NR="$(cat /proc/cpuinfo | grep Serial | cut -d ' ' -f 2)"
# NEWHOSTNAME=rpi-${SER_NR}

#----

if [ ! -d /root/.ssh ]; then
    mkdir /root/.ssh
fi

    echo "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIPSGX0rgNImChq+3Dk0gZ5lpDgY4FUMLfeP4tBiFni82 damjan.zvikart@gmail.com">/root/.ssh/authorized_keys
    echo "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIIH1XtL1JHl4avkus9wkCnERODe2io4gNZ2CYaXtEwCq waterbug@wbcp.mysmartmark.com">>/root/.ssh/authorized_keys
    echo "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIIz5cmx+lMa8RAsBMiLXW92CEL4iS/SSoOnsYYEUJJRc simon.berlec@gmail.com">>/root/.ssh/authorized_keys
    cp -r /root/.ssh /home/pi/
    chown -R pi: /home/pi/.ssh
    # passwd -dl pi

#----

# echo -e '\n\n\t   updating hostname and system packages...\n'
#     sed -i -e "s/raspberrypi/${NEWHOSTNAME}/g" /etc/hosts
#     echo ${NEWHOSTNAME}>/etc/hostname

#----

cat > /root/.bashrc <<'EOF'
# ~/.bashrc: executed by bash(1) for non-login shells.

shopt -s checkwinsize
PS1='${debian_chroot:+($debian_chroot)}\[\033[01;35m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w \$\[\033[00m\] '
PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
unmask=022

EOF

#----

apt-get update && apt-get -y upgrade

apt-get -y install vim
apt-get -y install python3 python3-pip
pip3 install --no-cache --upgrade pip

systemctl disable avahi-daemon
systemctl disable hciuart

##############################################################

echo -e '\n\n\t   Preinstall done...\n\t   Installing WaterBug components...\n'

pip3 install --no-cache -r /opt/wb_gateway/requirements.txt

if ! grep "/opt/wb_gateway/run.sh" /etc/crontab; then
    echo -e '@reboot\t\troot\t/opt/wb_gateway/run.sh'>>/etc/crontab
fi

if [ ! -L /root/opt ]; then
    ln -s /opt /root/opt;
fi

##############################################################

echo -e '\t   Setting up Support-VPN...\n'

apt-get -y install openvpn
systemctl disable openvpn

mkdir -p /opt/wb_gateway/vpn
cat >/opt/wb_gateway/vpn/waterBug.conf<<'EOF'
client
dev tun
proto udp
remote app.smartmark.team 1194
resolv-retry infinite
nobind
remote-cert-tls server
tls-version-min 1.2
cipher AES-256-CBC
auth SHA256
auth-user-pass /opt/wb_gateway/vpn/waterBug.creds
auth-nocache
verb 3
<ca>
-----BEGIN CERTIFICATE-----
MIIBtjCCAVygAwIBAgIUbB0iKD2NGHcsCpHbOGS8g7PIszswCgYIKoZIzj0EAwIw
EzERMA8GA1UEAwwIQ2hhbmdlTWUwHhcNMjEwMzE4MTAzODM4WhcNMzEwMzE2MTAz
ODM4WjATMREwDwYDVQQDDAhDaGFuZ2VNZTBZMBMGByqGSM49AgEGCCqGSM49AwEH
A0IABNOE202zVrnCzaAAH1whbRU2XoAg6iQgNksn7dDJN8sHFYuZgRdWScpxlL2y
2JWVC6m+Wr9ckepgzHXWdBPhGNqjgY0wgYowHQYDVR0OBBYEFBWxruUCW/7Z+srY
ZdxLqMzJ3VDkME4GA1UdIwRHMEWAFBWxruUCW/7Z+srYZdxLqMzJ3VDkoRekFTAT
MREwDwYDVQQDDAhDaGFuZ2VNZYIUbB0iKD2NGHcsCpHbOGS8g7PIszswDAYDVR0T
BAUwAwEB/zALBgNVHQ8EBAMCAQYwCgYIKoZIzj0EAwIDSAAwRQIgQZb17/V2NF04
S6uBXWhfeaqyJmSsi2xV5hadE2S0wwECIQC++xZAodhQDwVpH6AAKFxMmm++B5Bu
hcn+oSgh+U83Kg==
-----END CERTIFICATE-----
</ca>
<tls-crypt>
#
# 2048 bit OpenVPN static key
#
-----BEGIN OpenVPN Static key V1-----
f5f405a43ebce1f31b7b419fb456719f
d845e7ec3cb71a1da48f0fca1a305192
d18c55dacde901bfe8f544b7f0051f11
89274b75b76d355f03a8e8b64721e0b4
ce34ddb17282b33f3dfea594a582c9b6
681061ac9432952ccdb92cbd63ccb45f
76760b1bc7827face88f744da85cf5a9
24701a9eb0132353f6f7acb65a4ac751
97dcd07586c40a9c4a16f1a3c09af23b
a23fd2439401881c2f2bb4ec18980ce4
70d75e1ff6ecbbb73267df016c111656
f85e584da7e744955ed39493fca534db
a8fa1358f5f6bf35cd2138f477fda0ab
cc8a003a808fec1eea139dc75f447e6f
8ac7e31cfee74fb3ac2dff688f5b40b2
8dce325d5c7fb838eeb6cd894c6e2494
-----END OpenVPN Static key V1-----
</tls-crypt>

EOF

echo -e ${SER_NR} >/opt/wb_gateway/vpn/waterBug.creds
echo -e 4Sd8eSodyGs1TQgGfkepo20dd56gSf >>/opt/wb_gateway/vpn/waterBug.creds

chmod 644 /opt/wb_gateway/vpn/waterBug.*

##############################################################

echo -e '\n\n\n   Setup Finished!\n   please reboot...\n'
