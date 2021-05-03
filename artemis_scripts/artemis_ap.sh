#!/bin/sh
sudo ip link set down dev $1
sudo ip link set mtu 1500 dev $1
sudo iwconfig $1 mode manual
#sudo iwconfig $1 essid arty-net
#sudo iwconfig $1 ap 02:12:34:56:78:9A
#sudo iwconfig $1 channel 8
sleep 1s
sudo /usr/sbin/hostapd hostapd.conf
sudo ifconfig $1 192.168.8.1/24
sudo ip link set up dev $1
