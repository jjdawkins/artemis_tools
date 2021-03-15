#!/bin/sh
sudo ip link set down dev $1
sudo ip link set mtu 1532 dev $1
sudo iwconfig $1 mode ad-hoc
sudo iwconfig $1 essid arty-net
sudo iwconfig $1 ap 02:12:34:56:78:9A
sudo iwconfig $1 channel 8
sleep 1s
sudo ip link set up dev $1

ifconfig $1 192.168.8.$2/24
