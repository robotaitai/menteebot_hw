#!/bin/bash
modprobe can
modprobe can_raw

/sbin/ip link set can5 up type can bitrate 1000000
/sbin/ip link set can6 up type can bitrate 1000000

/sbin/ip link set can7 up type can bitrate 1000000
/sbin/ip link set can8 up type can bitrate 1000000
#/sbin/ip link set can9 up type can bitrate 1000000


