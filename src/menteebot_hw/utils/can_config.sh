#!/bin/bash
modprobe can
modprobe can_raw

/sbin/ip link set can5 up type can bitrate 1000000
/sbin/ip link set can6 up type can bitrate 1000000
/sbin/ip link set can4 up type can bitrate 1000000 #restart-ms 100
/sbin/ip link set can8 up type can bitrate 1000000
/sbin/ip link set can9 up type can bitrate 1000000

#ifconfig can4 txqueuelen 100
#ifconfig can5 txqueuelen 100
#ifconfig can6 txqueuelen 100
#ifconfig can8 txqueuelen 100
#ifconfig can9 txqueuelen 100
