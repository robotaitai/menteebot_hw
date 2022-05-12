#!/bin/bash
modprobe -vr can
modprobe -vr can_raw
/sbin/ip link set can5 down
/sbin/ip link set can6 down
/sbin/ip link set can7 down
/sbin/ip link set can8 down
/sbin/ip link set can9 down



echo "Can is Down"
