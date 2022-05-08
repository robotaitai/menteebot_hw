#!/bin/bash

/sbin/ip link set can5 down
/sbin/ip link set can6 down
/sbin/ip link set can7 down
/sbin/ip link set can8 down
/sbin/ip link set can9 down

modprobe -vr can
modprobe -vr can_raw

echo "Can is Down"
