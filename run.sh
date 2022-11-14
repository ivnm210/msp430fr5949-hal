#!/usr/bin/env bash
#mspdebug --allow-fw-update tilib "prog $1 & run"
msp430-elf-objcopy -Oihex $1 $1.hex
#LD_LIBRARY_PATH=/opt/msp430-gcc/bin ~/mspdebug/mspdebug tilib -d /dev/ttyACM0 "prog $1.hex"
sudo chmod a+rw /dev/ttyACM0
LD_LIBRARY_PATH=~/ti/MSPFlasher_1.3.20 ~/ti/ccs1020/ccs/ccs_base/DebugServer/bin/mspdebug tilib --allow-fw-update -d /dev/ttyACM0 "prog $1.hex"
