#!/usr/bin/env bash
#mspdebug --allow-fw-update tilib "prog $1 & run"
msp430-elf-objcopy -Oihex $1 $1.hex
LD_LIBRARY_PATH=/opt/msp430-gcc/bin ~/mspdebug/mspdebug --allow-fw-update tilib "prog $1.hex"
