#!/usr/bin/bash
set -e
sudo modprobe bcm2835_smi
sudo modprobe bcm2835_smi_dev
gcc rc_dshot600_demo.c -lm
sudo chrt -f 10 ./a.out
