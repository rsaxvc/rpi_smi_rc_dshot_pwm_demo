#!/usr/bin/bash
set -e
sudo modprobe bcm2835_smi
sudo modprobe bcm2835_smi_dev
gcc rc_pwm_demo.c -lm
./a.out
