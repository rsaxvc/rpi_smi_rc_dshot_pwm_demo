#!/usr/bin/bash
set -e
sudo modprobe bcm2835_smi
sudo modprobe bcm2835_smi_dev
gcc main.c
./a.out
