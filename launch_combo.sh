#!/usr/bin/bash
set -e
sudo modprobe bcm2835_smi
sudo modprobe bcm2835_smi_dev
g++ rc_combo_demo.cpp -lm -g -Og
taskset -c 3 sudo chrt -f 10 ./a.out
