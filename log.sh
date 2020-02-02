#!/bin/sh
platformio device monitor --port /dev/ttyUSB0 --baud 115200 --raw 2>&1 | tee -a serial.log
