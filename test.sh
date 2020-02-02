#!/bin/sh
platformio run -e espressif32 -t upload && platformio device monitor
