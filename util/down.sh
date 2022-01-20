#!/bin/sh

./util --mode=download --dev=/dev/ttyUSB0

sleep 3

minicom -R utf-8 usb0

