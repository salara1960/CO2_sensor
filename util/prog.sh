#!/bin/sh

./util --mode=prog --file=new.bin --dev=/dev/ttyUSB0

sleep 3

minicom -R utf-8 usb0 -C log_usb0.cap

