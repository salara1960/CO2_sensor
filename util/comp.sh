#!/bin/sh

./util --mode=compare --file=new.bin --dev=/dev/ttyUSB0

sleep 3

minicom -R utf-8 usb0

