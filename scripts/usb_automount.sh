#!/bin/sh

if [ -e "/dev/sda1" ]; then
    /usr/bin/mount /dev/sda1 /media/usb
fi

