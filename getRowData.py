#!/usr/bin/python
import usb, sys

print map(hex, usb.core.find(idVendor=0x59e3, idProduct=0x74C7).ctrl_transfer(0x40|0x80, 0x7C, 0, 1, 20))
