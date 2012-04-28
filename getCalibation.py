#!/usr/bin/python
import usb, sys

print usb.core.find(idVendor=0x59e3, idProduct=0x74C7).ctrl_transfer(0x40|0x80, 0x6C, 0, int(sys.argv[1]), 12)
