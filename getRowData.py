#!/usr/bin/python
import usb, sys, numpy, time
dev = usb.core.find(idVendor=0x59e3, idProduct=0x74C7)
while True:
	values = dev.ctrl_transfer(0x40|0x80, 0x7C, 0, 1, 20)
	values = numpy.resize(values, (5,4))
	values = [value[1] | value[0] << 2 for value in values]
	time.sleep(.1)
	print values
