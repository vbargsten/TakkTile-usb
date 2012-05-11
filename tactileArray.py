#! /usr/bin/python

import usb
import numpy
import time

class Tactile:
	def __init__(self):
		self.dev = usb.core.find(idVendor=0x59e3, idProduct=0x74C7)
		"""initialize the USB device
		load calibration information from USB into Tactile.calibration"""
		# calibrationData is 8 bytes per sensor
		self.calibrationData = 8*[5*[8*[0]]]
		# calibratedData is a set of floating point numbers
		self.calibratedData = 8*[5*[0.5]]
		# rawData is a set of fixed point numbers
		self.rawData = 8*[5*[512]]

	def _getTinyAddressFromRowColumn(self, row, column = 0):
		tinyAddr = ((row&0x0F) << 4 | (column&0x07) << 1)
		return tinyAddr

	def getDataRaw(self, row):
		"""return an array of five integers between 0 and 1023, matching the 10b sample depth of the sensors."""
		data = self.dev.ctrl_transfer(0x40|0x80, 0x7C, 0, row, 20)
		data = numpy.resize(data, (5,4))
		#print [map(hex, datum[0:2]) for datum in data]
		temperature = [((datum[3] >> 6| datum[2] << 2)) for datum in data]
		data = [((datum[1] >> 6| datum[0] << 2)) for datum in data]
		self.rawData[row] = data
		# return the 1x5 array
		return data, temperature

	def getDataRawSeq(self, row):
		"""return an array of five integers between 0 and 1023, matching the 10b sample depth of the sensors."""
		data = self.dev.ctrl_transfer(0x40|0x80, 0x8C, 0, row, 20)
		data = numpy.resize(data, (5,4))
		#print [map(hex, datum[0:2]) for datum in data]
		temperature = [((datum[3] >> 6| datum[2] << 2)) for datum in data]
		data = [((datum[1] >> 6| datum[0] << 2)) for datum in data]
		self.rawData[row] = data
		# return the 1x5 array
		return data, temperature

	def getData(self, row):
		"""return an array of five floating point numbers between 0 and 1, calibrated and temperature compensated"""
		# get raw data
		data = self.getDataRaw(row)
		# do math
		data = [datum/1023.0 for datum in data]
		# record data in internal object
		self.calibratedData[row] = data
		# return the 1x5 array
		return data
	
	def getCalibrationData(self, row, column):
		"""return the eight-item-long list of calibration bytes"""
		return self.calibrationData[row][column]

if __name__ == "__main__":
	tact = Tactile()
	shotgun = [ tact.getDataRaw(1)[0] for sample in range(1000) ]
	shotgun = numpy.rollaxis(numpy.array(shotgun), 1)
	sequential = [ tact.getDataRawSeq(1)[0] for sample in range(1000) ]
	sequential = numpy.rollaxis(numpy.array(sequential), 1)
	print numpy.std(sequential[0]), numpy.std(shotgun[0])
	print numpy.mean(sequential[0]), numpy.mean(shotgun[0])	
