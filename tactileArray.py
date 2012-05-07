#! /usr/bin/python

import random
import pprint

class Tactile:
	def __init__(self):
		"""initialize the USB device
		load calibration information from USB into Tactile.calibration"""
		# calibrationData is 8 bytes per sensor
		self.calibrationData = 8*[5*[8*[0]]]
		# calibratedData is a set of floating point numbers
		self.calibratedData = 8*[5*[0.5]]
		# rawData is a set of fixed point numbers
		self.rawData = 8*[5*[512]]

	def getDataRaw(self, row):
		"""return an array of five integers between 0 and 1023, matching the 10b sample depth of the sensors."""
		# bogus data for testing purposes
		data = [random.randint(0,1023) for i in range(5)]
		#data = self.doUsbControlTransfer(row)
		self.rawData[row] = data
		# return the 1x5 array
		return data

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
	pprint.pprint([tact.getDataRaw(row) for row in range(8)])
