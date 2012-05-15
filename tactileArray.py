#! /usr/bin/python
# (C) 2012 Biorobotics Lab and Nonolith Labs                                                                                                                                                    
# Written by Ian Daniher
# Licensed under the terms of the GNU GPLv3+

import usb
import numpy
import time

class Tactile:
	def __init__(self):
		self.dev = usb.core.find(idVendor=0x59e3, idProduct=0x74C7)
		"""initialize the USB device
		load calibration information from USB into Tactile.calibration"""
		# calibrationData is 12 bytes per sensor
		self.calibrationData = 9*[5*[12*[0]]]
		# those eight bytes contain six coefficients
		self.calibrationCoefficients = 9*[5*[{"a0":0, "b1":0, "b2":0, "c12":0, "c11":0, "c22":0}]]
		# get the calibration bytes for all the sensors in row1
		self.calibrationData[1] = [self.getCalibrationData(1, column) for column in range(5)]
		# calculate the coefficients
		self.calculateCalibrationCoefficients()

	
	def _getTinyAddressFromRowColumn(self, row, column = 0):
		tinyAddr = ((row&0x0F) << 4 | (column&0x07) << 1)
		return tinyAddr

	def calculateCalibrationCoefficients(self):
		def unTwos(x, bitlen):
			if( (x&(1<<(bitlen-1))) != 0 ):
				x = x - (1<<bitlen)
			return x
		for row in [1]:
			for column in range(5):
				cd = self.calibrationData[row][column] 
				cc = self.calibrationCoefficients[row][column]
				# undo Two's complement if applicable, pack into proper bit width
				cc["a0"] = unTwos(((cd[0] << 8) | cd[1]), 16)
				cc["b1"] = unTwos(((cd[2] << 8) | cd[3]), 16)
				cc["b2"] = unTwos(((cd[4] << 8) | cd[5]), 16)
				cc["c12"] = unTwos(((cd[6] << 6) | (cd[7] >> 2)), 14)
				cc["c11"] = unTwos(((cd[8] << 3) | (cd[9] >> 5)), 11)
				cc["c22"] = unTwos(((cd[10] << 3) | (cd[11] >> 5)), 11)
				# divide by float(1 << (fractionalBits + zeroPad)) to handle weirdness
				cc["a0"] /= float(1 << 3)
				cc["b1"] /= float(1 << 13)
				cc["b2"] /= float(1 << 14)
				cc["c12"] /= float(1 << 22)
				cc["c11"] /= float(1 << 21)
				cc["c22"] /= float(1 << 25)
		print self.calibrationCoefficients[1]

	def getDataRaw(self, row):
		"""return an array of five integers between 0 and 1023, matching the 10b sample depth of the sensors."""
		data = self.dev.ctrl_transfer(0x40|0x80, 0x7C, 0, row, 20)
		data = numpy.resize(data, (5,4))
		#print [map(hex, datum[0:2]) for datum in data]
		temperature = [((datum[3] >> 6| datum[2] << 2)) for datum in data]
		data = [((datum[1] >> 6| datum[0] << 2)) for datum in data]
		# return the 1x5 array
		return data, temperature

	def getData(self, row):
		"""return an array of five floating point numbers between 0 and 1, calibrated and temperature compensated"""
		# get raw data
		Padc, Tadc = self.getDataRaw(row)
		Pcomp = len(Padc)*[0]
		# for element in the returned pressure data
		for column in range(len(Padc)):
			# load the calibration coefficients calculated when Tactile is initialized
			cc = self.calibrationCoefficients[row][column]
			# apply the formula contained on page 13 of Freescale's AN3785
			# "The 10-bit compensated pressure output for MPL115A, Pcomp, is calculated as follows: 
			#  Pcomp = a0 + (b1 + c11*Padc + c12*Tadc) * Padc + (b2 + c22*Tadc) * Tadc"
			Pcomp[column] = cc["a0"] + (cc["b1"] + cc["c11"]*Padc[column] + cc["c12"]*Tadc[column])*Padc[column] + (cc["b2"] + cc["c22"]*Tadc[column])*Tadc[column]
		return Pcomp 
	
	def getCalibrationData(self, row, column):
		"""return the 12-item-long list of calibration bytes"""
		tinyAddr = self._getTinyAddressFromRowColumn(row, column)
		self.calibrationData[row][column] = self.dev.ctrl_transfer(0x40|0x80, 0x6C, 0, tinyAddr, 12)
		return self.calibrationData[row][column]

if __name__ == "__main__":
	tact = Tactile()
	print tact.getData(1)
