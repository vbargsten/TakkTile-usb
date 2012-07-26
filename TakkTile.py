#! /usr/bin/python
# (C) 2012 Biorobotics Lab and Nonolith Labs                                                                                                                                                    
# Written by Ian Daniher
# Licensed under the terms of the GNU GPLv3+

import usb
import re

unTwos = lambda x, bitlen: x-(1<<bitlen) if (x&(1<<(bitlen-1))) else x

class TakkTile:

	_getTinyAddressFromRowColumn = lambda self, row, column: (((row)&0x0F) << 4 | (column&0x07) << 1)
	exists = lambda self: bool(self.arrayID)^1

	def __init__(self, arrayID = 0):
		# search for a USB device with the proper VID/PID combo
		self.dev = usb.core.find(idVendor=0x59e3, idProduct=0x74C7)
		if self.dev == None:
			print("Can't find TakkTile USB interface!")
			quit()
		self.arrayID = arrayID
		# populates bitmap of live sensors
		self.alive = self.getAlive()
		# retrieve calibration bytes and calculate the polynomial's coefficients
		self.calibrationCoefficients = [[self.getCalibrationCoefficients(row, column) for column in range(5)] for row in range(8)]
		self.UID = self.dev.ctrl_transfer(0x80, usb.REQ_GET_DESCRIPTOR, 
			(usb.util.DESC_TYPE_STRING << 8) | self.dev.iSerialNumber, 0, 255)[2::].tostring().decode('utf-16')

	def getAlive(self):
		""" Return an array containing the cell number of all alive cells. """
		pad = lambda x: bin(x)[2::].zfill(5)[::-1]
		bitmap = self.dev.ctrl_transfer(0x40|0x80, 0x5C, 0, 0, 8)
		bitmap = ''.join(map(pad, bitmap))
		return [match.span()[0] for match in re.finditer('1', bitmap)] 
	

	def getCalibrationCoefficients(self, row, column):
		""" This function implements the compensation & calibration coefficient calculations from page 15 of AN3785. """
		# iterate through rows and columns
		# get calibration data from a specified location
		cd = self.getCalibrationData(row, column)  
		# define short alias to save me keystrokes & enhance readability
		cc = {"a0":0, "b1":0, "b2":0, "c12":0, "c11":0, "c22":0}
		# cell not alive
		if max(cd) == 0:
			return cc
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
		return cc


	def getDataRaw(self, row):
		"""Query the TakkTile USB interface for the pressure and temperature samples from a specified row of sensors.."""
		_chunk = lambda l, x: [l[i:i+x] for i in xrange(0, len(l), x)]
		# 0x7C is "get data" vendor request, takes a row as a wValue, returns 20 bytes
		data = self.dev.ctrl_transfer(0x40|0x80, 0x7C, 0, row, 20)
		# use _chunk to shape the 20 bytes into five chunks of four bytes each
		data = _chunk(data, 4)
		# temperature is contained in the last two bytes of each four byte chunk, pressure in the first two
		# each ten bit number is encoded in two bytes, MSB first, zero padded / left alligned
		temperature = [unTwos((datum[3] >> 6| datum[2] << 2), 10) for datum in data]
		data = [unTwos((datum[1] >> 6| datum[0] << 2), 10) for datum in data]
		# fix overflow problems
		return data, temperature

	def getData(self, row):
		"""Return measured pressure in kPa, temperature compensated and factory calibrated."""
		# get raw 10b data
		Padc, Tadc = self.getDataRaw(row)
		print Padc, Tadc
		# initialize array for compensated pressure readings
		Pcomp = len(Padc)*[0]
		# for element in the returned pressure data...
		for column in range(len(Padc)):
			# load the calibration coefficients calculated when the TakkTile class is initialized
			cc = self.calibrationCoefficients[row][column]
			# apply the formula contained on page 13 of Freescale's AN3785
			# "The 10-bit compensated pressure output for MPL115A, Pcomp, is calculated as follows: 
			#  Pcomp = a0 + (b1 + c11*Padc + c12*Tadc) * Padc + (b2 + c22*Tadc) * Tadc"
			Pcomp[column] = cc["a0"] + (cc["b1"] + cc["c11"]*Padc[column] + cc["c12"]*Tadc[column])*Padc[column] + (cc["b2"] + cc["c22"]*Tadc[column])*Tadc[column]
			# convert from 10b number to kPa
			Pcomp[column] = 65.0/1023.0*Pcomp[column]+50
			# round to keep sane sigfig count
			Pcomp[column] = round(Pcomp[column], 4)
		return Pcomp 
	
	def getCalibrationData(self, row, column):
		"""Request the 12 calibration bytes from a sensor at a specified row and column."""
		# get the attiny's virtual address for the specified row/column
		# read the calibration data via vendor request and return it 
		return self.dev.ctrl_transfer(0x40|0x80, 0x6C, column, row, 12)	

if __name__ == "__main__":
	tact = TakkTile()
	print tact.alive
	print tact.UID
	import time
	while True:
		start = time.time()
		data = tact.getData(0)
		end = time.time()
		print round(end-start, 6), data
		time.sleep(.005)
