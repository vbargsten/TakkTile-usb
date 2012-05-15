#! /usr/bin/python

import numpy

#TODO: AT THE MOMENT IT WORKS ONLY WITH ONE ROW !!!

class TactileDisplay:
	def __init__(self):
		import tactileArray
		self.tArray = tactileArray.Tactile()
		self.pressure=[]
		self.temperature=[]
		self.calibrationRaw=self.tArray.calibrationData[1]
		self.calculateCalibrationCoefficients()

	def calculateCalibrationCoefficients(self):
		d=self.calibrationRaw
		lenCal = len(d)
		self.calibration=numpy.array([6*[0] for i in range(lenCal)])
		for i in range(lenCal):
			# a0
			self.calibration[i][0]=numpy.array((d[i][0] <<8 | d[i][1]) ,dtype=numpy.int16)
			# b1
			self.calibration[i][1]=numpy.array((d[i][2] <<8 | d[i][3]),dtype=numpy.int16)
			# b2
			self.calibration[i][2]=numpy.array((d[i][4] <<8 | d[i][5]),dtype=numpy.int16)
			# c12
			self.calibration[i][3]=numpy.array((d[i][6] <<8 | d[i][7]),dtype=numpy.int16)
			# c11
			self.calibration[i][4]=numpy.array((d[i][8] <<8 | d[i][9]),dtype=numpy.int16)
			# c22
			self.calibration[i][5]=numpy.array((d[i][10] <<8 | d[i][11]),dtype=numpy.int16)

			# for Debugging
			# self.calibration[i][0]=numpy.array(0x41DF,dtype=numpy.int16)
			# self.calibration[i][1]=numpy.array(0xB028,dtype=numpy.int16)
			# self.calibration[i][2]=numpy.array(0xBEAD,dtype=numpy.int16)
			# self.calibration[i][3]=numpy.array(0x38CC,dtype=numpy.int16)


	def getCalibratedData(self):

		self.pressure,self.temperature=self.tArray.getDataRaw(1)

		Padc=numpy.array(self.pressure)
		Padc=numpy.array(Padc << 6,dtype=numpy.int16)
		Padc=numpy.array(Padc >> 6,dtype=numpy.int16)

		Tadc=numpy.array(self.temperature)
#		Tadc=numpy.array(Tadc << 6,dtype=numpy.int16)
#		Tadc=numpy.array(Tadc >> 6,dtype=numpy.int16)

		# DEBUG - IAN, WHY THESE TWO VALUES ARE DIFFERENT ?!
		#T=numpy.array(self.temperature[1])
		#T=numpy.array(T << 6,dtype=numpy.int16)
		#T=numpy.array(T >> 6,dtype=numpy.int16)
		#print T, self.temperature[1]

		Pcomp = [0 for i in range(len(Padc))]

		for i in range(len(Padc)):
			# a0= 0x41DF = 2107.875
			a0 = float(self.calibration[i][0])/(8.0)  #2^3
			# b1 = 0xB028 = -2.49512
			b1 = float(self.calibration[i][1])/(8192.0) #2^13
			# % b2 - 0xBEAD = -1.02069
			b2 = float(self.calibration[i][2])/(16384.0) #2^14
			# % c12 - 0x38CC = 0.00086665
			c12 = float(self.calibration[i][4])/(16777216.0) #2^24

			c12x2 = c12 * Tadc[i]
			a1 = b1 + c12x2
			a1x1 = a1 * Padc[i]
			y1 = a0 + a1x1
			a2x2 = b2 * Tadc[i]

			# should be 733.29270
			Pcomp[i] = y1 + a2x2

#		return 	Pcomp
		return 	numpy.array(Pcomp,dtype=numpy.int16)


if __name__ == "__main__":
	tact = TactileDisplay()
	while True:
		print tact.getCalibratedData()
