#! /usr/bin/python
# (C) 2012 Biorobotics Lab and Nonolith Labs                                                                                                                                                    
# Written by Ian Daniher
# Licensed under the terms of the GNU GPLv3+

# updated Jan/1/2015 to include a number of TakkFast devices

import usb
import re
import itertools
import sys


_unTwos = lambda x, bitlen: x-(1<<bitlen) if (x&(1<<(bitlen-1))) else x
_chunk = lambda l, x: [l[i:i+x] for i in range(0, len(l), x)]
_flatten = lambda l: list(itertools.chain(*[[x] if type(x) not in [list] else x for x in l]))

class TakkTile:

    # get I2C address for a given cell from varying other references
    _getTinyAddressFromRowColumn = lambda self, row, column: (((row)&0x0F) << 4 | (column&0x07) << 1)
    _getTinyAddressFromIndex = lambda self, index: (((index/5)&0x0F) << 4 | ((index%5)&0x07) << 1)

    def __init__(self, arrayID = 0):
        # search for a USB device with the proper VID/PID combo
        self.devs = list(usb.core.find(find_all=1, idVendor=0x59e3, idProduct=0x74C7))
        
        print("Found", len(self.devs), "devices.")
        if len(self.devs) < 1:
            raise Exception("Can't find TakkTile USB interface!")
            #quit()

        # if there are more than one device attached, then arrange by xmegas serial number.
        dev_dict={}
        for dev in self.devs:
            # populate self.UID with vendor request to get the xmega's serialNumber
            UID = dev.ctrl_transfer(0x80, usb.REQ_GET_DESCRIPTOR, 
                (usb.util.DESC_TYPE_STRING << 8) | dev.iSerialNumber, 0, 255)[2::].tostring().decode('utf-16')
            dev_dict.update({UID: dev})

        self.devs = []
        for key in sorted(dev_dict.keys()):
            self.devs.append(dev_dict[key])
        self.UIDs=sorted(dev_dict.keys())

        self.arrayID = arrayID
        self.dev=self.devs[arrayID]
        
        #self.dev.reset()
        
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)
            self.dev.attach_kernel_driver(0)
    
        # populates bitmap of live sensors
        # for each following device offset 50 in indexing
        self.alive = [x+arrayID*50 for x in self.getAlive()]

        
        # calibrationCoefficients is a dictionary mapping cell index to a dictionary of calibration variables 
        self.calibrationCoefficients=(dict(list(map(self.getCalibrationCoefficients, self.alive))))

    def getAlive(self):
        """ Return an array containing the cell number of all alive cells. """
        # get an eight byte bitmap of live sensors
        bitmap = self.dev.ctrl_transfer(0x40|0x80, 0x5C, 0, 0, 8)
        # for each byte, convert it to binary format, trim off '0b', zerofill, and join to a shared string
        bitmap = ''.join([bin(x)[2::].zfill(5)[::-1] for x in bitmap])
        # find and return the index of all '1's
        return [match.span()[0] for match in re.finditer('1', bitmap)]
    

    def getCalibrationCoefficients(self, index):
        """ This function implements the compensation & calibration coefficient calculations from page 15 of AN3785. """
        # get raw calibration data from a specified location
        cd = self.getCalibrationData(index) + [0, 0, 0, 0] 
        cc = {"a0":0, "b1":0, "b2":0, "c12":0, "c11":0, "c22":0}
        # cell not alive
        if max(cd) != 0:
            # undo Two's complement if applicable, pack into proper bit width
            cc["a0"] = _unTwos(((cd[0] << 8) | cd[1]), 16)
            cc["b1"] = _unTwos(((cd[2] << 8) | cd[3]), 16)
            cc["b2"] = _unTwos(((cd[4] << 8) | cd[5]), 16)
            cc["c12"] = _unTwos(((cd[6] << 6) | (cd[7] >> 2)), 14)
            cc["c11"] = _unTwos(((cd[8] << 3) | (cd[9] >> 5)), 11)
            cc["c22"] = _unTwos(((cd[10] << 3) | (cd[11] >> 5)), 11)
            # divide by float(1 << (fractionalBits + zeroPad)) to handle weirdness
            cc["a0"] /= float(1 << 3)
            cc["b1"] /= float(1 << 13)
            cc["b2"] /= float(1 << 14)
            cc["c12"] /= float(1 << 22)
            cc["c11"] /= float(1 << 21)
            cc["c22"] /= float(1 << 25)
        return (index, cc)

    def getDataRaw(self):
        """Query the TakkTile USB interface for the pressure and temperature samples from a specified row of sensors.."""
        data = self.dev.read(0x81, 720, 5000)
        try:
            assert len(data) % 4 == 0
            assert len(data)/4 == len(self.alive)
        except:
            raise Exception("data read from USB endpoint is not correct length")
        data = _chunk(data, 4)
        # temperature is contained in the last two bytes of each four byte chunk, pressure in the first two
        # each ten bit number is encoded in two bytes, MSB first, zero padded / left alligned
        temperature = [_unTwos((datum[3] >> 6| datum[2] << 2), 10) for datum in data if datum.count(0) != 4]
        pressure = [_unTwos((datum[1] >> 6| datum[0] << 2), 10) for datum in data if datum.count(0) != 4]
        pressure = list(map(abs, pressure))
        temperature = list(map(abs, temperature))
        # return a dictionary mapping sensor indexes to a tuple containing (pressure, temperature)
        return dict(list(zip(self.alive, list(zip(pressure, temperature)))))

    def getData(self):
        """Return measured pressure in kPa, temperature compensated and factory calibrated."""
        # get raw 10b data
        data = self.getDataRaw()
        #print 'self.getDataRaw():', data
        # helper functions to increase readability
        Padc = lambda cell: data[cell][0]
        Tadc = lambda cell: data[cell][1]
        # initialize array for compensated pressure readings
        Pcomp = {}
        # for element in the returned pressure data...
        for cell in self.alive:
            # load the calibration coefficients calculated when the TakkTile class is initialized
            cc = self.calibrationCoefficients[cell]
            # apply the formula contained on page 13 of Freescale's AN3785
            # "The 10-bit compensated pressure output for MPL115A, Pcomp, is calculated as follows: 
            #  Pcomp = a0 + (b1 + c11*Padc + c12*Tadc) * Padc + (b2 + c22*Tadc) * Tadc"
            Pcomp[cell] = cc["a0"] + (cc["b1"] + cc["c11"]*Padc(cell) + cc["c12"]*Tadc(cell))*Padc(cell) + (cc["b2"] + cc["c22"]*Tadc(cell))*Tadc(cell)
            # convert from 10b number to kPa
            Pcomp[cell] = 65.0/1023.0*Pcomp[cell]+50
            # round to keep sane sigfig count
            Pcomp[cell] = round(Pcomp[cell], 4)
        return Pcomp 
    
    def getCalibrationData(self, index):
        """Request the 12 calibration bytes from a sensor at a specified index."""
        # get the attiny's virtual address for the specified index 
        # read the calibration data via vendor request and return it 
        return list(self.dev.ctrl_transfer(0x40|0x80, 0x6C, index%5, int( index/5 ), 8))

    def startSampling(self):
        return self.dev.ctrl_transfer(0x40|0x80, 0xC7, 100, 0xFF, 1)[0]  

    def stopSampling(self):
        return self.dev.ctrl_transfer(0x40|0x80, 0xC7, 0, 0, 1)[0]  
#%%
if __name__ == "__main__":

    import sys, pprint
    import matplotlib.pyplot as plt
    import numpy as np
        
    plt.ion()
    
    fig = plt.figure("padplot", clear=True)
    ax = fig.add_subplot()
    ax.set_xlim(0, 1250)
    plt.show()
    
    try:
        tact.alive
    except:
        tact = TakkTile()
        
    print("UIDs: ", tact.UIDs)
    print("Alive: ", tact.alive,"\n\n")
    try:
        count = int(sys.argv[1])
    except:
        count = 3000
    import time
    tact.startSampling()
    start = time.time()
        
    meas0 = None
    meas  = None
    bars = None
    
    try:
        for i in range(count):
            
            print ("i=",i)
    #        print(tact.getDataRaw())
            d = tact.getDataRaw()
            
            if i == 0: #meas == None:
                meas = np.empty((len(d), 2))
                meas[:,0] = list(d.keys())
                
            for idx in range( len(meas[:,0]) ):
                meas[idx,1] = d[ meas[idx,0] ][0]
                
            if  i == 0:
                meas0 = meas.copy()
                meas0[:,1] = 0.0
                bars = ax.barh(meas0[:,0], meas0[:,1]*0)    
            else:
                for idx in range( len(meas[:,0]) ):
                    bars.patches[idx].set_width(meas[idx, 1]-meas0[idx, 1])
                
            for i in d:
                print(( "{}: {}".format(i, d[i]) ))
                
            #ax.relim()
            # update ax.viewLim using the new dataLim
            #ax.autoscale_view()
                
            fig.canvas.draw()            
            fig.canvas.flush_events()
            #time.sleep( 0.5 )    
                
            #d_0 = d    
    finally:            
        print("stopping")
        end = time.time()
        tact.stopSampling()
        #tact.dev.reset()
        del tact
        print((end-start)/int(count))
