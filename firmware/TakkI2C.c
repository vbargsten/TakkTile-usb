// (C) 2012 Biorobotics Lab and Nonolith Labs
// (C) 2011, 2012 Ian Daniher (Nonolith Labs) <ian@nonolithlabs.com>
// (C) 2012 Kevin Mehall (Nonolith Labs) <kevin@nonolithlabs.com>
// Licensed under the terms of the GNU GPLv3+

#include "TakkTile.h"

inline uint8_t calcTinyAddr(uint8_t row, uint8_t column) { return (((row)&0x0F) << 4 | (column&0x07) << 1); }

uint8_t botherAddress(uint8_t address, bool stop){
	// Function to write address byte to I2C, returns 1 if ACK, 0 if NACK.
	// 'stop' specifies an optional stop bit on the transaction.
	// NB: Don't read from a non-existant address or the CPU will hang waiting for ACK

	// quick command mode - RIF/WIF trips on ACK
	TWIC.MASTER.CTRLB |= TWI_MASTER_QCEN_bm;
	// set address to bother
	TWIC.MASTER.ADDR = address;
	// if address ends in one, wait for a read to finish
	if (address & 1) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
	// if address ends in zero, wait for a write to finish
	else while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	// initiate stop condition if (stop)
	if (stop) TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
	// return 1 if ACK, 0 if NACK
	return ((TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm) >> 4)^1; 
}

inline void startConversion(){
	// Initiates the analog-to-digital conversion of pressure and temperature
	// on all MPL115A2 sensors on all attached rows.

	// enable all MPL115A2 by writing to 0x0C
	uint8_t ACK = botherAddress(calcTinyAddr(0, 6), 1);
	// write address byte of MPL115A2
	botherAddress(0xC0, 0);
	// write 0x01 to 0x12 - start conversion of pressure & temperature
	TWIC.MASTER.DATA = 0x12;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x01;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	// end transaction
	TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
	// if you got an ACK on enable, disable all the MPL115A2s
	if (ACK == 1) botherAddress(calcTinyAddr(0, 6)^1, 1);
}

void getCalibrationData(void){
	// Iterate through all rows and all columns. If that cell is alive,
	// read 12 calibration bytes from 0x04 into calibrationData.
	
	for (uint8_t row = 0; row < 8; row++) {
		for (uint8_t column = 0; column < 5; column++) {
			TWIC.MASTER.CTRLC &= ~TWI_MASTER_ACKACT_bm;
			TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm;
			if ( (bitmap[row]&(1<<column)) == (1<<column) ){
				// attiny address formula
				uint8_t tinyAddr = calcTinyAddr(row, column); 
				// enable cell
				botherAddress(tinyAddr, 1);
				// start write to MPL115A2 
				botherAddress(0xC0, 0);
				TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm; 
				// set start address to 0
				TWIC.MASTER.DATA = 0x04;
				while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
				// end transaction
				TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
				// start read from MPL115A2
				TWIC.MASTER.ADDR = 0xC1;
				while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
				for (uint8_t byteCt = 0; byteCt < 8; byteCt++){
					uint8_t index = 40*row+8*column+byteCt;
					calibrationData[index] = TWIC.MASTER.DATA;
					// if transaction isn't over, wait for ACK
					if (byteCt < 7) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
					// if transaction is almost over, set next byte to NACK
					if (byteCt == 6) TWIC.MASTER.CTRLC |= TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
				}
				botherAddress(tinyAddr^1, 1);
			}
			else TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
		}
	}
}

void getSensorData(void){
	/* Iterate through all rows and all columns. If that cell is alive,
	read four data bytes from memory address 0x00 into USB buffer via send_byte */
	uint8_t datum = 0x00;
	for (uint8_t row = 0; row < 8; row++) {
		for (uint8_t column = 0; column < 5; column++) {
			TWIC.MASTER.CTRLC &= ~TWI_MASTER_ACKACT_bm;
			TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm;
			// attiny address formula
			if ( (bitmap[row]&(1<<column)) == (1<<column) ){
				uint8_t tinyAddr = calcTinyAddr(row, column); 
				// enable cell
				botherAddress(tinyAddr, 1);
				// start write to MPL115A2
				botherAddress(0xC0, 0);
				TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm;	 
				// set start address to 0
				TWIC.MASTER.DATA = 0x00;
				while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
				// end transaction
				TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
				// start read from MPL115A2
				TWIC.MASTER.ADDR = 0xC1;
				while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
				// clock out four bytes
				for (uint8_t byteCt = 0; byteCt < 4; byteCt++){
					datum = TWIC.MASTER.DATA;
					send_byte(datum);
					sensorData[(row*5 + column)*4 + byteCt] = datum;
					// if transaction isn't over, wait for ACK
					if (byteCt < 3) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
					// if transaction is almost over, set next byte to NACK
					if (byteCt == 2) TWIC.MASTER.CTRLC |= TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
				}
			botherAddress(tinyAddr^1, 1);
			}
			else TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
		}
	}
	break_and_flush();
}


void getAlive(void){
	/* Iterate through rows and columns, generating a bitmap of alive sensors. */

	for (uint8_t row = 0; row < 8; row++) {
		uint8_t sensor_bm = 0;
		for (uint8_t column = 0; column < 5; column++) {
			// attiny address formula
			uint8_t tinyAddr = calcTinyAddr(row, column); 
			_delay_us(5);
			// if the write address ACKs....
			if (botherAddress(tinyAddr, 1) == 1) {
				_delay_us(5);
				// ping the MPL115A2
				if ( botherAddress(0xC0, 1) == 1 ) sensor_bm |= 1 << column;
				// then turn off the sensor with an address LSB of 1
				_delay_us(5);
				botherAddress(tinyAddr^1, 1);
			}
		bitmap[row] = sensor_bm; 
		}
	}
}
