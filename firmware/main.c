// (C) 2012 Biorobotics Lab and Nonolith Labs
// (C) 2011, 2012 Ian Daniher (Nonolith Labs) <ian@nonolithlabs.com>
// (C) 2011 Kevin Mehall (Nonolith Labs) <km@kevinmehall.net>
// Licensed under the terms of the GNU GPLv3+

#include "TakkTile.h"

#include <avr/eeprom.h>

#define F_TWI    1000000
#define TWI_BAUD ((F_CPU / (2 * F_TWI)) - 5) 

uint8_t row0data[20];

inline uint8_t calcTinyAddr(uint8_t row, uint8_t column) { return (((row)&0x0F) << 4 | (column&0x07) << 1); }

uint8_t botherAddress(uint8_t address, bool stop){
	TWIC.MASTER.CTRLB |= TWI_MASTER_QCEN_bm;
	// set address to bother
	TWIC.MASTER.ADDR = address;
	// if address ends in one, wait for a read to finish
	if (address & 1) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
	// if address ends in zero, wait for a write to finish
	else while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	if (stop) TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
	// return 1 if ACK, 0 if NACK
	return ((TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm) >> 4)^1; 
}

inline void startConversion(uint8_t row){
	// if sensor 2 is alive on a given row, the row is alive
	if ( (bitmap[row]&(1<<2)) == (1<<2) ){
	// enable all MPL115A2s
		botherAddress(calcTinyAddr(row, 6), 1);
		// write address byte of MPL115A2
		botherAddress(0xC0, 0);
		// write 1 to 0x12 - start conversion of pressure & temperature
		TWIC.MASTER.DATA = 0x12;
		while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
		TWIC.MASTER.DATA = 0x01;
		while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
		// end transaction
		TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
		// disable all MPL115A2s
		botherAddress(calcTinyAddr(row, 6)^1, 1);
	}
}

void getCalData(uint8_t row, uint8_t column, uint8_t *dataOut){
	// iterate through columns of a given row, enabling the cell, clocking out four bytes of information starting at 0x00
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
		// clock out four bytes
		for (uint8_t byteCt = 0; byteCt < 12; byteCt++){
			dataOut[byteCt] = TWIC.MASTER.DATA;
			// if transaction isn't over, wait for ACK
			if (byteCt < 11) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
			// if transaction is almost over, set next byte to NACK
			if (byteCt == 10) TWIC.MASTER.CTRLC |= TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
		}
		botherAddress(tinyAddr^1, 1);
	}
	else TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
}

void getRowData(uint8_t row, uint8_t *dataOut){
	// iterate through columns of a given row, enabling the cell, clocking out four bytes of information starting at 0x00
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
				uint8_t index = byteCt + column*4;
				dataOut[index] = TWIC.MASTER.DATA;
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


void getAlive(void){
	for (uint8_t row = 0; row < 8; row++) {
		uint8_t sensor_bm = 0;
		for (uint8_t column = 0; column < 5; column++) {
			// attiny address formula
			uint8_t tinyAddr = calcTinyAddr(row, column); 
			// if the write address ACKs....
			if (botherAddress(tinyAddr, 1) == 1) {
				// ping the MPL115A2
				if ( botherAddress(0xC0, 1) == 1 ) sensor_bm |= 1 << column;
				// then turn off the sensor with an address LSB of 1
				botherAddress(tinyAddr^1, 1);
			}
		bitmap[row] = sensor_bm; 
		}
	}
}

ISR(TCC0_CCA_vect){
    PORTR.OUTTGL = 1 << 1;
	getRowData(0, row0data);
	startConversion(0);
	TCC0.CNT = 0;
}
int main(void){
	USB_ConfigureClock();
	PORTR.DIRSET = 1 << 1;
	PORTR.OUTSET = 1 << 1;

	TWIC.MASTER.BAUD = TWI_BAUD;
	TWIC.MASTER.CTRLA = TWI_MASTER_ENABLE_bm;  
	TWIC.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;

	// config samples timer
	TCC0.PER = 0;
	TCC0.CNT = 0;
	TCC0.INTCTRLB = TC_CCAINTLVL_LO_gc;
	TCC0.CTRLA = TC_CLKSEL_DIV256_gc;
	TCC0.CTRLB = TC0_CCAEN_bm | TC_WGMODE_SINGLESLOPE_gc;
	TCC0.CCA = 480; 
	getAlive();

	USB_Init();
	PMIC.CTRL = PMIC_LOLVLEN_bm;
	sei();	
	for (;;){
		USB_Evt_Task();
		USB_Task();
	}
}

#define xstringify(s) stringify(s)
#define stringify(s) #s

const char PROGMEM hwversion[] = xstringify(HW_VERSION);
const char PROGMEM fwversion[] = xstringify(FW_VERSION);

uint8_t usb_cmd = 0;
uint8_t cmd_data = 0;

/** Event handler for the library USB Control Request reception event. */
bool EVENT_USB_Device_ControlRequest(USB_Request_Header_t* req){
	// zero out ep0_buf_in
	for (uint8_t i = 0; i < 64; i++) ep0_buf_in[i] = 0;
	usb_cmd = 0;
	if ((req->bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_VENDOR){
		switch(req->bRequest){
			case 0x00: // Info
				if (req->wIndex == 0){
					USB_ep0_send_progmem((uint8_t*)hwversion, sizeof(hwversion));
				}else if (req->wIndex == 1){
					USB_ep0_send_progmem((uint8_t*)fwversion, sizeof(fwversion));
				}
				
				return true;
				
			case 0xBA: // bother a specified I2C address, return '1' if address ACKs, '0' if NACK
				ep0_buf_in[0] = botherAddress(req->wIndex, req->wValue);
				USB_ep0_send(1);
				return true;

			case 0x5C: // return bitmap of alive rows
				for (uint8_t i = 0; i < 8; i++) {ep0_buf_in[i] = bitmap[i];}
				USB_ep0_send(8);
				return true;

			case 0x6C: // return the 12 calibration bytes for a specified address
				getCalData(req->wIndex, req->wValue, ep0_buf_in);
				USB_ep0_send(12);
				return true;

			case 0x7C: // return the 20 bytes of pressure and temperature information from a specified row
				if (TCC0.PER == 0){
					startConversion(0);
					TCC0.PER = 1<<15;
				}
				for (uint8_t i = 0; i < 20; i++) {ep0_buf_in[i] = row0data[i];}
				USB_ep0_send(20);
				return true;

			case 0xE0: // Read EEPROM
				eeprom_read_block(ep0_buf_in, (void*)(req->wIndex*64), 64);
				USB_ep0_send(64);
				return true;
				
			case 0xE1: // Write EEPROM
				usb_cmd = req->bRequest;
				cmd_data = req->wIndex;
				USB_ep0_send(0);
				return true; // Wait for OUT data (expecting an OUT transfer)
				
			case 0xBB: // disconnect from USB, jump to bootloader
				USB_enter_bootloader();
				return true;
		}
	}
	return false;
}

void EVENT_USB_Device_ControlOUT(uint8_t* buf, uint8_t count){
	switch (usb_cmd){
		case 0xE1: // Write EEPROM
			eeprom_update_block(buf, (void*)(cmd_data*64), count);
			break;
	}
}
