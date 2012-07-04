// (C) 2012 Biorobotics Lab and Nonolith Labs                                                                                                                                                    
// (C) 2011, 2012 Ian Daniher (Nonolith Labs) <ian@nonolithlabs.com>
// (C) 2011 Kevin Mehall (Nonolith Labs) <km@kevinmehall.net>
// Licensed under the terms of the GNU GPLv3+

#include "takktile.h"

#include <avr/eeprom.h>

#define F_TWI    400000
#define TWI_BAUD ((F_CPU / (2 * F_TWI)) - 5) 

int main(void){
	configHardware();
	
	PMIC.CTRL = PMIC_LOLVLEN_bm;
	sei();	
	for (;;){
		USB_Task(); // lower-priority USB polling, like control requests
	}
}

uint8_t botherAddress(uint8_t address, bool stop){
	TWIC.MASTER.CTRLB |= TWI_MASTER_QCEN_bm;
	// set address to bother
	TWIC.MASTER.ADDR = address;
	// if address ends in one, wait for a read to finish
	if (address & 1) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
	// if address ends in zero, wait for a write to finish
	else while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	if (stop == 1) TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
	// return 1 if NACK, 0 if ACK
	return TWIC.MASTER.STATUS&TWI_MASTER_RXACK_bm;
}

uint8_t scanRow(uint8_t row){
	uint8_t sensor_bm = 0;
	for (uint8_t column = 0; column < 5; column++) {
		// attiny address formula
		uint8_t tinyAddr = ((row&0x0F) << 4 | (column&0x07) << 1);
		// if the write address ACKs....
		if (botherAddress(tinyAddr, 1) == 0) {
			// ping the MPL115A2
			if ( botherAddress(0xC0, 1) == 0 ) sensor_bm |= 1 << column;
			// then turn off the sensor with an address LSB of 1
			botherAddress(tinyAddr^1, 1);
		}
	}
	return sensor_bm;
}


void getCalibrationBytes(uint8_t tinyAddr, uint8_t *dataOut){
	// if attiny ACKs
	if (botherAddress(tinyAddr, 1) == 0){
		// set TWI to Smart Mode, helpful for the upcoming Read transaction
		TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm;
		// write 0x04 to MPL115A2 sensor to set start read addy
		TWIC.MASTER.ADDR = 0xC0;
		while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
		TWIC.MASTER.DATA = 0x04;
		while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
		TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
		if ( botherAddress(0xC0, 1) == 0 ){
			// setup read
			TWIC.MASTER.ADDR = 0xC1;
			while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
			// clock in 12 bytes
			TWIC.MASTER.CTRLC &= ~TWI_MASTER_ACKACT_bm;
			for (uint8_t byteCt = 0; byteCt < 12; byteCt++){
				dataOut[byteCt] = TWIC.MASTER.DATA;
				// if byteCt < 11, wait for RIF to trip
				if (byteCt < 11) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
				// if byteCt == 10, setup read to end w/ NACK and STOP
				if (byteCt == 10) TWIC.MASTER.CTRLC |= TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
			}
		}
		// disable MPL115A2
		botherAddress(tinyAddr^1, 1);
	}
}

void getRowData(uint8_t row, uint8_t *dataOut){
	// enable all MPL115A2s
	botherAddress(0x1C, 1);
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
	botherAddress(0x1C^1, 1);
	// wait 1ms for conversion to finish
	_delay_ms(1);
	// iterate through columns of a given row, enabling the cell, clocking out four bytes of information starting at 0x00
	for (uint8_t column = 0; column < 5; column++) {
		TWIC.MASTER.CTRLC &= ~TWI_MASTER_ACKACT_bm;
		TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm;
		// attiny address formula
		uint8_t tinyAddr = ((row&0x0F) << 4 | (column&0x07) << 1);
		// enable cell
		botherAddress(tinyAddr, 1);
		// if MPL115A2 ACKs...
		if ( botherAddress(0xC0, 0) == 0 ){
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
		}
		else TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
		TWIC.MASTER.CTRLB = TWI_MASTER_QCEN_bm;
		botherAddress(tinyAddr^1, 1);
		TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm;
	}
}
	

void configTWI(void){
	// quick command mode trips RIF/WIF as soon as the slave ACKs
	TWIC.MASTER.CTRLB = TWI_MASTER_QCEN_bm; 
	TWIC.MASTER.BAUD = TWI_BAUD;
	TWIC.MASTER.CTRLA = TWI_MASTER_ENABLE_bm;  
	TWIC.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
}

/* Configures the board hardware and chip peripherals for the project's functionality. */
void configHardware(void){
	USB_ConfigureClock();
	configTWI();
	PORTR.DIRSET = 1 << 1;
	PORTR.OUTSET = 1 << 1;
	USB_Init();
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
				for (uint8_t row = 0; row < 8; row++) ep0_buf_in[row] = scanRow(row+1);
				USB_ep0_send(8);
				return true;

			case 0x6C: // return the 12 calibration bytes for a specified address
				getCalibrationBytes(req->wIndex, ep0_buf_in);
				USB_ep0_send(12);
				return true;

			case 0x7C: // return the 20 bytes of pressure and temperature information from a specified row
				getRowData(req->wIndex, ep0_buf_in);
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
				cli();
				PMIC.CTRL = 0;
				USB_ep0_send(0);
				USB_ep0_wait_for_complete();
				_delay_us(10000);
				USB_Detach();
				void (*enter_bootloader)(void) = (void *) 0x47fc /*0x8ff8/2*/;
				enter_bootloader();
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
