// (C) 2012 Biorobotics Lab and Nonolith Labs
// (C) 2011, 2012 Ian Daniher (Nonolith Labs) <ian@nonolithlabs.com>
// (C) 2012 Kevin Mehall (Nonolith Labs) <kevin@nonolithlabs.com>
// Licensed under the terms of the GNU GPLv3+

#include "TakkTile.h"
#include <avr/eeprom.h>
#include "usb_pipe.h"
#include <avr/io.h>

// run I2C at 1MHz
#define F_TWI	1000000
#define TWI_BAUD ((F_CPU / (2 * F_TWI)) - 5) 

bool timeout_or_sampling_no_longer_enabled = 0;

USB_PIPE(ep_in, 0x81 | USB_EP_PP, USB_EP_TYPE_BULK_gc, 64, 512, 1, 0, PIPE_ENABLE_FLUSH);

// Queue a byte to be sent over the bulk EP. Blocks if the buffer is full
static inline void send_byte(uint8_t byte){
	while (!usb_pipe_can_write(&ep_in, 1)); // This should never actually block if your buffer is big enough
	pipe_write_byte(ep_in.pipe, byte);
	USB.INTFLAGSBSET = USB_TRNIF_bm;
}

// Sends a break to end the USB read and flushes the USB pipe
static inline void break_and_flush(){
	usb_pipe_flush(&ep_in);
	USB.INTFLAGSBSET = USB_TRNIF_bm;
	while (!usb_pipe_can_write(&ep_in, 1)){
		if (timeout_or_sampling_no_longer_enabled){
			usb_pipe_reset(&ep_in);
			return;
		}	
	}
}


// equation for calculating I2C address from row and column
inline uint8_t calcTinyAddr(uint8_t row, uint8_t column) { return (((row)&0x0F) << 4 | (column&0x07) << 1); }

uint8_t botherAddress(uint8_t address, bool stop){
	// Function to write address byte to I2C, returns 1 if ACK, 0 if NACK.
	// 'stop' specifies an optional stop bit on the transaction.
	// NB: Don't read from a non-existant address

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
	
	// This could also easily be made to dump data over the bulk pipe with send_byte()
	// on the host you would send the vReq, then do a bulk read for more than the max size, that would end with and synchronize on the break

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
	// break_and_flush()
}

void getSensorData(void){
	/* Iterate through all rows and all columns. If that cell is alive,
	read four data bytes from 0x00 into sensorData */

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
					send_byte(TWIC.MASTER.DATA);
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
	// Timer interrupt that trips 1ms after TCC0.CNT is set to 0.
	// Change the LED state, clock out all data from all alive sensors, and start next conversion

	PORTR.OUTTGL = 1 << 1;
	getSensorData();

	startConversion();
	TCC0.CNT = 0;
}

int main(void){
	USB_ConfigureClock();
	PORTR.DIRSET = 1 << 1;
	USB_Init();
	
	// Enable USB interrupts
	USB.INTCTRLA = USB_BUSEVIE_bm | USB_INTLVL_MED_gc;
	USB.INTCTRLB = USB_TRNIE_bm | USB_SETUPIE_bm;

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
	sei(); 

	TWIC.MASTER.BAUD = TWI_BAUD;
	TWIC.MASTER.CTRLA = TWI_MASTER_ENABLE_bm;  
	TWIC.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;

	TCC0.CTRLA = TC_CLKSEL_DIV256_gc;
	TCC0.CTRLB = TC0_CCAEN_bm | TC_WGMODE_SINGLESLOPE_gc;
	TCC0.INTCTRLB = TC_CCAINTLVL_LO_gc;
	TCC0.CCA = 120; 
	TCC0.PER = 0;

	getAlive();
	getCalibrationData();

	for (;;){}
}

void EVENT_USB_Device_ConfigurationChanged(uint8_t config){
	PORTR.OUTSET = 1 << 1;	
	usb_pipe_init(&ep_in);
}

ISR(USB_BUSEVENT_vect){
	if (USB.INTFLAGSACLR & USB_SOFIF_bm){
		USB.INTFLAGSACLR = USB_SOFIF_bm;
	}else if (USB.INTFLAGSACLR & (USB_CRCIF_bm | USB_UNFIF_bm | USB_OVFIF_bm)){
		USB.INTFLAGSACLR = (USB_CRCIF_bm | USB_UNFIF_bm | USB_OVFIF_bm);
	}else if (USB.INTFLAGSACLR & USB_STALLIF_bm){
		USB.INTFLAGSACLR = USB_STALLIF_bm;
	}else{
		USB.INTFLAGSACLR = USB_SUSPENDIF_bm | USB_RESUMEIF_bm | USB_RSTIF_bm;
		USB_Evt_Task();
	}
}

ISR(USB_TRNCOMPL_vect){
	USB.FIFOWP = 0;
	USB.INTFLAGSBCLR = USB_SETUPIF_bm | USB_TRNIF_bm;
	usb_pipe_handle(&ep_in);
	USB_Task();
}


#define xstringify(s) stringify(s)
#define stringify(s) #s

const char PROGMEM hwversion[] = xstringify(HW_VERSION);
const char PROGMEM fwversion[] = xstringify(FW_VERSION);

uint8_t usb_cmd = 0;
uint8_t cmd_data = 0;

// Add a flag to prevent other i2c-using requests from happening while sampling is enabled. ControlRequest is now from an ISR, and could happen at any time inside of GetData and friends.

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

			// bother a specified I2C address, return '1' if address ACKs, '0' if NACK
			// mnemonic - 0xBotherAddress
			case 0xBA: 
				ep0_buf_in[0] = botherAddress(req->wIndex, req->wValue);
				USB_ep0_send(1);
				return true;

			// start sampling
			// mnemonic - 0xConfigure7imer
			case 0xC7:
				if (req->wIndex != 0) {
					TCC0.PER = 1 << 15;
					startConversion();
					ep0_buf_in[0] = 1;
					usb_pipe_reset(&ep_in);
					timeout_or_sampling_no_longer_enabled = 0;
					TCC0.CCA = req->wValue;}
				else {
					TCC0.PER = 0;
					ep0_buf_in[0] = 0;
					usb_pipe_reset(&ep_in);
					timeout_or_sampling_no_longer_enabled = 1;
				}
				TCC0.CNT = 0;
				USB_ep0_send(1);
				return true;

			// return a bitmap of alive cells 
			// mnemonic - 0x5Can
			case 0x5C: 
				for (uint8_t i = 0; i < 8; i++) {ep0_buf_in[i] = bitmap[i];}
				USB_ep0_send(8);
				return true;

			case 0x6C: {
				uint8_t offset = 40*req->wIndex+8*req->wValue;
				for (uint8_t i = 0; i < 8; i++) {ep0_buf_in[i] = calibrationData[offset+i];}
				USB_ep0_send(8);
				return true;
				}

			// read EEPROM	
			case 0xE0: 
				eeprom_read_block(ep0_buf_in, (void*)(req->wIndex*64), 64);
				USB_ep0_send(64);
				return true;

			// write EEPROM	
			case 0xE1: 
				usb_cmd = req->bRequest;
				cmd_data = req->wIndex;
				USB_ep0_send(0);
				return true; // Wait for OUT data (expecting an OUT transfer)

			// disconnect from USB, jump to bootloader	
			case 0xBB: 
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

