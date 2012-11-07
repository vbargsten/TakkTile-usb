// (C) 2012 Biorobotics Lab and Nonolith Labs
// (C) 2011, 2012 Ian Daniher (Nonolith Labs) <ian@nonolithlabs.com>
// (C) 2012 Kevin Mehall (Nonolith Labs) <kevin@nonolithlabs.com>
// Licensed under the terms of the GNU GPLv3+

#include "TakkTile.h"
#include "TakkI2C.c"

// run I2C at 1MHz
#define F_TWI	1000000
#define TWI_BAUD ((F_CPU / (2 * F_TWI)) - 5) 

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

			// return calibration information
			// mnemonic - 0x6etCalibration
			case 0x6C: {
				uint8_t offset = 40*req->wIndex+8*req->wValue;
				for (uint8_t i = 0; i < 8; i++) {ep0_buf_in[i] = calibrationData[offset+i];}
				USB_ep0_send(8);
				return true;
				}

			// disconnect from USB, jump to bootloader	
			case 0xBB: 
				USB_enter_bootloader();
				return true;
		}
	}
	return false;
}
