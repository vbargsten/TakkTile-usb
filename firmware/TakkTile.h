#pragma once
#define F_CPU 32000000UL

// includes
#include "usb/avr/io.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Descriptors.h"
#include "usb/usb.h"

uint8_t bitmap[8];

// function prototypes
void configHardware(void);
bool EVENT_USB_Device_ControlRequest(USB_Request_Header_t* req);
