#pragma once
#define F_CPU 32000000UL

// includes
#include "usb/avr/io.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Descriptors.h"
#include "usb/usb.h"

uint8_t bitmap[8];
uint8_t sensorData[256];
uint8_t calibrationData[512];

// function prototypes
void configHardware(void);
void startConversion(void);
void getRowData(uint8_t row, uint8_t *dataOut);
bool EVENT_USB_Device_ControlRequest(USB_Request_Header_t* req);
