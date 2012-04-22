#pragma once
#define F_CPU 32000000UL

// includes
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Descriptors.h"
#include "usb.h"

typedef struct IN_packet{
} __attribute__((packed)) IN_packet;

typedef struct OUT_packet{
} __attribute__((packed)) OUT_packet;

// function prototypes
void configHardware(void);
bool EVENT_USB_Device_ControlRequest(USB_Request_Header_t* req);
