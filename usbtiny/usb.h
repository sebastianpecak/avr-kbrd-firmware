// ======================================================================
// Public interface of the USB driver
//
// Copyright 2006-2008 Dick Streefland
//
// This is free software, licensed under the terms of the GNU General
// Public License as published by the Free Software Foundation.
// ======================================================================

#ifndef USB_H
#define    USB_H

#include <avr/pgmspace.h>


//    Standard requests
#define USB_REQ_GET_STATUS            0
#define USB_REQ_CLEAR_FEATURE        1
#define USB_REQ_SET_FEATURE            3
#define USB_REQ_SET_ADDRESS            5
#define USB_REQ_GET_DESCRIPTOR        6
#define USB_REQ_SET_DESCRIPTOR        7
#define USB_REQ_GET_CONFIGURATION    8
#define USB_REQ_SET_CONFIGURATION    9
#define USB_REQ_GET_INTERFACE        10
#define USB_REQ_SET_INTERFACE        11

// bmRequestType
#define REQUEST_HOSTTODEVICE    0x00
#define REQUEST_DEVICETOHOST    0x80
#define REQUEST_DIRECTION        0x80

#define USB_REQ_STANDARD        0x00
#define REQUEST_CLASS            0x20
#define REQUEST_VENDOR            0x40
#define USB_REQ_TYPE            0x60
#define USB_REQ_DESC_MASK 0x1F
#define USB_REQ_RECIPIENT_MASK 0x1F
#define USB_REQ_RECIPIENT_IFACE 0x1

#define REQUEST_DEVICE            0x00
#define REQUEST_INTERFACE        0x01
#define REQUEST_ENDPOINT        0x02
#define REQUEST_OTHER            0x03
#define REQUEST_RECIPIENT        0x03

#define REQUEST_DEVICETOHOST_CLASS_INTERFACE    (REQUEST_DEVICETOHOST | REQUEST_CLASS | REQUEST_INTERFACE)
#define REQUEST_HOSTTODEVICE_CLASS_INTERFACE    (REQUEST_HOSTTODEVICE | REQUEST_CLASS | REQUEST_INTERFACE)
#define REQUEST_DEVICETOHOST_STANDARD_INTERFACE (REQUEST_DEVICETOHOST | REQUEST_STANDARD | REQUEST_INTERFACE)

#define USB_DEVICE_DESCRIPTOR_TYPE             1
#define USB_CONFIGURATION_DESCRIPTOR_TYPE      2
#define USB_STRING_DESCRIPTOR_TYPE             3
#define USB_INTERFACE_DESCRIPTOR_TYPE          4
#define USB_ENDPOINT_DESCRIPTOR_TYPE           5

#define USB_DESC_STR_ID_SUPPORTED_LANG 0


#define USB_HID_DESCRIPTOR_SIZE 9
#define USB_HID_DESCRIPTOR_TYPE 0x21
#define USB_HID_REPORT_DESC_TYPE 0x22


typedef    unsigned char    byte_t;
typedef    unsigned int    uint_t;

// usb.c
extern    void        usb_init ( void );
extern    void        usb_poll ( void );

// crc.S
extern    void        crc ( byte_t* data, byte_t len );

// application callback functions
extern    byte_t        usb_setup ( byte_t data[8] );
extern void usb_setup_new(byte_t **data, byte_t *len);
extern    void        usb_out ( byte_t* data, byte_t len );
extern    byte_t        usb_in ( byte_t* data, byte_t len );

//extern const uint8_t hid_report_desc[47] PROGMEM;
//extern const byte_t hid_desc[9] PROGMEM;

enum
{
    TX_STATE_IDLE = 0,        // transmitter idle
    TX_STATE_RAM,            // usb_tx_data is a RAM address
    TX_STATE_ROM,            // usb_tx_data is a ROM address
    TX_STATE_CALLBACK,        // call usb_in() to obtain transmit data
};

#if    USBTINY_CALLBACK_IN == 2
typedef    uint_t        txlen_t;
#else
typedef    byte_t        txlen_t;
#endif

/*static*/  extern  byte_t    usb_tx_state;        // TX_STATE_*, see enum above
/*static*/  extern  txlen_t    usb_tx_total;        // total transmit size
/*static*/  extern  byte_t*    usb_tx_data;        // pointer to data to transmit

#endif    // USB_H
