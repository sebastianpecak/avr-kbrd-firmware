// ======================================================================
// USB driver
//
// Entry points:
//     usb_init()    - enable the USB interrupt
//     usb_poll()    - poll for incoming packets and process them
//
// This code communicates with the interrupt handler through a number of
// global variables, including two input buffers and one output buffer.
// Packets are queued for transmission by copying them into the output
// buffer. The interrupt handler will transmit such a packet on the
// reception of an IN packet.
//
// Standard SETUP packets are handled here. Non-standard SETUP packets
// are forwarded to the application code by calling usb_setup(). The
// macros USBTINY_CALLBACK_IN and USBTINY_CALLBACK_OUT control whether
// the callback functions usb_in() and usb_out() will be called for IN
// and OUT transfers.
//
// Maximum stack usage (gcc 4.1.0 & 4.3.4) of usb_poll(): 5 bytes plus
// possible additional stack usage in usb_setup(), usb_in() or usb_out().
//
// Copyright 2006-2010 Dick Streefland
//
// This is free software, licensed under the terms of the GNU General
// Public License as published by the Free Software Foundation.
// ======================================================================

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "def.h"
#include "usb.h"

#define    LE(word)            (word) & 0xff, (word) >> 8

// ----------------------------------------------------------------------
// typedefs
// ----------------------------------------------------------------------



// ----------------------------------------------------------------------
// USB constants
// ----------------------------------------------------------------------

enum
{
    DESCRIPTOR_TYPE_DEVICE = 1,
    DESCRIPTOR_TYPE_CONFIGURATION,
    DESCRIPTOR_TYPE_STRING,
    DESCRIPTOR_TYPE_INTERFACE,
    DESCRIPTOR_TYPE_ENDPOINT,
};

// ----------------------------------------------------------------------
// Interrupt handler interface
// ----------------------------------------------------------------------

#if    USBTINY_NO_DATA
byte_t    tx_ack;                // ACK packet
byte_t    tx_nak;                // NAK packet
#else
byte_t    tx_ack = USB_PID_ACK;        // ACK packet
byte_t    tx_nak = USB_PID_NAK;        // NAK packet
#endif

byte_t    usb_rx_buf[2*USB_BUFSIZE];    // two input buffers
byte_t    usb_rx_off;            // buffer offset: 0 or USB_BUFSIZE
byte_t    usb_rx_len;            // buffer size, 0 means empty
byte_t    usb_rx_token;            // PID of token packet: SETUP or OUT

byte_t    usb_tx_buf[USB_BUFSIZE];    // output buffer
byte_t    usb_tx_len;            // output buffer size, 0 means empty

byte_t    usb_address;            // assigned device address
byte_t    usb_new_address;        // new device address

// ----------------------------------------------------------------------
// Local data
// ----------------------------------------------------------------------

/*static*/    byte_t    usb_tx_state;        // TX_STATE_*, see enum above
/*static*/    txlen_t    usb_tx_total;        // total transmit size
/*static*/    byte_t*    usb_tx_data;        // pointer to data to transmit

#if    defined USBTINY_VENDOR_NAME
struct
{
    byte_t    length;
    byte_t    type;
    int    string[sizeof(USBTINY_VENDOR_NAME)-1];
}    const    string_vendor PROGMEM =
{
    2 * sizeof(USBTINY_VENDOR_NAME),
    DESCRIPTOR_TYPE_STRING,
    { CAT2(L, USBTINY_VENDOR_NAME) }
};
#  define    VENDOR_NAME_ID    1
#else
#  define    VENDOR_NAME_ID    0
#endif

#if    defined USBTINY_DEVICE_NAME
struct
{
    byte_t    length;
    byte_t    type;
    int    string[sizeof(USBTINY_DEVICE_NAME)-1];
}    const    string_device PROGMEM =
{
    2 * sizeof(USBTINY_DEVICE_NAME),
    DESCRIPTOR_TYPE_STRING,
    { CAT2(L, USBTINY_DEVICE_NAME) }
};
#  define    DEVICE_NAME_ID    2
#else
#  define    DEVICE_NAME_ID    0
#endif

#if    defined USBTINY_SERIAL
struct
{
    byte_t    length;
    byte_t    type;
    int    string[sizeof(USBTINY_SERIAL)-1];
}    const    string_serial PROGMEM =
{
    2 * sizeof(USBTINY_SERIAL),
    DESCRIPTOR_TYPE_STRING,
    { CAT2(L, USBTINY_SERIAL) }
};
#  define    SERIAL_ID    3
#else
#  define    SERIAL_ID    0
#endif

#if    VENDOR_NAME_ID || DEVICE_NAME_ID || SERIAL_ID
static    byte_t    const    string_langid [] PROGMEM =
{
    4,                // bLength
    DESCRIPTOR_TYPE_STRING,        // bDescriptorType (string)
    LE(0x0409),            // wLANGID[0] (American English)
};
#endif

// Device Descriptor
static    byte_t    const    descr_device [18] PROGMEM =
{
    18,                // bLength
    DESCRIPTOR_TYPE_DEVICE,        // bDescriptorType
    LE(0x0110),            // bcdUSB
    USBTINY_DEVICE_CLASS,        // bDeviceClass
    USBTINY_DEVICE_SUBCLASS,    // bDeviceSubClass
    USBTINY_DEVICE_PROTOCOL,    // bDeviceProtocol
    8,                // bMaxPacketSize0
    LE(USBTINY_VENDOR_ID),        // idVendor
    LE(USBTINY_DEVICE_ID),        // idProduct
    LE(USBTINY_DEVICE_VERSION),    // bcdDevice
    VENDOR_NAME_ID,            // iManufacturer
    DEVICE_NAME_ID,            // iProduct
    SERIAL_ID,            // iSerialNumber
    1,                // bNumConfigurations
};

// union USB_EndpointAddress
// {
//     uint8_t address;
//     struct
//     {
//         uint8_t number    : 4;
//         uint8_t reserved  : 3; // Should be zero.
//         uint8_t direction : 1; //
//     };
// };

// Bit 0..3 The endpoint number
// Bit 4..6 Reserved, reset to zero
// Bit 7 Direction, ignored for
// Control endpoints:
// 0 - OUT endpoint
// 1 - IN endpoint

// this is 47 bytes long.
// working kbrd has 67 bytes long, but the same format...
// const uint8_t hid_report_desc[] PROGMEM = {

//     //  Keyboard
//     0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)  // 47
//     0x09, 0x06,                    // USAGE (Keyboard)
//     0xa1, 0x01,                    // COLLECTION (Application)
//     0x85, 0x02,                    //   REPORT_ID (2)
//     0x05, 0x07,                    //   USAGE_PAGE (Keyboard)

//     0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
//     0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
//     0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//     0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
//     0x75, 0x01,                    //   REPORT_SIZE (1)

//     0x95, 0x08,                    //   REPORT_COUNT (8)
//     0x81, 0x02,                    //   INPUT (Data,Var,Abs)
//     0x95, 0x01,                    //   REPORT_COUNT (1)
//     0x75, 0x08,                    //   REPORT_SIZE (8)
//     0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)

//     0x95, 0x06,                    //   REPORT_COUNT (6)
//     0x75, 0x08,                    //   REPORT_SIZE (8)
//     0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//     0x25, 0x73,                    //   LOGICAL_MAXIMUM (115)
//     0x05, 0x07,                    //   USAGE_PAGE (Keyboard)

//     0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
//     0x29, 0x73,                    //   USAGE_MAXIMUM (Keyboard Application)
//     0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
//     0xc0,                          // END_COLLECTION
// };

const uint8_t hid_report_desc_new[] PROGMEM = {
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x15, 0x00, 0x25, 0x01,
    0x95, 0x05, 0x75, 0x01, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x05, 0x07, 0x1a, 0xe0,
    0x00, 0x2a, 0xe7, 0x00, 0x15, 0x00, 0x25, 0x01, 0x95, 0x08, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x19, 0x00, 0x2a, 0xff, 0x00, 0x15, 0x00, 0x26, 0xff, 0x00, 0x95, 0x06,
    0x81, 0x00, 0xc0
};

// the same as in descr_config.
// TODO: check if it is put to program memory.
// const byte_t hid_desc[] PROGMEM =
// {
//     // HID DESCRIPTOR.
//     9, // bLength
//     USB_HID_DESCRIPTOR_TYPE, // bDescriptorType
//     0x01, 0x01, // bcdHID
//     0, // bCountryCode
//     1, // bNumDescriptors
//     USB_HID_REPORT_DESC_TYPE, // bDescriptorType
//     LE((uint16_t)sizeof(hid_report_desc_new)), // wDescriptorLength
// };

// Configuration Descriptor
static    byte_t    const    descr_config [] PROGMEM =
{
    9,                // bLength
    DESCRIPTOR_TYPE_CONFIGURATION,    // bDescriptorType
    LE(9 + 9 + (7 * USBTINY_ENDPOINT) + USB_HID_DESCRIPTOR_SIZE),    // wTotalLength
    1,                // bNumInterfaces
    1,                // bConfigurationValue
    0,                // iConfiguration
    0xa0,//(USBTINY_MAX_POWER ? 0x80 : 0xc0), // bmAttributes
    250,//(USBTINY_MAX_POWER + 1) / 2,    // MaxPower

    // Standard Interface Descriptor
    9,                // bLength
    DESCRIPTOR_TYPE_INTERFACE,    // bDescriptorType
    0,                // bInterfaceNumber
    0,                // bAlternateSetting
    USBTINY_ENDPOINT,        // bNumEndpoints
    USBTINY_INTERFACE_CLASS,    // bInterfaceClass
    USBTINY_INTERFACE_SUBCLASS,    // bInterfaceSubClass
    USBTINY_INTERFACE_PROTOCOL,    // bInterfaceProtocol
    0,                // iInterface

    // HID DESCRIPTOR.
    9, // bLength
    USB_HID_DESCRIPTOR_TYPE, // bDescriptorType
    LE(0x0110), // bcdHID
    0, // bCountryCode
    1, // bNumDescriptors
    USB_HID_REPORT_DESC_TYPE, // bDescriptorType
    LE((uint16_t)sizeof(hid_report_desc_new)), // wDescriptorLength

#if    USBTINY_ENDPOINT
    // Additional Endpoint
    7,                // bLength
    DESCRIPTOR_TYPE_ENDPOINT,    // bDescriptorType
    USBTINY_ENDPOINT_ADDRESS,    // bEndpointAddress
    USBTINY_ENDPOINT_TYPE,        // bmAttributes
    LE(8),                // wMaxPacketSize
    USBTINY_ENDPOINT_INTERVAL,    // bInterval
#endif
};

typedef struct USB_SetupReq
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint8_t wValueL;
    uint8_t wValueH;
    uint16_t wIndex;
    uint16_t wLength;
} USB_SetupReq_t;

// 0 = Standard
// 1 = Class
// 2 = Vendor
// 3 = Reserved
#define USB_REQ_GET_TYPE()

// ----------------------------------------------------------------------
// Inspect an incoming packet.
// ----------------------------------------------------------------------
static    void    usb_receive ( byte_t* data, byte_t rx_len )
{
    byte_t    len;
    byte_t    type;
    txlen_t    limit;

    usb_tx_state = TX_STATE_RAM;
    len = 0;
    limit = 0;

    if (USB_PID_SETUP == usb_rx_token)// == USB_PID_SETUP )
    {
#if    USBTINY_CALLBACK_IN == 2
        limit = * (uint_t*) & data[6];
#else
        limit = data[6];
        if    ( data[7] )
        {
            limit = 255;
        }
#endif
        type = data[0] & USB_REQ_TYPE;//0x60;
        //if    ( type == 0x00 )
        if (USB_REQ_STANDARD == type)
        {    // Standard request
            if (USB_REQ_GET_STATUS == data[1])// == 0 )    // GET_STATUS
            {
                len = 2;
#if    USBTINY_MAX_POWER == 0
                data[0] = (data[0] == 0x80);
#else
                data[0] = 0;
#endif
                data[1] = 0;
            }
            else if (USB_REQ_SET_ADDRESS == data[1])// == 5 )    // SET_ADDRESS
            {
                usb_new_address = data[2];
#ifdef    USBTINY_USB_OK_LED
                SET(USBTINY_USB_OK_LED);// LED on
#endif
            }
            else if (USB_REQ_GET_DESCRIPTOR == data[1])// == 6 )    // GET_DESCRIPTOR
            {
                usb_tx_state = TX_STATE_ROM;
                if (USB_DEVICE_DESCRIPTOR_TYPE == data[3])// == 1 )
                {    // DEVICE
                    data = (byte_t*) &descr_device;
                    len = sizeof(descr_device);
                }
                else if (USB_HID_REPORT_DESC_TYPE == data[3])
                {
                    //PORTB |= _BV(1);
                    data = (byte_t*) &hid_report_desc_new;
                    len = sizeof(hid_report_desc_new);
                }
                else if (USB_CONFIGURATION_DESCRIPTOR_TYPE == data[3])// == 2 )
                {    // CONFIGURATION
                    data = (byte_t*) &descr_config;
                    len = sizeof(descr_config);
                }
#if    VENDOR_NAME_ID || DEVICE_NAME_ID || SERIAL_ID
                else if (USB_STRING_DESCRIPTOR_TYPE == data[3])// == 3 )
                {    // STRING
                    if (USB_DESC_STR_ID_SUPPORTED_LANG == data[2])// == 0 )
                    {
                        data = (byte_t*) &string_langid;
                        len = sizeof(string_langid);
                    }
#if    VENDOR_NAME_ID
                    else if (VENDOR_NAME_ID == data[2])// == VENDOR_NAME_ID )
                    {
                        data = (byte_t*) &string_vendor;
                        len = sizeof(string_vendor);
                    }
#endif
#if    DEVICE_NAME_ID
                    else if (DEVICE_NAME_ID == data[2])// == DEVICE_NAME_ID )
                    {
                        data = (byte_t*) &string_device;
                        len = sizeof(string_device);
                    }
#endif
#if    SERIAL_ID
                    else if (SERIAL_ID == data[2])// == SERIAL_ID )
                    {
                        data = (byte_t*) &string_serial;
                        len = sizeof(string_serial);
                    }
#endif
                }
#endif
            }
            else if (USB_REQ_GET_CONFIGURATION == data[1])// == 8 )    // GET_CONFIGURATION
            {
                data[0] = 1;        // return bConfigurationValue
                len = 1;
            }
            else if (USB_REQ_GET_INTERFACE == data[1])// == 10 )    // GET_INTERFACE
            {
                data[0] = 0;
                len = 1;
            }
            // Standard interface request.
            // else if (USB_REQ_RECIPIENT_IFACE == (data[0] & USB_REQ_RECIPIENT_MASK))
            // {

            // }
        }
        // else if (REQUEST_CLASS == type)
        // {
        //     if (USB_REQ_GET_DESCRIPTOR == data[1])
        //     {
        //         if (USB_HID_REPORT_DESC_TYPE == data[3])
        //         {
        //             PORTB |= _BV(1);
        //             data = (byte_t*) &hid_report_desc;
        //             len = sizeof(hid_report_desc);
        //         }
        //     }
        // }
        else
        {    // Class or Vendor request
            //usb_setup_new(&data, &len);
            //usb_tx_state = TX_STATE_ROM;
            //len = usb_setup( data );
#if    USBTINY_CALLBACK_IN
            if    ( len == 0xff )
            {
                usb_tx_state = TX_STATE_CALLBACK;
            }
#endif
        }
        if    (  len < limit
#if    USBTINY_CALLBACK_IN == 2
            && len != 0xff
#endif
            )
        {
            limit = len;
        }
        usb_tx_data = data;
    }
    // else if (USB_PID_IN == usb_rx_token)
    // {
    //     usb_tx_data = (byte_t *)&report;
    //     limit = sizeof(report);
    // }
#if    USBTINY_CALLBACK_OUT
    else if    ( rx_len > 0 )
    {    // usb_rx_token == USB_PID_OUT
        usb_out( data, rx_len );
    }
#endif
    usb_tx_total  = limit;
    usb_tx_buf[0] = USB_PID_DATA0;    // next data packet will be DATA1
}

// ----------------------------------------------------------------------
// Load the transmit buffer with the next packet.
// ----------------------------------------------------------------------
static    void    usb_transmit ( void )
{
    byte_t    len;
    byte_t*    src;
    byte_t*    dst;
    byte_t    i;
    byte_t    b;

    usb_tx_buf[0] ^= (USB_PID_DATA0 ^ USB_PID_DATA1);
    if    ( usb_tx_total > 8 )
    {
        len = 8;
    }
    else
    {
        len = (byte_t) usb_tx_total;
    }
    dst = usb_tx_buf + 1;
    if    ( len > 0 )
    {
#if    USBTINY_CALLBACK_IN
        if    ( usb_tx_state == TX_STATE_CALLBACK )
        {
            len = usb_in( dst, len );
        }
        else
#endif
        {
            src = usb_tx_data;
            if    ( usb_tx_state == TX_STATE_RAM )
            {
                for    ( i = 0; i < len; i++ )
                {
                    *dst++ = *src++;
                }
            }
            else    // usb_tx_state == TX_STATE_ROM
            {
                for    ( i = 0; i < len; i++ )
                {
                    b = pgm_read_byte( src );
                    src++;
                    *dst++ = b;
                }
            }
            usb_tx_data = src;
        }
        usb_tx_total -= len;
    }
    crc( usb_tx_buf + 1, len );
    usb_tx_len = len + 3;
    if    ( len < 8 )
    {    // this is the last packet
        usb_tx_state = TX_STATE_IDLE;
    }
}

// ----------------------------------------------------------------------
// Initialize the low-level USB driver.
// ----------------------------------------------------------------------
extern    void    usb_init ( void )
{
    USB_INT_CONFIG |= USB_INT_CONFIG_SET;
    USB_INT_ENABLE |= (1 << USB_INT_ENABLE_BIT);
#ifdef    USBTINY_USB_OK_LED
    OUTPUT(USBTINY_USB_OK_LED);
#endif
#ifdef    USBTINY_DMINUS_PULLUP
    SET(USBTINY_DMINUS_PULLUP);
    OUTPUT(USBTINY_DMINUS_PULLUP);    // enable pullup on D-
#endif
#if    USBTINY_NO_DATA
    tx_ack = USB_PID_ACK;
    tx_nak = USB_PID_NAK;
#endif
    sei();
}

// ----------------------------------------------------------------------
// Poll USB driver:
// - check for incoming USB packets
// - refill an empty transmit buffer
// - check for USB bus reset
// ----------------------------------------------------------------------
extern    void    usb_poll ( void )
{
    byte_t    i;

    // check for incoming USB packets
    if    ( usb_rx_len != 0 )
    {
        usb_receive( usb_rx_buf + USB_BUFSIZE - usb_rx_off + 1, usb_rx_len - 3 );
        usb_tx_len = 0;    // abort pending transmission
        usb_rx_len = 0;    // accept next packet
    }
    // refill an empty transmit buffer, when the transmitter is active
    if    ( usb_tx_len == 0 && usb_tx_state != TX_STATE_IDLE )
    {
        usb_transmit();
    }
    // check for USB bus reset
    for    ( i = 10; i > 0 && ! (USB_IN & USB_MASK_DMINUS); i-- )
    {
    }
    if    ( i == 0 )
    {    // SE0 for more than 2.5uS is a reset
        usb_new_address = 0;
        usb_address = 0;
#ifdef    USBTINY_USB_OK_LED
        CLR(USBTINY_USB_OK_LED);    // LED off
#endif
    }
}
