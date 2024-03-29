// ======================================================================
// USBtiny Configuration
//
// Copyright 2006-2010 Dick Streefland
//
// This is free software, licensed under the terms of the GNU General
// Public License as published by the Free Software Foundation.
// ======================================================================

// The D+ and D- USB signals should be connected to two pins of the same
// I/O port. The following macros define the port letter and the input
// bit numbers:
#define    USBTINY_PORT            D
#define    USBTINY_DPLUS            2
#define    USBTINY_DMINUS            3

// The D+ signal should be connected to an interrupt input to trigger an
// interrupt at the start of a packet. When you use the same pin for the
// D+ USB signal and the interrupt input, only two I/O pins are needed
// for the USB interface. The following macro defines the interrupt
// number:
#define    USBTINY_INT            0

// Optional definition of the I/O pin to control the 1K5 pullup of the
// D- signal. This will force a reconnect after RESET. (+4 bytes)
// WE CANNOT USE THIS PULL-UP TO HAVE ALL PINS FOR KEYBOARD (19 REQUIRED).
//#define    USBTINY_DMINUS_PULLUP        (D,6)

// Optional definition of the I/O pin to drive the "USB OK" LED, that
// will turn on when the host has assigned a device address. (+6 bytes)
// WE CANNOT USE THIS LED TO HAVE ALL PINS FOR KEYBOARD (19 REQUIRED).
//#define    USBTINY_USB_OK_LED        (D,5)

// The power requirement of the USB device in mA, or 0 when the device
// is not bus powered:
#define    USBTINY_MAX_POWER        100

// The USB vendor and device IDs. These values should be unique for
// every distinct device. You can get your own vendor ID from the USB
// Implementers Forum (www.usb.org) if you have a spare $1500 to kill.
// Alternatively, you can buy a small range of device IDs from
// www.voti.nl or www.mecanique.co.uk, or be naughty and use something
// else, like for instance vendor ID 0x6666, which is registered as
// "Prototype product Vendor ID".
// The USBtinyISP project (http://www.ladyada.net/make/usbtinyisp/) has
// allocated an official VID/PID pair for USBtiny. Since these IDs are
// supported by avrdude since version 5.5, we use them here as well:
#define SHARKOON_Technologies_GmbH 0x1ea7
#define    USBTINY_VENDOR_ID        SHARKOON_Technologies_GmbH//0xFFFF
#define Mediatrack_Edge_Mini_Keyboard 0x0066
#define    USBTINY_DEVICE_ID        Mediatrack_Edge_Mini_Keyboard//0x0001

// The version of the device as a 16-bit number: 256*major + minor.
#define    USBTINY_DEVICE_VERSION        0x0200

// The following optional macros may be used as an identification of
// your device. Undefine them when you run out of flash space.
//#define    USBTINY_VENDOR_NAME        "Dick Streefland"
#define    USBTINY_DEVICE_NAME        "Pecak-Kbrd"
#undef    USBTINY_SERIAL

// Define the device class, subclass and protocol. Device class 0xff
// is "vendor specific".
#define    USBTINY_DEVICE_CLASS           0 //0xff
#define    USBTINY_DEVICE_SUBCLASS        0
#define    USBTINY_DEVICE_PROTOCOL        0

#define USB_HID_IFACE_CLASS 0x03
#define USB_HID_IFACE_SUBCLASS_NONE 0x0
#define USB_HID_IFACE_SUBCLASS_BOOT 0x1
#define USB_HID_IFACE_PROT_KBRD 0x1
#define USB_ENDPOINT_ATTR_INTERRUPT 0x3

// Define the interface class, subclass and protocol. Interface class
// 0xff is "vendor specific".
#define    USBTINY_INTERFACE_CLASS       USB_HID_IFACE_CLASS
#define    USBTINY_INTERFACE_SUBCLASS    USB_HID_IFACE_SUBCLASS_BOOT
#define    USBTINY_INTERFACE_PROTOCOL    USB_HID_IFACE_PROT_KBRD

// Normally, usb_setup() should write the reply of up to 8 bytes into the
// packet buffer, and return the reply length. When this macro is defined
// as 1, you have the option of returning 0xff instead. In that case, the
// USB driver will call a function usb_in() to obtain the data to send
// back to the host. This can be used to generate the data on-the-fly.
// When you set this macro to 2, the transfer count is not limited to 255.
#define    USBTINY_CALLBACK_IN        1

// When this macro is defined as 0, OUT packets are simply ignored.
// When defined as 1, the function usb_out() is called for OUT packets.
// You need this option to send data from the host to the device in
// a control transfer.
#define    USBTINY_CALLBACK_OUT        1

// When this macro is defined as 1, an optimized CRC function is used
// that calculates the CRC about twice as fast as the standard version,
// but at the expense of 42 bytes of additional flash memory.
#define    USBTINY_FAST_CRC        1

#define USB_HID_KBRD_POLL_INTERVAL_MS 0x64 // max for now. 0x04 for working kbrd.

// Set the macro USBTINY_ENDPOINT to 1 to add an additional endpoint,
// according to the values of the three other macros.
#define    USBTINY_ENDPOINT        1
#define    USBTINY_ENDPOINT_ADDRESS    0x81    // IN endpoint #1
#define    USBTINY_ENDPOINT_TYPE        USB_ENDPOINT_ATTR_INTERRUPT //0x00    // control transfer type
#define    USBTINY_ENDPOINT_INTERVAL    USB_HID_KBRD_POLL_INTERVAL_MS //0    // ignored

// Set the macro USBTINY_NO_DATA to 1 to avoid using initialized data
// by USBtiny. This will slightly increase the code size, but when the
// data section is empty, the code to initialize the data section will
// be omitted from the startup code.
#define    USBTINY_NO_DATA            1
