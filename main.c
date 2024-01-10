// ======================================================================
// USB AVR programmer and SPI interface
//
// http://www.ladyada.net/make/usbtinyisp/
//
// This code works for both v1.0 and v2.0 devices.
//
// Copyright 2006-2010 Dick Streefland
//
// This is free software, licensed under the terms of the GNU General
// Public License as published by the Free Software Foundation.
// ======================================================================

//#define F_CPU 12000000

#include <avr/io.h>
#include "def.h"
#include "usb.h"
#include <util/delay.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "HID_scan_codes.h"

// ----------------------------------------------------------------------
// I/O pins:
// ----------------------------------------------------------------------
// #define    PORT        PORTB
// #define    DDR        DDRB
// #define    PIN        PINB

// #define    LED        PB0        // output
// #define    RESET        PB4        // output
// #define    MOSI        PB5        // output
// #define    MISO        PB6        // input
// #define    SCK        PB7        // output

// #define    LED_MASK    _BV(LED)
// #define    RESET_MASK    _BV(RESET)
// #define    MOSI_MASK    _BV(MOSI)
// #define    MISO_MASK    _BV(MISO)
// #define    SCK_MASK    _BV(SCK)


// HID 'Driver'
// ------------
#define HID_GET_REPORT        0x01
#define HID_GET_IDLE          0x02
#define HID_GET_PROTOCOL      0x03
#define HID_SET_REPORT        0x09
#define HID_SET_IDLE          0x0A
#define HID_SET_PROTOCOL      0x0B

#define DATA_IN_BY_CALLBACK 0xFF

//uint8_t hidDescIter = 0;
//uint8_t hidReportDescIter = 0;
uint8_t dataInIter = 0;

typedef enum DataInCallback
{
    DATA_IN_HID_DESC,
    DATA_IN_HID_REPORT_DESC,
    DATA_IN_UNKNOWN
} DataInCallback_e;

DataInCallback_e dataInCallback = DATA_IN_UNKNOWN;

// handle HID spefific setup.
byte_t usb_setup_hid(byte_t *data, byte_t len)
{
    const byte_t requestType = data[0] & USB_REQ_TYPE;
    const byte_t descriptor_from_req = data[0] & USB_REQ_DESC_MASK;
    const byte_t request_from_data = data[1];
    const byte_t descriptor_from_data = data[3];

    // Expect CLASS type request for HID.
    if (REQUEST_CLASS == requestType) // class means HID here.
    {
        if (USB_HID_DESCRIPTOR_TYPE == descriptor_from_req)
        {
            dataInIter = 0;
            //hidDescIter = 0;
            dataInCallback = DATA_IN_HID_DESC;
            return DATA_IN_BY_CALLBACK;
        }
        else if (USB_HID_REPORT_DESC_TYPE == descriptor_from_req)
        {
            for (uint8_t i = 0; i < 8; ++i)
            {
                data[i] = 0xff;
            }
            return 8;

            dataInIter = 0;
            //hidReportDescIter = 0;
            dataInCallback = DATA_IN_HID_REPORT_DESC;
            return DATA_IN_BY_CALLBACK;
        }

        if (USB_REQ_GET_DESCRIPTOR == request_from_data)
        {
            if (USB_HID_DESCRIPTOR_TYPE == descriptor_from_data)
            {
                dataInIter = 0;
                //hidDescIter = 0;
                dataInCallback = DATA_IN_HID_DESC;
                return DATA_IN_BY_CALLBACK;
            }
            else if (USB_HID_REPORT_DESC_TYPE == descriptor_from_data)
            {
                for (uint8_t i = 0; i < 8; ++i)
                {
                    data[i] = 0xff;
                }
                return 8;

                dataInIter = 0;
                //hidReportDescIter = 0;
                dataInCallback = DATA_IN_HID_REPORT_DESC;
                return DATA_IN_BY_CALLBACK;
            }
        }
    }
    else if (REQUEST_VENDOR == requestType)
    {
        // not supported.
    }
    else
    {
        // invalid.
    }

    // if (requestType == REQUEST_DEVICETOHOST_CLASS_INTERFACE)
    // {
    //     //The Get_Report request allows the host to receive a report via the Control pipe.
    //     if (request == HID_GET_REPORT) {
    //         // TODO: HID_GetReport();
    //         return 0;
    //     }
    //     if (request == HID_GET_PROTOCOL) {
    //         // TODO: Send8(protocol);
    //         return 0;
    //     }
    //     if (request == HID_GET_IDLE) {
    //         // TODO: Send8(idle);
    //         return 0;
    //     }
    // }

    // if (requestType == REQUEST_HOSTTODEVICE_CLASS_INTERFACE)
    // {
    //     if (request == HID_SET_PROTOCOL) {
    //         // The USB Host tells us if we are in boot or report mode.
    //         // This only works with a real boot compatible device.
    //         //protocol = setup.wValueL;
    //         //return true;
    //         return 0;
    //     }
    //     if (request == HID_SET_IDLE) {
    //         //idle = setup.wValueL;
    //         //return true;
    //         return 0;
    //     }
    //     if (request == HID_SET_REPORT)
    //     {
    //         //uint8_t reportID = setup.wValueL;
    //         //uint16_t length = setup.wLength;
    //         //uint8_t data[length];
    //         // Make sure to not read more data than USB_EP_SIZE.
    //         // You can read multiple times through a loop.
    //         // The first byte (may!) contain the reportID on a multreport.
    //         //USB_RecvControl(data, length);
    //         return 0;
    //     }
    // }

    return 0;
}

void usb_setup_new(byte_t **data, byte_t *len)
{
    // const byte_t requestType = (*data)[0] & USB_REQ_TYPE;
    // const byte_t descriptor_from_req = (*data)[0] & USB_REQ_DESC_MASK;
    // const byte_t request_from_data = (*data)[1];
    // const byte_t descriptor_from_data = (*data)[3];

    // // Expect CLASS type request for HID.
    // if (REQUEST_CLASS == requestType) // class means HID here.
    // {
    //     if (USB_HID_DESCRIPTOR_TYPE == descriptor_from_req)
    //     {
    //     }
    //     else if (USB_HID_REPORT_DESC_TYPE == descriptor_from_req)
    //     {
    //         *data = hid_report_desc;
    //         *len = sizeof(hid_report_desc);
    //     }

    //     if (USB_REQ_GET_DESCRIPTOR == request_from_data)
    //     {
    //         if (USB_HID_DESCRIPTOR_TYPE == descriptor_from_data)
    //         {
    //         }
    //         else if (USB_HID_REPORT_DESC_TYPE == descriptor_from_data)
    //         {
    //             *data = hid_report_desc;
    //             *len = sizeof(hid_report_desc);
    //         }
    //     }
    // }
    // else if (REQUEST_VENDOR == requestType)
    // {
    //     // not supported.
    // }
    // else
    // {
    //     // invalid.
    // }
}

// ----------------------------------------------------------------------
// Handle a non-standard SETUP packet / CLASS OR VENDOR SPECIFIC.
// ----------------------------------------------------------------------
byte_t    usb_setup ( byte_t data[8] )
{
    return usb_setup_hid(data, 8);
    // byte_t    bit;
    // byte_t    mask;
    // byte_t    req;

    // // Generic requests
    // req = data[1];
    // if    ( req == USBTINY_ECHO )
    // {
    //     return 8;
    // }
    // if    ( req == USBTINY_READ )
    // {
    //     data[0] = PIN;
    //     return 1;
    // }
    // if    ( req == USBTINY_WRITE )
    // {
    //     PORT = data[2];
    //     return 0;
    // }
    // bit = data[2] & 7;
    // mask = 1 << bit;
    // if    ( req == USBTINY_CLR )
    // {
    //     PORT &= ~ mask;
    //     return 0;
    // }
    // if    ( req == USBTINY_SET )
    // {
    //     PORT |= mask;
    //     return 0;
    // }
    // if    ( req == USBTINY_DDRWRITE )
    // {
    //     DDR = data[2];
    // }

    // // Programming requests
    // if    ( req == USBTINY_POWERUP )
    // {
    //     sck_period = data[2];
    //     mask = LED_MASK;
    //     if    ( data[4] )
    //     {
    //         mask |= RESET_MASK;
    //     }
    //     CLR(BUFFEN);
    //     DDR  = LED_MASK | RESET_MASK | SCK_MASK | MOSI_MASK;
    //     PORT = mask;
    //     return 0;
    // }
    // if    ( req == USBTINY_POWERDOWN )
    // {
    //     DDR  = 0x00;
    //     PORT = 0x00;
    //     SET(BUFFEN);
    //     return 0;
    // }
    // if    ( ! PORT )
    // {
    //     return 0;
    // }
    // if    ( req == USBTINY_SPI )
    // {
    //     spi( data + 2, data + 0, 4 );
    //     return 4;
    // }
    // if    ( req == USBTINY_SPI1 )
    // {
    //     spi( data + 2, data + 0, 1 );
    //     return 1;
    // }
    // if    ( req == USBTINY_POLL_BYTES )
    // {
    //     poll1 = data[2];
    //     poll2 = data[3];
    //     return 0;
    // }
    // address = * (uint_t*) & data[4];
    // if    ( req == USBTINY_FLASH_READ )
    // {
    //     cmd0 = 0x20;
    //     return 0xff;    // usb_in() will be called to get the data
    // }
    // if    ( req == USBTINY_EEPROM_READ )
    // {
    //     cmd0 = 0xa0;
    //     return 0xff;    // usb_in() will be called to get the data
    // }
    // timeout = * (uint_t*) & data[2];
    // if    ( req == USBTINY_FLASH_WRITE )
    // {
    //     cmd0 = 0x40;
    //     return 0;    // data will be received by usb_out()
    // }
    // if    ( req == USBTINY_EEPROM_WRITE )
    // {
    //     cmd0 = 0xc0;
    //     return 0;    // data will be received by usb_out()
    // }
    // return 0;
}

#define MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))

// Progmem or ram???
void memcpy_progmem(byte_t *destination, const void *progmem_src, uint8_t num)
{
    for (uint8_t i = 0; i < num; ++i)
    {
        destination[i] = 0x05;// pgm_read_byte(((byte_t *)progmem_src) + i);
        //((byte_t *)destination)[i] = ((byte_t *)source)[i];
    }
}

// ----------------------------------------------------------------------
// Handle an IN packet. device writes data to host via callback.
// ----------------------------------------------------------------------
byte_t usb_in(byte_t *data, byte_t len)
{
    //if (DATA_IN_HID_DESC == dataInCallback)
    // {
    //     const uint8_t alreadySent = sizeof(hid_desc) - dataInIter;
    //     const uint8_t toCopy = MIN(len, alreadySent);
    //     memcpy_progmem(data, hid_desc + dataInIter, toCopy);
    //     dataInIter += toCopy;
    //     return toCopy;
    // }
    // else if (DATA_IN_HID_REPORT_DESC == dataInCallback)
    // // {
    //     const uint8_t toSend = sizeof(hid_report_desc) - dataInIter;
    //     const uint8_t toCopy = MIN(len, toSend);
    //     //memcpy_progmem(data, hid_report_desc + dataInIter, toCopy);
    //     for (uint8_t i = 0; i < toCopy; ++i)
    //     {
    //         data[i] = 0xff;
    //     }
    //     dataInIter += toCopy;
    //     return toCopy;
    //}

    //return 0;
    // byte_t    i;

    // for    ( i = 0; i < len; i++ )
    // {
    //     spi_rw();
    //     data[i] = res[3];
    // }
    // return len;
    return 0;
}

// ----------------------------------------------------------------------
// Handle an OUT packet. - hosts wants to warite data to device.
// ----------------------------------------------------------------------
void usb_out( byte_t* data, byte_t len )
{
    // byte_t    i;
    // uint_t    usec;
    // byte_t    r;

    // for    ( i = 0; i < len; i++ )
    // {
    //     cmd[3] = data[i];
    //     spi_rw();
    //     cmd[0] ^= 0x60;    // turn write into read
    //     for    ( usec = 0; usec < timeout; usec += 32 * sck_period )
    //     {    // when timeout > 0, poll until byte is written
    //         spi( cmd, res, 4 );
    //         r = res[3];
    //         if    ( r == cmd[3] && r != poll1 && r != poll2 )
    //         {
    //             break;
    //         }
    //     }
    // }
}

typedef struct PortPin
{
    volatile uint8_t *port;
    uint8_t pin;
} PortPin_t;

#define KBRD_BIT(_n) (1 << (_n))
#define KBRD_NO_OPT_ADDR(_addr) ((volatile uint8_t *)(_addr))
#define KBRD_SFR_OFFSET __SFR_OFFSET
#define KBRD_PORTD_OFFSET 0x0B
#define KBRD_PORTD_ADDR KBRD_NO_OPT_ADDR(KBRD_SFR_OFFSET + KBRD_PORTD_OFFSET)

#define KBRD_PORT_PIN_SET(_pp)    ((*(_pp).port) |= (_pp).pin)
#define KBRD_PORT_PIN_RESET(_pp)  ((*(_pp).port) &= ~(_pp).pin)
#define KBRD_PORT_PIN_TOGGLE(_pp) ((*(_pp).port) ^= (_pp).pin)

// List of reserved ports (for USB and external oscylator).
// static const PortPin_t USB_D_PLUS_PP  = {KBRD_PORTD_ADDR, KBRD_BIT(2)};
// static const PortPin_t USB_D_MINUS_PP = {KBRD_PORTD_ADDR, KBRD_BIT(3)};
// static const PortPin_t KBRD_XTAL1     = {KBRD_PORTB_ADDR, KBRD_BIT(6)};
// static const PortPin_t KBRD_XTAL2     = {KBRD_PORTB_ADDR, KBRD_BIT(7)};

// typedef struct USB_KeyReport
// {
//   uint8_t modifiers;
//   uint8_t reserved;
//   uint8_t keys[6];
// } USB_KeyReport_t;

// Maybe is some faster way.
// void memset(void *addr, uint8_t value, uint8_t size)
// {
//     for (uint8_t i = 0; i < size; ++i)
//     {
//         *((uint8_t *)addr + i) = value;
//     }
// }

// USB_KeyReport_t gKeyReport;

// void kbrd_init()
// {
//     // Columns.
//     const uint8_t portBMask = KBRD_BIT(0) | KBRD_BIT(1) | KBRD_BIT(2) | KBRD_BIT(3) | KBRD_BIT(4) | KBRD_BIT(5);
//     DDRB = portBMask;
//     PORTB = 0;
//     const uint8_t portCMask = KBRD_BIT(0) | KBRD_BIT(1) | KBRD_BIT(2) | KBRD_BIT(3);
//     DDRC = portCMask;
//     PORTC = 0;
//     // Rows.
//     DDRC = 0;// ~(KBRD_BIT(4) | KBRD_BIT(5) | KBRD_BIT(6));
//     DDRD = 0;//~(KBRD_BIT(0) | KBRD_BIT(1) | KBRD_BIT(4) | KBRD_BIT(5) | KBRD_BIT(6) | KBRD_BIT(7));
//     PORTD = 0;
// }

// #define KBRD_IS_BIT_SET(_byte, _bit) (0 != (_byte) & KBRD_BIT(_bit))

// // Keyboard matrix consists of 19 pins.
// // 10 columns and 9 rows.
// // columns: PB 0, 1, 2, 3, 4, 5; PC 0, 1, 2, 3.
// // rows   : PC 4, 5, 6; PD 0, 1, 4, 5, 6, 7.

// // checks the matrix for key pressed and fill report.
// // max 6 pressed at once.
// void kbrd_scan()
// {
//     uint8_t keysPressed = 0;

//     // Clean report.
//     memset(&gKeyReport, 0, sizeof(gKeyReport));
//     // Set each column and read row.
//     PORTB |= KBRD_BIT(0);
//     // Check all rows.
//     if (KBRD_IS_BIT_SET(PORTC, 4)) // what key?
//     {
//         //gKeyReport
//     }
// }

uint8_t inPacketCount = 0, inPacketCountPrev = 0;

void init_in_token_count();
void load_in_token_count();

typedef struct USB_KeyReport
{
  uint8_t modifiers;
  uint8_t reserved;
  uint8_t keys[6];
} USB_KeyReport_t;

enum KeyboardState
{
    KBRD_STATE_PRESSED,
    KBRD_STATE_RELEASED
};

uint8_t keyState = KBRD_STATE_PRESSED;

void call_every_1000ms()
{
    const USB_KeyReport_t reportPressed = {0, 0, {KEY_A, KEY_B, KEY_C, KEY_D, KEY_E, KEY_F}};
    const USB_KeyReport_t reportReleased = {0, 0, {0, 0, 0, 0, 0, 0}};

    const USB_KeyReport_t *report = NULL;
    if (KBRD_STATE_PRESSED == keyState)
    {
        report = &reportPressed;
        keyState = KBRD_STATE_RELEASED;
    }
    else
    {
        report = &reportReleased;
        keyState = KBRD_STATE_PRESSED;
    }

    // TODO: turn into send_report function.
    cli();
    usb_tx_state = TX_STATE_RAM;
    usb_tx_total = sizeof(*report);
    usb_tx_data = (byte_t *)report;
    sei();
}
////////////////////////////////////////////////////////////////////////////////
static inline void set_bit(volatile uint8_t *port, uint8_t bit)
{
    *port |= _BV(bit);
}
static inline void reset_bit(volatile uint8_t *port, uint8_t bit)
{
    *port &= ~_BV(bit);
}
static inline void toggle_bit(volatile uint8_t *port, uint8_t bit)
{
    *port ^= _BV(bit);
}
////////////////////////////////////////////////////////////////////////////////
typedef struct IoPort
{
    volatile uint8_t *dir;
    volatile uint8_t *port;
    volatile uint8_t *pin;
} IoPort_t;
////////////////////////////////////////////////////////////////////////////////
static inline void ioport_input(const IoPort_t *ioport)
{
    *(ioport->dir) = 0x00;
}
static inline void ioport_output(const IoPort_t *ioport)
{
    *(ioport->dir) = 0xFF;
}
static inline void ioport_set(const IoPort_t *ioport)
{
    *(ioport->port) = 0xFF;
}
static inline void ioport_reset(const IoPort_t *ioport)
{
    *(ioport->port) = 0x00;
}
static inline void ioport_toggle(const IoPort_t *ioport)
{
    *(ioport->port) ^= 0xFF;
}
static inline uint8_t ioport_read(const IoPort_t *ioport)
{
    return *(ioport->pin);
}
static inline void ioport_write(const IoPort_t *ioport, uint8_t value)
{
    *(ioport->port) = value;
}
////////////////////////////////////////////////////////////////////////////////
static const IoPort_t IO_B = {&DDRB, &PORTB, &PINB};
static const IoPort_t IO_C = {&DDRC, &PORTC, &PINC};
static const IoPort_t IO_D = {&DDRD, &PORTD, &PIND};
////////////////////////////////////////////////////////////////////////////////
typedef struct IoPin
{
    IoPort_t *ioConfig;
    uint8_t pin;
} IoPin_t;
////////////////////////////////////////////////////////////////////////////////
static inline void iopin_set_input(const IoPin_t *iopin)
{
    reset_bit(iopin->ioConfig->dir, iopin->pin);
}
static inline void iopin_set_output(const IoPin_t *iopin)
{
    set_bit(iopin->ioConfig->dir, iopin->pin);
}
static inline void iopin_set(const IoPin_t *iopin)
{
    set_bit(iopin->ioConfig->port, iopin->pin);
}
static inline void iopin_reset(const IoPin_t *iopin)
{
    reset_bit(iopin->ioConfig->port, iopin->pin);
}
static inline void iopin_toggle(const IoPin_t *iopin)
{
    toggle_bit(iopin->ioConfig->port, iopin->pin);
}
static inline uint8_t iopin_read(const IoPin_t *iopin)
{
    return (*(iopin->ioConfig->pin) & _BV(iopin->pin));
}
////////////////////////////////////////////////////////////////////////////////
//static const IoPin_t IO_HEARTH_BEAT = {&IO_B, 0};
//static const IoPin_t IO_DEBUG       = {&IO_B, 1};
static const IoPin_t IO_HEARTH_BEAT = {&IO_D, PORTD4};
static const IoPin_t IO_DEBUG       = {&IO_D, PORTD5};

// #define DIR_REG_TUPLE(_port, _pin)  DDR ## _port, _pin
// #define PORT_REG_TUPLE(_port, _pin) (PORT ## (_port), _pin)
// #define PIN_REG_TUPLE(_port, _pin)  (PIN ## (_port), _pin)

// #define DEBUG_PORT       B, 0
// #define HEARTH_BEAT_PORT (B, 1)
// #define HEARTH_BEAT_PIN 1
// #define DEBUG_PIN 0

// DEBUG_PIN problematic???
static void led_init()
{
    // PORTB-1 used for USB_HID_REPORT_DESC_TYPE notification.
    // PORTB-2 used for hearth beat.
    // PORTB-3 used for IN packets count (usb driver change required).
    //DDRB |= _BV(1) | _BV(2) | _BV(3);
    //PORTB &= ~(_BV(1) | _BV(3));
    //PORTB |= _BV(2);

    //set_bit(&DDRB, HEARTH_BEAT_PIN); // direction: output
    //set_bit(&DDRB, DEBUG_PIN);       // direction: output
    //reset_bit(&PORTB, HEARTH_BEAT_PIN);
    //reset_bit(&PORTB, DEBUG_PIN);

    iopin_set_output(&IO_HEARTH_BEAT);
    iopin_set_output(&IO_DEBUG);
    iopin_reset(&IO_HEARTH_BEAT);
    iopin_reset(&IO_DEBUG);
}

USB_KeyReport_t reportPrev, reportNew;
uint8_t reportNew_keysCount;

//PORTx is for sending a value out to the port when it is set as an output. PINx is for use in getting the current value from the port when it is set as an input.
//PORTx also sets the state of the internal pull-up resistors when the port is set to input. And on more recent models, writing to PINx will toggle the value of the pins (the PORTx value) when they are set as outputs.
//The DDxn bit in the DDRx Register selects the direction of this pin. If DDxn is written logic one, Pxn is configured as an output pin. If DDxn is written logic zero, Pxn is configured as an input pin.
//If PORTxn is written logic one when the pin is configured as an input pin, the pull-up resistor is activated. To switch the pull-up resistor off, PORTxn has to be written logic zero or the pin has to be configured as an output pin.
// static void kbrd_init()
// {
//     // Init columns.
//     set_bit(&DDRC, 2); // direction: output.
//     set_bit(&DDRC, 3); // direction: output.
//     set_bit(&DDRC, 4); // direction: output.
//     set_bit(&DDRC, 5); // direction: output.
//     set_bit(&PORTC, 2); // output: high.
//     set_bit(&PORTC, 3); // output: high.
//     set_bit(&PORTC, 4); // output: high.
//     set_bit(&PORTC, 5); // output: high.
//     // Init rows.
//     reset_bit(&DDRB, 2); // direction: input.
//     reset_bit(&DDRB, 3); // direction: input.
//     reset_bit(&DDRB, 4); // direction: input.
//     reset_bit(&DDRB, 5); // direction: input.
//     set_bit(&PORTB, 2); // pull-up resistor: on.
//     set_bit(&PORTB, 3); // pull-up resistor: on.
//     set_bit(&PORTB, 4); // pull-up resistor: on.
//     set_bit(&PORTB, 5); // pull-up resistor: on.
//     // Reset HID-report buffers.
//     memset(&reportPrev, 0, sizeof(reportPrev));
//     memset(&reportNew, 0, sizeof(reportNew));
//     reportNew_keysCount = 0;
// }

static inline void send_hid_report(const USB_KeyReport_t *keyReport)
{
    cli();
    if (TX_STATE_IDLE != usb_tx_state)
    {
        goto DTOR;
    }
    usb_tx_state = TX_STATE_RAM;
    usb_tx_total = sizeof(*keyReport);
    usb_tx_data = (byte_t *)keyReport;
DTOR:
    sei();
}

#define MAX_KEYS_COUNT 6

uint8_t is_pending_report = 0;

static inline void hearth_beat()
{
    iopin_toggle(&IO_HEARTH_BEAT);
    //toggle_bit(&PORTB, HEARTH_BEAT_PIN);
}

/////////////////////////////////////////////////////////////
static const IoPin_t KBRD_IO_COLUMN[] =
{
    {&IO_B, PORTB0},
    {&IO_B, PORTB1},
    {&IO_B, PORTB2},
    {&IO_B, PORTB3},
    {&IO_B, PORTB4},
    {&IO_B, PORTB5},
    {&IO_C, PORTC0}
};
static const int KBRD_IO_COLUMN_SIZE = sizeof(KBRD_IO_COLUMN) / sizeof(KBRD_IO_COLUMN[0]);
/////////////////////////////////////////////////////////////
static const IoPin_t KBRD_IO_ROW[] =
{
    {&IO_C, PORTC1},
    {&IO_C, PORTC2},
    {&IO_C, PORTC3},
    {&IO_C, PORTC4},
    {&IO_C, PORTC5},
    //{&IO_C, PORTC6}, // reserved for RESET signal.
    {&IO_D, PORTD0},
    {&IO_D, PORTD1}
};
static const int KBRD_IO_ROW_SIZE = sizeof(KBRD_IO_ROW) / sizeof(KBRD_IO_ROW[0]);
/////////////////////////////////////////////////////////////

#define KBRD_PAGE_MAX 2

/**
 * @brief Information about matix pressed key (position and page).
 *        Used to map matrix key position to HID scan code.
 * 
 */
typedef union MatrixKeyInfo
{
    struct
    {
        uint8_t row      : 3; // bit 0..2: row index.
        uint8_t column   : 3; // bit 3..5: column index.
        uint8_t page     : 1; // bit 6: forward or reverse scan.
        uint8_t reserved : 1; // bit 7: not used now, must be 0.
    };
    uint8_t key;
} MatrixKeyInfo_t;

// Maps matrix pos X, Y (uint)
static const uint8_t KBRD_MATRIX_POS_TO_SCAN_CODE[] =
{
    /////////////////////////////
    // PAGE 0.
    //        R P COL ROW
    KEY_1, // 0 0 000 000
    KEY_Q, // 0 0 000 001
    KEY_A, // 0 0 000 010
    KEY_Z, // 0 0 000 011
    KEY_O, // 0 0 000 100
    KEY_O, // 0 0 000 101
    KEY_O, // 0 0 000 110
    KEY_O, // 0 0 000 111
    KEY_2, // 0 0 001 000
    KEY_W, // 0 0 001 001
    KEY_S, // 0 0 001 010
    KEY_X, // 0 0 001 011
    KEY_O, // 0 0 001 100
    KEY_O, // 0 0 001 101
    KEY_O, // 0 0 001 110
    KEY_O, // 0 0 001 111
    KEY_3, // 0 0 010 000
    KEY_E, // 0 0 010 001
    KEY_D, // 0 0 010 010
    KEY_C, // 0 0 010 011
    KEY_O, // 0 0 010 100
    KEY_O, // 0 0 010 101
    KEY_O, // 0 0 010 110
    KEY_O, // 0 0 010 111
    KEY_4, // 0 0 011 000
    KEY_R, // 0 0 011 001
    KEY_F, // 0 0 011 010
    KEY_V, // 0 0 011 011
    KEY_O, // 0 0 011 100
    KEY_O, // 0 0 011 101
    KEY_O, // 0 0 011 110
    KEY_O, // 0 0 011 111
    KEY_O, // 0 0 100 000
    KEY_O, // 0 0 100 001
    KEY_O, // 0 0 100 010
    KEY_O, // 0 0 100 011
    KEY_O, // 0 0 100 100
    KEY_O, // 0 0 100 101
    KEY_O, // 0 0 100 110
    KEY_O, // 0 0 100 111
    KEY_O, // 0 0 101 000
    KEY_O, // 0 0 101 001
    KEY_O, // 0 0 101 010
    KEY_O, // 0 0 101 011
    KEY_O, // 0 0 101 100
    KEY_O, // 0 0 101 101
    KEY_O, // 0 0 101 110
    KEY_O, // 0 0 101 111
    KEY_O, // 0 0 110 000
    KEY_O, // 0 0 110 001
    KEY_O, // 0 0 110 010
    KEY_O, // 0 0 110 011
    KEY_O, // 0 0 110 100
    KEY_O, // 0 0 110 101
    KEY_O, // 0 0 110 110
    KEY_O, // 0 0 110 111
    KEY_O, // 0 0 111 000
    KEY_O, // 0 0 111 001
    KEY_O, // 0 0 111 010
    KEY_O, // 0 0 111 011
    KEY_O, // 0 0 111 100
    KEY_O, // 0 0 111 101
    KEY_O, // 0 0 111 110
    KEY_O, // 0 0 111 111
    /////////////////////////////
    // PAGE 1.
    KEY_7, // 0 1 000 000
    KEY_8, // 0 1 000 001
    KEY_9, // 0 1 000 010
    KEY_0, // 0 1 000 011
    KEY_P, // 0 1 000 100
    KEY_P, // 0 1 000 101
    KEY_P, // 0 1 000 110
    KEY_P, // 0 1 000 111
    KEY_P, // 0 1 001 000
    KEY_P, // 0 1 001 001
    KEY_P, // 0 1 001 010
    KEY_P, // 0 1 001 011
    KEY_P, // 0 1 001 100
    KEY_P, // 0 1 001 101
    KEY_P, // 0 1 001 110
    KEY_P, // 0 1 001 111
    KEY_P, // 0 1 010 000
    KEY_P, // 0 1 010 001
    KEY_P, // 0 1 010 010
    KEY_P, // 0 1 010 011
    KEY_P, // 0 1 010 100
    KEY_P, // 0 1 010 101
    KEY_P, // 0 1 010 110
    KEY_P, // 0 1 010 111
    KEY_P, // 0 1 011 000
    KEY_P, // 0 1 011 001
    KEY_P, // 0 1 011 010
    KEY_P, // 0 1 011 011
    KEY_P, // 0 1 011 100
    KEY_P, // 0 1 011 101
    KEY_P, // 0 1 011 110
    KEY_P, // 0 1 011 111
    KEY_P, // 0 1 100 000
    KEY_P, // 0 1 100 001
    KEY_P, // 0 1 100 010
    KEY_P, // 0 1 100 011
    KEY_P, // 0 1 100 100
    KEY_P, // 0 1 100 101
    KEY_P, // 0 1 100 110
    KEY_P, // 0 1 100 111
    KEY_P, // 0 1 101 000
    KEY_P, // 0 1 101 001
    KEY_P, // 0 1 101 010
    KEY_P, // 0 1 101 011
    KEY_P, // 0 1 101 100
    KEY_P, // 0 1 101 101
    KEY_P, // 0 1 101 110
    KEY_P, // 0 1 101 111
    KEY_P, // 0 1 110 000
    KEY_P, // 0 1 110 001
    KEY_P, // 0 1 110 010
    KEY_P, // 0 1 110 011
    KEY_P, // 0 1 110 100
    KEY_P, // 0 1 110 101
    KEY_P, // 0 1 110 110
    KEY_P, // 0 1 110 111
    KEY_P, // 0 1 111 000
    KEY_P, // 0 1 111 001
    KEY_P, // 0 1 111 010
    KEY_P, // 0 1 111 011
    KEY_P, // 0 1 111 100
    KEY_P, // 0 1 111 101
    KEY_P, // 0 1 111 110
    KEY_P  // 0 1 111 111
};

/*
ERROR situation handling:
1/ could switch MCU into error HID device that will send error message to host repetadely.
*/

#define ENABLE_RUNTIME_ASSERTS

#if defined(ENABLE_RUNTIME_ASSERTS)
#define ASSERT_ARRAY_RANGE(_index, _array) \
    do { \
        if ((_index) >= sizeof(_array)) { \
            return; \
        } \
    } while (0)
#else
#define ASSERT_ARRAY_RANGE(_index, _array) do {} while (0)
#endif // ENABLE_RUNTIME_ASSERTS

#define IS_SCAN_CODE_MODIFIER(_scan_code) \
    ((_scan_code) >= KEY_LEFTCTRL && (_scan_code) <= KEY_RIGHTMETA)

static inline void kbrd_hid_report_update_modifier(uint8_t modifier_scan_code)
{
    const uint8_t bit_index = modifier_scan_code - KEY_LEFTCTRL;
    set_bit(&reportNew.modifiers, bit_index);
}

/**
 * @brief Pass key by value, as it is 1 byte in size.
 * 
 * @param key 
 */
static void kbrd_hid_report_update(MatrixKeyInfo_t key)
{
    // Make sure that key is mapped.
    if (sizeof(KBRD_MATRIX_POS_TO_SCAN_CODE) <= key.key)
    {
        return; // error.
    }
    // Map matrix key position to HID scan code.
    const uint8_t hidScanCode = KBRD_MATRIX_POS_TO_SCAN_CODE[key.key];
    // Process special/modifier keys.
    if (IS_SCAN_CODE_MODIFIER(hidScanCode))
    {
        kbrd_hid_report_update_modifier(key.key);
        return;
    }
    // Process normal key.
    ASSERT_ARRAY_RANGE(reportNew_keysCount, reportNew.keys); // could be disabled.
    reportNew.keys[reportNew_keysCount++] = hidScanCode;
}

static const IoPin_t *KBRD_IO_IN  = NULL;
static const IoPin_t *KBRD_IO_OUT = NULL;

// static void kbrd_init()
// {
//     // Init each column.
//     for (uint8_t col = 0; col < KBRD_IO_COLUMN_SIZE; ++col)
//     {
//         iopin_set_output(KBRD_IO_COLUMN + col); // direction: output.
//         iopin_set(KBRD_IO_COLUMN + col);        // output: high.
//     }
//     // Init each row.
//     for (uint8_t row = 0; row < KBRD_IO_ROW_SIZE; ++row)
//     {
//         iopin_set_input(KBRD_IO_ROW + row); // direction: input.
//         iopin_set(KBRD_IO_ROW + row);       // pull-up resistor: on.
//     }
//     // Reset HID-report buffers.
//     memset(&reportPrev, 0, sizeof(reportPrev));
//     memset(&reportNew, 0, sizeof(reportNew));
//     reportNew_keysCount = 0;
// }

static void kbrd_hid_init()
{
    // Reset HID-report buffers.
    memset(&reportPrev, 0, sizeof(reportPrev));
    memset(&reportNew, 0, sizeof(reportNew));
    reportNew_keysCount = 0;
}

static void kbrd_page_init()
{
    // Init each column.
    for (uint8_t col = 0; col < KBRD_IO_COLUMN_SIZE; ++col)
    {
        iopin_set_output(KBRD_IO_OUT + col); // direction: output.
        iopin_set(KBRD_IO_OUT + col);        // output: high.
    }
    // Init each row.
    for (uint8_t row = 0; row < KBRD_IO_ROW_SIZE; ++row)
    {
        iopin_set_input(KBRD_IO_IN + row); // direction: input.
        iopin_set(KBRD_IO_IN + row);       // pull-up resistor: on.
    }
}

/**
 * @brief Configures IO for either page 0 or 1.
 *        Page 0 is: COLUMN is output, ROW is input.
 *        Page 1 is: COLUMN is input , ROW is output.
 * 
 * @param page 
 */
static void kbrd_set_page(uint8_t page)
{
    if (0 == page)
    {
        KBRD_IO_IN  = KBRD_IO_ROW;
        KBRD_IO_OUT = KBRD_IO_COLUMN;
    }
    else
    {
        KBRD_IO_IN  = KBRD_IO_COLUMN;
        KBRD_IO_OUT = KBRD_IO_ROW;
    }

    kbrd_page_init();
}

/**
 * @brief One thing to notice - column and row index will be switched in MatrixKeyInfo_t
 *        one page 1. Maybe need to swap them to keep indexing consistent.
 * 
 */
static void kbrd_scan()
{
    MatrixKeyInfo_t key;

    // Only if there is no pending report to send.
    if (TX_STATE_IDLE != usb_tx_state)
    {
        return;
    }

    // reserved field must be 0.
    key.reserved = 0;

    // For each page.
    for (uint8_t page = 0; page < KBRD_PAGE_MAX; ++page)
    {
        // Reconfigure keyboard matrix page (switch roles of column and row).
        kbrd_set_page(page);
        // Update key page.
        key.page = page;
        // Set each column low in sequence and sense rows low in sequence.
        for (uint8_t col = 0; col < KBRD_IO_COLUMN_SIZE; ++col)
        {
            iopin_reset(KBRD_IO_OUT + col); // output: LOW.
            key.column = col;
            for (uint8_t row = 0; row < KBRD_IO_ROW_SIZE; ++row)
            {
                if (0 != iopin_read(KBRD_IO_IN + row)) // not pressed.
                {
                    continue;
                }
                // Make full key.
                key.row = row;
                // Save pressed key in report.
                kbrd_hid_report_update(key);
                // Check if report is full.
                if (reportNew_keysCount >= 6) // check cannot be disabled.
                {
                    break;
                }
            }
            iopin_set(KBRD_IO_OUT + col);   // output: HIGH.
            // Check if report is full.
            if (reportNew_keysCount >= 6) // check cannot be disabled.
            {
                break;
            }
        }
        // Check if report is full.
        if (reportNew_keysCount >= 6) // check cannot be disabled.
        {
            break;
        }
    }

    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////

    // Send report only if new report is different from previous one.
    if (0 != memcmp(&reportPrev, &reportNew, sizeof(USB_KeyReport_t)))
    {
        memcpy(&reportPrev, &reportNew, sizeof(USB_KeyReport_t));
        send_hid_report(&reportPrev);
        iopin_toggle(&IO_DEBUG);
    }

    // Reset new report if needed.
    if (0 < reportNew_keysCount)
    {
        reportNew_keysCount = 0;
        memset(&reportNew, 0, sizeof(USB_KeyReport_t));
    }
}

int main()
{
    uint16_t timer_1000ms = 0;
    uint8_t  timer_10ms  = 0;
    uint8_t init_done = 0;

    led_init();
    //kbrd_init();
    kbrd_set_page(0);
    kbrd_hid_init();
    usb_init();

    for (;;)
    {
        if (0 != init_done)
        {
            if (timer_10ms++ >= 10)
            {
                kbrd_scan();
                timer_10ms = 0;
            }
        }

        usb_poll();

        if (timer_1000ms++ >= 1000)
        {
            init_done = 1;
            timer_1000ms = 0;

            //call_every_1000ms();

            hearth_beat();
        }

        _delay_ms(1);
    }

    return 0;
}
