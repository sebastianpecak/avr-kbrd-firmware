void port_pin_test_atmega328()
{
    
}

/////////////////////////////////////////////////////////////
// PORT-B
static inline void port_b_output()
{
    DDRB  = 0xFF;
    PORTB = 0x00; // output: low
}
static inline void port_b_input()
{
    DDRB  = 0x00;
    PORTB = 0xFF; // pull-up resistor: on
}
static inline void port_b_output_1000ms()
{
    PORTB ^= 0xFF; // toggle all bits.
}
static inline void port_b_input_1000ms()
{
    static uint8_t prevPortState = 0;
    const uint8_t newPortState = PINB;
    if (newPortState != prevPortState)
    {
        iopin_toggle(&IO_DEBUG);
        //toggle_bit(&PORTB, DEBUG_PIN);
        prevPortState = newPortState;
    }
}
// PORT-B
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
// PORT-C
static inline void port_c_output()
{
    DDRC  = 0xFF;
    PORTC = 0x00; // output: low
}
static inline void port_c_input()
{
    DDRC  = 0x00;
    PORTC = 0xFF; // pull-up resistor: on
}
static inline void port_c_output_1000ms()
{
    PORTC ^= 0xFF; // toggle all bits.
}
static inline void port_c_input_1000ms()
{
    static uint8_t prevPortState = 0;
    const uint8_t newPortState = PINC;
    if (newPortState != prevPortState)
    {
        iopin_toggle(&IO_DEBUG);
        //toggle_bit(&PORTB, DEBUG_PIN);
        prevPortState = newPortState;
    }
}
// PORT-C
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
// PORT-D
static const IoPin_t FREE_D_PINS[] =
{
    {&IO_D, PORTD0},
    {&IO_D, PORTD1},
    //{&IO_D, PORTD2}, // USB D+
    //{&IO_D, PORTD3}, // USB D-
    {&IO_D, PORTD4},
    {&IO_D, PORTD5},
    {&IO_D, PORTD6},
    {&IO_D, PORTD7}
};
static const int FREE_D_PINS_SIZE = sizeof(FREE_D_PINS) / sizeof(FREE_D_PINS[0]);
/////////////////////////////////////////////////////////////
static inline void port_d_output()
{
    for (int i = 0; i < FREE_D_PINS_SIZE; ++i)
    {
        iopin_set_output(FREE_D_PINS + i);
        iopin_reset(FREE_D_PINS + i);
    }
}
static inline void port_d_input()
{
    for (int i = 0; i < FREE_D_PINS_SIZE; ++i)
    {
        iopin_set_input(FREE_D_PINS + i);
        iopin_set(FREE_D_PINS + i); // pull-up resistor: on
    }
}
static inline void port_d_output_1000ms()
{
    for (int i = 0; i < FREE_D_PINS_SIZE; ++i)
    {
        iopin_toggle(FREE_D_PINS + i);
    }
}
static inline void port_d_input_1000ms()
{
    static uint8_t prevPortState = 0;
    // Only used ports.
    const uint8_t newPortState = ioport_read(&IO_D) & 0xF3;
    if (newPortState != prevPortState)
    {
        iopin_toggle(&IO_DEBUG);
        prevPortState = newPortState;
    }
}
// PORT-D
/////////////////////////////////////////////////////////////