#include <stdint.h>
#include <avr/io.h>

class IoPort
{
private:
    volatile uint8_t &_dirReg;
    volatile uint8_t &_portReg;
    volatile uint8_t &_pinReg;

public:
    enum class Direction
    {
        INPUT  = 0,
        OUTPUT = 1
    };

    IoPort(volatile uint8_t &dirReg, volatile uint8_t &portReg, volatile uint8_t &pinReg) :
        _dirReg(dirReg),
        _portReg(portReg),
        _pinReg(pinReg)
    {
    }
    // inline void SetPinDir(uint8_t pin, Direction dir)
    // {
    //     _dirReg = (_dirReg & ~_BV(pin)) | 
    // }
    inline void SetInput(uint8_t pin)
    {
    }
    inline void SetOutput()
    {
    }
};

void test()
{
    IoPort portB(DDRB, PORTB, PINB);
    portB.SetInput(1);
}