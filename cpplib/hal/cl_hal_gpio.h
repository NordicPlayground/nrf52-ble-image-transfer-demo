#ifndef __ULIB_GPIO_H
#define __ULIB_GPIO_H

#include <stdint.h>
#include "nrf.h"

namespace CppLib{
    
class Pin
{
protected:
    uint32_t mPinNum, mPinNumMask;
    volatile NRF_GPIO_Type *mGpioPort;
public:    
    Pin();
    Pin(uint32_t pinNum);

    uint32_t get();

    void open(uint32_t pinNum);
    void close();
};

class PinOut : public Pin
{
public:  
    PinOut();
    PinOut(uint32_t pinNum);

    void open(uint32_t pinNum);

    void set(uint32_t state);
    uint32_t get();
    void high();
    void low();
    void toggle();
    int operator=(int value);
    int operator=(Pin &value);
};

class Led : public PinOut
{
    bool mInverted;
public:
    Led(uint32_t pinNum);
    Led(uint32_t pinNum, bool inverted);

    void on(); 
    void off();
    int operator=(int value);
    int operator=(Pin &value);
};

class PinIn : public Pin
{
public:
    PinIn();
    PinIn(uint32_t pinNum);
    PinIn(uint32_t pinNum, uint32_t pullUp);

    void open(uint32_t pinNum);
    void open(uint32_t pinNum, uint32_t pullUp);
    void close();

    bool isHigh();
    bool isLow();
};

class Button : public PinIn
{
    bool mInverted;
    bool mPressedLast;
    
public:
    Button(uint32_t pinNum);
    Button(uint32_t pinNum, bool inverted);

    bool isDown();
    bool isUp();
    bool isPressed();
    bool isReleased();
};

}
#endif
