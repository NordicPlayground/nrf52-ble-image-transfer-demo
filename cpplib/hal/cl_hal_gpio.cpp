#include "hal//cl_hal_gpio.h"
#include "system//cl_system.h"

namespace CppLib{
    
Pin::Pin()
{
    mPinNum = 0xFFFFFFFF;
    mPinNumMask = 0;
}

Pin::Pin(uint32_t pinNum)
{
    open(pinNum);
}

uint32_t Pin::get()
{
    return ((mGpioPort->IN & mPinNumMask) != 0 ? 1 : 0);
}

void Pin::open(uint32_t pinNum)
{
    mPinNum = pinNum; 
    mPinNumMask = 1 << (pinNum % 32); 
#if(CPPLIB_BOARD_GPIO_NUM > 32)
    mGpioPort = (pinNum >= 32 ? NRF_P1 : NRF_P0);
#else
    mGpioPort = NRF_P0;
#endif
    nrfSystem.registerGpio(pinNum);    
}

// PinOut ----------------------------------------------

PinOut::PinOut() : Pin()
{
        
}

PinOut::PinOut(uint32_t pinNum) : Pin(pinNum)
{
    open(pinNum);
}

void PinOut::open(uint32_t pinNum)
{
    Pin::open(pinNum);
    mGpioPort->DIRSET = mPinNumMask;  
}

void PinOut::set(uint32_t state)
{
    if(state) mGpioPort->OUTSET = mPinNumMask;
    else mGpioPort->OUTCLR = mPinNumMask;
}

void PinOut::high()
{
    mGpioPort->OUTSET = mPinNumMask;
}
void PinOut::low()
{
    mGpioPort->OUTCLR = mPinNumMask;
}
void PinOut::toggle()
{
    mGpioPort->OUT ^= mPinNumMask;
}

int PinOut::operator=(int value)
{
    set(value);
    return value;
}

int PinOut::operator=(Pin &value)
{
    set(value.get());
    return value.get();
}
// Led -------------------------------------------

Led::Led(uint32_t pinNum) : PinOut(pinNum)
{
    mInverted = false;
}

Led::Led(uint32_t pinNum, bool inverted) : PinOut(pinNum)
{
    nrfSystem.registerError(LS_DEBUG, "LED", pinNum, "LED created");
    mInverted = inverted;
}

void Led::on()
{
    mInverted ? low() : high();
}

void Led::off()
{
    mInverted ? high() : low();
}

int Led::operator=(int value)
{
    value ? on() : off();
    return value;
}

int Led::operator=(Pin &value)
{
    value.get() ? on() : off();
    return value.get();
}

// PinIn

PinIn::PinIn() : Pin()
{
    
}

PinIn::PinIn(uint32_t pinNum) : Pin(pinNum)
{
    open(pinNum);
}

PinIn::PinIn(uint32_t pinNum, uint32_t pullUp) : Pin(pinNum)
{
    open(pinNum, pullUp);
}

void PinIn::open(uint32_t pinNum)
{
    Pin::open(pinNum);
    mGpioPort->DIRCLR = mPinNumMask;      
}

void PinIn::open(uint32_t pinNum, uint32_t pullUp)
{
    Pin::open(pinNum);
    mGpioPort->DIRCLR = mPinNumMask;  
    mGpioPort->PIN_CNF[pinNum % 32] = GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos |
                                      GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos |
                                      pullUp << GPIO_PIN_CNF_PULL_Pos;
}

bool PinIn::isHigh()
{
    return mGpioPort->IN & mPinNumMask;
}

bool PinIn::isLow()
{
    return !(mGpioPort->IN & mPinNumMask);
}

Button::Button(uint32_t pinNum) : PinIn(pinNum)
{
    mInverted = false;
    NRF_GPIO->PIN_CNF[pinNum] = GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos |
                                GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos |
                                GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos;
    mPressedLast = false;
}

Button::Button(uint32_t pinNum, bool inverted) : PinIn(pinNum)
{
    mInverted = inverted;
    mGpioPort->PIN_CNF[pinNum] = GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos |
                                 GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos |
                                 (mInverted ? GPIO_PIN_CNF_PULL_Pullup : GPIO_PIN_CNF_PULL_Pulldown) << GPIO_PIN_CNF_PULL_Pos; 
}

bool Button::isDown()
{
    return mInverted ? isLow() : isHigh();
}

bool Button::isUp()
{
    return mInverted ? isHigh() : isLow();
}

bool Button::isPressed()
{
    bool mPressed = isDown();
    if(mPressed && !mPressedLast)
    {
        mPressedLast = mPressed;
        return true;
    }
    else 
    {
        mPressedLast = mPressed;
        return false;
    }
}

bool Button::isReleased()
{
    bool mPressed = isDown();
    if(!mPressed && mPressedLast)
    {
        mPressedLast = mPressed;
        return true;
    }
    else 
    {
        mPressedLast = mPressed;
        return false;
    }
}
} // Namespace CppLib
