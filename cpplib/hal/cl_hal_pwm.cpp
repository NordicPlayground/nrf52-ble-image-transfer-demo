#include "cl_hal_pwm.h"

PwmChannel::PwmChannel()
{
    
}

Pwm::Pwm(uint32_t pin1)
{
    if((mBase = nrfSystem.allocPwm(&mPwmPeripheralIndex)) != 0)
    {
        mOpen = true;
        nrfSystem.registerGpio(mBase->PSEL.OUT[0] = pin1);
    }
    else
    {
        mOpen = false;
    }
}

Pwm::Pwm(uint32_t pin1, uint32_t pin2)
{
    
}

Pwm::Pwm(uint32_t pin1, uint32_t pin2, uint32_t pin3)
{
    
}

Pwm::Pwm(uint32_t pin1, uint32_t pin2, uint32_t pin3, uint32_t pin4)
{
    
}