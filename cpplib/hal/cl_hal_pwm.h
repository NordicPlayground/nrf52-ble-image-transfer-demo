#ifndef __CPPLIB_HAL_PWM_H
#define __CPPLIB_HAL_PWM_H

#include <stdint.h>
#include "nrf.h"

namespace CppLib {

class Pwm;
    
class PwmChannel
{
private:
    PwmChannel &mPwmReference;
    uint32_t    mPwmIndex;
public:
    PwmChannel(void);
    void open(Pwm &parent, uint32_t index);
    void operator=(int percentage);
    void operator=(float ratio);
};

class Pwm
{
private:
    NRF_PWM_Type *mBase;
    uint32_t mPwmPeripheralIndex;
    bool mOpen;
    uint32_t mPinNum[4];
    PwmChannel mChannel[4];

public:
    Pwm(uint32_t pin1);
    Pwm(uint32_t pin1, uint32_t pin2);
    Pwm(uint32_t pin1, uint32_t pin2, uint32_t pin3);
    Pwm(uint32_t pin1, uint32_t pin2, uint32_t pin3, uint32_t pin4);

    PwmChannel &operator[](int index);
};    


}

#endif
