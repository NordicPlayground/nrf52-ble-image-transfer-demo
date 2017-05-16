#include "hal//cl_hal_ppi.h"

namespace CppLib {
    
PPIChannel::PPIChannel(void)
{
    mIndex = 0xFFFFFFFF;
}

bool PPIChannel::validIndex()
{
    if(mIndex < 32) return true;
    else 
    {
        nrfSystem.registerError(LS_ERROR, "PPIChannel", 0, "PPI channel not open or invalid");   
        return false;    
    }
}

void PPIChannel::open()
{
    mIndex = nrfSystem.allocPPIChannel();
    if(mIndex >= 32)
    {
        nrfSystem.registerError(LS_ERROR, "PPIChannel: open", 0, "No free PPI channels");
    }
}

void PPIChannel::open(volatile uint32_t *event, volatile uint32_t *task)
{
    open();
    if(validIndex())
    {
        NRF_PPI->CH[mIndex].EEP = (uint32_t)event;
        NRF_PPI->CH[mIndex].TEP = (uint32_t)task;       
    }
}

void PPIChannel::assign(volatile uint32_t *event, volatile uint32_t *task)
{
    if(validIndex())
    {
        NRF_PPI->CH[mIndex].EEP = (uint32_t)event;
        NRF_PPI->CH[mIndex].TEP = (uint32_t)task;       
    }    
}

void PPIChannel::enable()
{
    if(validIndex())
    {
        NRF_PPI->CHENSET = (1 << mIndex);
    }
}

void PPIChannel::disable()
{
    if(validIndex())
    {
        NRF_PPI->CHENCLR = (1 << mIndex);
    }
}

void PPIChannel::close()
{
    if(validIndex())
    {
        nrfSystem.deallocPPIChannel(mIndex);
        mIndex = 0xFFFFFFFF;
    }
}

}
