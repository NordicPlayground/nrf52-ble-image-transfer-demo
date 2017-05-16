#ifndef __CPPLIB_PPI_H
#define __CPPLIB_PPI_H

#include "system//cl_system.h"

namespace CppLib {

class Event
{
private:  
    volatile uint32_t *mBaseRegister;
public:
    Event(){ mBaseRegister = 0; }
    Event(uint32_t baseRegAddr){ mBaseRegister = (volatile uint32_t *)baseRegAddr; }
    
    void open(uint32_t baseRegAddr){ mBaseRegister = (volatile uint32_t *)baseRegAddr; }
    
    uint32_t getRegisterAddress(){ return (uint32_t)mBaseRegister; }
    
    bool get(){ return *mBaseRegister != 0; }
    void waitUntilSet(){ while(*mBaseRegister == 0); }
    void sleepUntilSet(){ while(*mBaseRegister == 0) __WFE(); }
    void clear(){ *mBaseRegister = 0; }
};

class Task
{
private:
    volatile uint32_t *mBaseRegister;    
public:
    Task(){ mBaseRegister = 0; }
    Task(uint32_t baseRegAddr){ mBaseRegister = (volatile uint32_t *)baseRegAddr; }
    
    void open(uint32_t baseRegAddr){ mBaseRegister = (volatile uint32_t *)baseRegAddr; }
    
    void activate(){ *mBaseRegister = 1; }
};
    
class PPIChannel
{
    uint32_t mIndex;
    
    bool validIndex();
    
public:
    PPIChannel(void);
    void open();
    void open(volatile uint32_t *event, volatile uint32_t *task);
    void assign(volatile uint32_t *event, volatile uint32_t *task);
    void enable();
    void disable();
    void close();
};

} // End of namespace CppLib

#endif
