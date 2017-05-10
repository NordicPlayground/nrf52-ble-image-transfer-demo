#ifndef __ULIB_PPI_H
#define __ULIB_PPI_H

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
    
class PPI
{
public:
    void open(Event &event, Task &task);
    void close();
};

} // End of namespace CppLib

#endif
