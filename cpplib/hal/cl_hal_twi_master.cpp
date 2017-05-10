#include "hal//cl_hal_twi_master.h"
#include "system//cl_system.h"

namespace CppLib {
    
TwiMaster *TwiMaster::mActiveClassList[CPPLIB_BOARD_TWIM_INTERFACE_NUM] = {0};

TwiMaster::TwiMaster()
{
    mOpen           = false;
    mTwimActive     = false;
    pinSda          = CPPLIB_BOARD_TWIM_PIN_SDA;
    pinScl          = CPPLIB_BOARD_TWIM_PIN_SCL;
    address         = 1;
    
    callbackTransactionComplete = 0;
    callbackError               = 0;
}

void TwiMaster::open()
{
    if(mOpen) nrfSystem.registerError(LS_ERROR, "TWIM: Open", 0, "Interface already open");
    else if((mBase = nrfSystem.allocTwim(&mTwimPeripheralIndex)) != 0)
    {
        mOpen = true;
        nrfSystem.registerTwimIrq(TwiMaster::onIrq);
        mActiveClassList[mTwimPeripheralIndex] = this;
        nrfSystem.registerGpio(mBase->PSEL.SDA = pinSda);
        nrfSystem.registerGpio(mBase->PSEL.SCL = pinScl);
        mBase->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K100 << TWIM_FREQUENCY_FREQUENCY_Pos;
        mBase->EVENTS_STOPPED = mBase->EVENTS_ERROR = 0;
        NVIC_SetPriority((IRQn_Type)cpplibResourceTwimIrqN[mTwimPeripheralIndex], 7);
        NVIC_EnableIRQ((IRQn_Type)cpplibResourceTwimIrqN[mTwimPeripheralIndex]);
        mBase->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
        nrfSystem.registerError(LS_DEBUG, "TWIM: Open", 0, "TWI master interface created");
    }
    else nrfSystem.registerError(LS_ERROR, "TWIM: Open", 0, "No peripheral free");
}

void TwiMaster::close()
{
    if(mOpen)
    {
        while(mTwimActive);
        NVIC_DisableIRQ((IRQn_Type)cpplibResourceTwimIrqN[mTwimPeripheralIndex]);
        NVIC_ClearPendingIRQ((IRQn_Type)cpplibResourceTwimIrqN[mTwimPeripheralIndex]);
        nrfSystem.deallocTwim(mBase);
        mActiveClassList[mTwimPeripheralIndex] = 0;
        nrfSystem.freeGpio(mBase->PSEL.SDA);
        nrfSystem.freeGpio(mBase->PSEL.SCL);
        mOpen = false;
        nrfSystem.registerError(LS_DEBUG, "TWIM: Close", 0, "Interface closed");
    }
    else nrfSystem.registerError(LS_ERROR, "TWIM: Close", 0, "Interface not open");
}

void TwiMaster::txRx(uint8_t *txBuf, uint32_t txLength, uint8_t *rxBuf, uint32_t rxLength)
{
    if(mOpen)
    {
        while(mTwimActive) __WFE();
        mTwimActive = true;
        mErrorLast = 0;
        mBase->ADDRESS = address;
        mBase->TXD.PTR = (uint32_t)txBuf;
        mBase->TXD.MAXCNT = txLength;
        mBase->RXD.PTR = (uint32_t)rxBuf;
        mBase->RXD.MAXCNT = (rxLength);
        mBase->SHORTS = TWIM_SHORTS_LASTTX_STARTRX_Msk | TWIM_SHORTS_LASTRX_STOP_Msk;
        mBase->INTENSET = TWIM_INTENSET_STOPPED_Msk | TWIM_INTENSET_ERROR_Msk;
        mBase->TASKS_STARTTX = 1;
    }
    else nrfSystem.registerError(LS_ERROR, "TWIM: Transfer", 0, "Interface not open");
}

void TwiMaster::tx(uint8_t *txData, uint32_t txLength)
{
    if(mOpen)
    {
        while(mTwimActive) __WFE();
        mTwimActive = true;
        mErrorLast = 0;
        mBase->ADDRESS = address;
        mBase->TXD.PTR = (uint32_t)txData;
        mBase->TXD.MAXCNT = txLength;
        mBase->RXD.PTR = 0;
        mBase->RXD.MAXCNT = 0;
        mBase->SHORTS = TWIM_SHORTS_LASTTX_STOP_Msk;
        mBase->INTENSET = TWIM_INTENSET_STOPPED_Msk | TWIM_INTENSET_ERROR_Msk;
        mBase->TASKS_STARTTX = 1;
    }
    else nrfSystem.registerError(LS_ERROR, "TWIM: Transfer", 0, "Interface not open");
}

uint32_t TwiMaster::completeOperation()
{
    while(mTwimActive);
    return mErrorLast;
}

void TwiMaster::onIrqInstance(void)
{
    if(mBase->EVENTS_STOPPED)
    {
        mBase->EVENTS_STOPPED = 0;
        mTwimActive = false;
        if(callbackTransactionComplete)
        {
            callbackTransactionComplete((uint8_t*)mBase->TXD.PTR,mBase->TXD.AMOUNT, (uint8_t*)mBase->RXD.PTR, mBase->RXD.AMOUNT);
        }
    }
    if(mBase->EVENTS_ERROR)
    {
        mBase->EVENTS_ERROR = 0;
        mErrorLast = mBase->ERRORSRC;
        mTwimActive = false;
        if(callbackError) callbackError(mErrorLast);
        //else nrfSystem.registerError(ERROR, "TWIM: IRQ", 0, "TWI error"); 
    }
}

void TwiMaster::onIrq(uint32_t peripheralIndex)
{
    if(peripheralIndex < CPPLIB_BOARD_TWIM_INTERFACE_NUM && mActiveClassList[peripheralIndex])
    {
        mActiveClassList[peripheralIndex]->onIrqInstance();
    }
}

} // End of namespace CppLib
