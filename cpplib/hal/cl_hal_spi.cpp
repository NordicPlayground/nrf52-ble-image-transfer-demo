#include "hal//cl_hal_spi.h"
#include "system//cl_system.h"

namespace CppLib {
    
SpiMaster *SpiMaster::mActiveClassList[CPPLIB_BOARD_SPIM_INTERFACE_NUM] = {0};

SpiMaster::SpiMaster()
{
    mOpen           = false;
    mSpiActive      = false;
    mInPartialTrans = false;
    blocking        = true;
    pinSck          = CPPLIB_BOARD_SPIM_PIN_SCK;
    pinMosi         = CPPLIB_BOARD_SPIM_PIN_MOSI;
    pinMiso         = CPPLIB_BOARD_SPIM_PIN_MISO;
    pinCsn          = CPPLIB_BOARD_SPIM_PIN_CSN;
    frequency       = SPIM_FREQUENCY_FREQUENCY_M1;
    clockPolarity   = USPI_CLOCK_POLARITY_HIGH;
    clockPhase      = USPI_CLOCK_PHASE_LEADING;
    bitOrder        = USPI_BIT_ORDER_MSB_LSB;
    mCallback       = 0;
}

void SpiMaster::open()
{
    if(mOpen) nrfSystem.registerError(LS_ERROR, "SPIM: Open", 0, "Interface already open");
    else if((mBase = nrfSystem.allocSpim(&mSpimPeripheralIndex)) != 0)
    {
        mOpen = true;
        nrfSystem.registerSpimIrq(SpiMaster::onIrq);
        mActiveClassList[mSpimPeripheralIndex] = this;
        nrfSystem.registerGpio(mBase->PSEL.SCK = pinSck);
        nrfSystem.registerGpio(mBase->PSEL.MOSI = pinMosi);
        nrfSystem.registerGpio(mBase->PSEL.MISO = pinMiso);
        mCsnPin.open(pinCsn);
        mBase->FREQUENCY = frequency << SPIM_FREQUENCY_FREQUENCY_Pos;
        mBase->CONFIG = clockPhase      << SPIM_CONFIG_CPHA_Pos |
                        clockPolarity   << SPIM_CONFIG_CPOL_Pos | 
                        bitOrder        << SPIM_CONFIG_ORDER_Pos;
        mBase->INTENSET = SPIM_INTENSET_END_Msk;
        mBase->EVENTS_END = 0;
        NVIC_SetPriority((IRQn_Type)cpplibResourceSpimIrqN[mSpimPeripheralIndex], 7);
        NVIC_EnableIRQ((IRQn_Type)cpplibResourceSpimIrqN[mSpimPeripheralIndex]);
        mBase->ENABLE = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;
        nrfSystem.registerError(LS_DEBUG, "SPIM", 0, "Spi master interface created");
    }
    else nrfSystem.registerError(LS_ERROR, "SPIM: Open", 0, "No peripheral free");
}

void SpiMaster::close()
{
    if(mOpen)
    {
        while(mSpiActive);
        NVIC_DisableIRQ((IRQn_Type)cpplibResourceSpimIrqN[mSpimPeripheralIndex]);
        NVIC_ClearPendingIRQ((IRQn_Type)cpplibResourceSpimIrqN[mSpimPeripheralIndex]);
        nrfSystem.deallocSpim(mBase);
        mActiveClassList[mSpimPeripheralIndex] = 0;
        nrfSystem.freeGpio(mBase->PSEL.SCK);
        nrfSystem.freeGpio(mBase->PSEL.MOSI);
        nrfSystem.freeGpio(mBase->PSEL.MISO);
        nrfSystem.freeGpio(pinCsn);
        mOpen = false;
        nrfSystem.registerError(LS_DEBUG, "SPIM", 0, "Spi master interface closed");
    }
    nrfSystem.registerError(LS_ERROR, "SPIM: Close", 0, "Interface not open");
}

void SpiMaster::transfer(uint8_t *txBuf, uint8_t *rxBuf, uint32_t length)
{
    if(length > CPPLIB_SPI_MAXLENGTH)
    {
        nrfSystem.registerError(LS_ERROR, "SPIM: Transfer", 0, "Transaction length exceeded");
        return;
    }
    if(mOpen)
    {
        while(mSpiActive) __WFE();
        mSpiActive = true;
        mBase->TXD.PTR      = (uint32_t)txBuf;
        mBase->TXD.MAXCNT   = length;
        mBase->RXD.PTR      = (uint32_t)rxBuf;
        mBase->RXD.MAXCNT   = length;
        if(!mInPartialTrans) mCsnPin.low();
        mBase->TASKS_START  = 1;
        if(blocking) while(mSpiActive);
    }
    else nrfSystem.registerError(LS_ERROR, "SPIM: Transfer", 0, "Interface not open");
}

void SpiMaster::transmit(uint8_t *txBuf, uint32_t txLength)
{
    if(txLength > CPPLIB_SPI_MAXLENGTH)
    {
        nrfSystem.registerError(LS_ERROR, "SPIM: Transmit", 0, "Transaction length exceeded");
        return;
    }
    if(mOpen)
    {
        while(mSpiActive) __WFE();
        mSpiActive = true;
        mBase->TXD.PTR      = (uint32_t)txBuf;
        mBase->TXD.MAXCNT   = txLength;
        mBase->RXD.PTR      = 0;
        mBase->RXD.MAXCNT   = 0;
        if(!mInPartialTrans) mCsnPin.low();
        mBase->TASKS_START  = 1;
        if(blocking) while(mSpiActive) __WFE();
    }
    else nrfSystem.registerError(LS_ERROR, "SPIM: Transmit", 0, "Interface not open");
}

void SpiMaster::receive(uint8_t *rxBuf, uint32_t rxLength)
{
    static uint8_t dummy = 0;
    if(rxLength > CPPLIB_SPI_MAXLENGTH)
    {
        nrfSystem.registerError(LS_ERROR, "SPIM: Receive", 0, "Transaction length exceeded");
        return;
    }   
    if(mOpen)
    {
        while(mSpiActive) __WFE();
        mSpiActive = true;
        mBase->TXD.PTR      = (uint32_t)&dummy;
        mBase->TXD.MAXCNT   = 1;
        mBase->RXD.PTR      = (uint32_t)rxBuf;
        mBase->RXD.MAXCNT   = rxLength;
        if(!mInPartialTrans) mCsnPin.low();
        mBase->TASKS_START  = 1;
        if(blocking) while(mSpiActive);
    }
    else nrfSystem.registerError(LS_ERROR, "SPIM: Receive", 0, "Interface not open");
}

void SpiMaster::chipSelect()
{
    NRF_GPIO->OUTCLR = (1 << pinCsn);   
    mCsnPin.low();
    mInPartialTrans = true;
}

void SpiMaster::chipDeselect()
{
    mCsnPin.high();
    mInPartialTrans = false;
}

uint8_t SpiMaster::put(uint8_t dataOut)
{
    static uint8_t txByte, rxByte;
    txByte = dataOut;
    if(mInPartialTrans)
    {
        while(mSpiActive);
        mSpiActive = true;
        mBase->TXD.PTR      = (uint32_t)&txByte;
        mBase->TXD.MAXCNT   = 1;
        mBase->RXD.PTR      = (uint32_t)&rxByte;
        mBase->RXD.MAXCNT   = 1;
        mBase->TASKS_START  = 1;
        while(mSpiActive);
        return rxByte;
    }
    return 0;
}
    
void SpiMaster::onIrqInstance(void)
{
    if(mBase->EVENTS_END)
    {
        mBase->EVENTS_END = 0;
        if(mInPartialTrans)
        {
            
        }
        else
        {
            mCsnPin.high();
        }
        mSpiActive = false;
        if(mCallback) mCallback(mBase->TXD.AMOUNT, mBase->RXD.AMOUNT);
    }
}

void SpiMaster::onIrq(uint32_t peripheralIndex)
{
    if(peripheralIndex < CPPLIB_BOARD_SPIM_INTERFACE_NUM && mActiveClassList[peripheralIndex])
    {
        mActiveClassList[peripheralIndex]->onIrqInstance();
    }
}

} // End of namespace CppLib

