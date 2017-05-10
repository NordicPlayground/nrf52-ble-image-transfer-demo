#include "hal//cl_hal_spi_slave.h"
#include "system//cl_system.h"

namespace CppLib {
    
SpiSlave *SpiSlave::mActiveClassList[CPPLIB_BOARD_SPIS_INTERFACE_NUM] = {0, 0};

SpiSlave::SpiSlave()
{
    mOpen           = false;
    mSpiActive      = false;
    pinSck          = CPPLIB_BOARD_SPIS_PIN_SCK;
    pinMosi         = CPPLIB_BOARD_SPIS_PIN_MOSI;
    pinMiso         = CPPLIB_BOARD_SPIS_PIN_MISO;
    pinCsn          = CPPLIB_BOARD_SPIS_PIN_CSN;
    clockPolarity   = USPI_CLOCK_POLARITY_HIGH;
    clockPhase      = USPI_CLOCK_PHASE_LEADING;
    bitOrder        = USPI_BIT_ORDER_MSB_LSB;
    callbackRxDataReceived = 0;
}

void SpiSlave::open()
{
    if(mOpen) nrfSystem.registerError(ERROR, "SPIS: Open", 0, "Interface already open");
    else if((mBase = nrfSystem.allocSpis(&mSpisPeripheralIndex)) != 0)
    {
        mOpen = true;
        nrfSystem.registerSpisIrq(SpiSlave::onIrq);
        mActiveClassList[mSpisPeripheralIndex] = this;
        mBase->PSEL.SCK = pinSck;
        mBase->PSEL.MOSI = pinMosi;
        mBase->PSEL.MISO = pinMiso;
        mBase->PSEL.CSN = pinCsn;
        mBase->CONFIG = clockPhase      << SPIS_CONFIG_CPHA_Pos |
                        clockPolarity   << SPIS_CONFIG_CPOL_Pos | 
                        bitOrder        << SPIS_CONFIG_ORDER_Pos;
        mBase->SHORTS = SPIS_SHORTS_END_ACQUIRE_Msk;
        mBase->INTENSET = SPIS_INTENSET_END_Msk | SPIS_INTENSET_ACQUIRED_Msk;
        NVIC_SetPriority((IRQn_Type)ulibResourceSpisIrqN[mSpisPeripheralIndex], 7);
        NVIC_EnableIRQ((IRQn_Type)ulibResourceSpisIrqN[mSpisPeripheralIndex]);
        mBase->ENABLE = SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos;
        nrfSystem.registerError(DEBUG, "SPIS", 0, "Spi slave interface created");
    }
    else nrfSystem.registerError(ERROR, "SPIS: Open", 0, "No peripheral free");
}

void SpiSlave::close()
{
    if(mOpen)
    {
        NVIC_DisableIRQ((IRQn_Type)ulibResourceSpisIrqN[mSpisPeripheralIndex]);
        nrfSystem.deallocSpis(mBase);
        mActiveClassList[mSpisPeripheralIndex] = 0;
        mOpen = false;
    }
    nrfSystem.registerError(ERROR, "SPIS: Close", 0, "Interface not open");
}

void SpiSlave::setTxData(uint8_t *txData, uint32_t length)
{
    if(mOpen)
    {
        while(mSpiActive);
        mSpiActive = true;
        mBase->TXD.PTR = (uint32_t)txData;
        mBase->TXD.MAXCNT = length;
        mBase->RXD.PTR = (uint32_t)mRxBuf;
        mBase->RXD.MAXCNT = CPPLIB_BOARD_SPIS_RX_BUF_SIZE;
        mBase->TASKS_RELEASE = 1;
    }
    else nrfSystem.registerError(ERROR, "SPIS: setTxData", 0, "Interface not open");
}

void SpiSlave::onIrqInstance(void)
{
    if(mBase->EVENTS_END)
    {
        mBase->EVENTS_END = 0;
        
    }
    
    if(mBase->EVENTS_ACQUIRED)
    {
        mBase->EVENTS_ACQUIRED = 0;
        if(mSpiActive)
        {
            mSpiActive = false;
            if(callbackRxDataReceived)
            {
                callbackRxDataReceived(0, 0, (uint8_t*)mBase->RXD.PTR, mBase->RXD.AMOUNT);
            }
        }
    }
}

void SpiSlave::onIrq(uint32_t peripheralIndex)
{
    if(peripheralIndex < CPPLIB_BOARD_SPIS_INTERFACE_NUM && mActiveClassList[peripheralIndex])
    {
        mActiveClassList[peripheralIndex]->onIrqInstance();
    }
}

} // End of namespace CppLib
