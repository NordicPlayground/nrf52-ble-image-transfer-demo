#include "hal//cl_hal_uart.h"
#include "boards//cl_board.h"
#include "system//cl_system.h"

namespace CppLib {
    
Uart *Uart::mActiveClassList[CPPLIB_BOARD_UART_INTERFACE_NUM] = {0};
Uart *Uart::mLogUart = 0;

Uart::Uart()
{
    mOpen  = false;
    mTxActive = false;
    pinRx  = CPPLIB_BOARD_UART_PIN_RX;
    pinTx  = CPPLIB_BOARD_UART_PIN_TX;
    pinRts = CPPLIB_BOARD_UART_PIN_RTS;
    pinCts = CPPLIB_BOARD_UART_PIN_CTS;
    baudrate = 9600;
    parity = false;
    flowControl = false;
}

void Uart::open()
{
    if((mBase = nrfSystem.allocUartE(&mUartPeripheralIndex)) != 0)
    {
        mOpen = true;
        nrfSystem.registerUartIrq(Uart::onIrq);
        mActiveClassList[0] = this;
        nrfSystem.registerGpio(mBase->PSEL.RXD = pinRx);
        nrfSystem.registerGpio(mBase->PSEL.TXD = pinTx);
        if(0)
        {
            nrfSystem.registerGpio(mBase->PSEL.RTS = pinRts);
            nrfSystem.registerGpio(mBase->PSEL.CTS = pinCts);
        }
        mBase->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud115200;
        mBase->CONFIG = UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos | UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos;
        mBase->RXD.PTR = (uint32_t)&mTmpRxBuf;
        mBase->RXD.MAXCNT = 1;
        mBase->INTENSET = UARTE_INTENSET_ENDTX_Msk | UARTE_INTENSET_ENDRX_Msk;
        NVIC_SetPriority((IRQn_Type)ulibResourceUarteIrqN[mUartPeripheralIndex], 6);
        NVIC_EnableIRQ((IRQn_Type)ulibResourceUarteIrqN[mUartPeripheralIndex]);
        mBase->ENABLE = UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos;
        mBase->TASKS_STARTRX = 1;
        nrfSystem.registerError(DEBUG, "UART", 0, "Uart Opened");
    }
    else nrfSystem.registerError(ERROR, "UART: Open", 0, "No peripheral free");
}

void Uart::close()
{
    if(mOpen)
    {
        NVIC_DisableIRQ((IRQn_Type)ulibResourceUarteIrqN[mUartPeripheralIndex]);
        nrfSystem.deallocUartE(mBase);
        mActiveClassList[mUartPeripheralIndex] = 0;
        nrfSystem.freeGpio(mBase->PSEL.TXD);
        nrfSystem.freeGpio(mBase->PSEL.RXD);
        if(0)
        {
            nrfSystem.freeGpio(mBase->PSEL.RTS);
            nrfSystem.freeGpio(mBase->PSEL.CTS);
        }
        mOpen = false;
    }
    nrfSystem.registerError(ERROR, "SPIM: Close", 0, "Interface not open");    
}

void Uart::initiateTx()
{
    if(mOpen && !mTxActive && !mFifoTx.isEmpty())
    {
        mTxActive = true;
        mBase->TXD.PTR = (uint32_t)mFifoTx.outPtr();
        if(mFifoTx.count() < (mFifoTx.getSize() - mFifoTx.getTail()))
            mBase->TXD.MAXCNT = mFifoTx.count();
        else
            mBase->TXD.MAXCNT = mFifoTx.getSize() - mFifoTx.getTail();
        mBase->TASKS_STARTTX = 1;
    }
}

Uart &Uart::put(char valueByte)
{
    put((uint8_t)valueByte);
    return *this;
}

Uart &Uart::put(uint8_t valueByte)
{
    while(mFifoTx.isFull())__WFE();
    mFifoTx.put(&valueByte);
    initiateTx();
    return *this;
}

Uart &Uart::put(const char *string)
{
    while(*string) put(*string++);
    return *this;
}

Uart &Uart::put(uint8_t *valueBuf, uint32_t bufLength)
{
    while(bufLength--) put(*valueBuf++);
    return *this;
}

Uart &Uart::put(float valueFloat)
{
    return *this;
}

uint8_t Uart::get()
{
    static uint8_t getByte;
    if(!mFifoRx.isEmpty()) mFifoRx.get(&getByte);
    return getByte;
}

void Uart::onIrqInstance(void)
{
    uint8_t dummy;
    if(mBase->EVENTS_ENDTX)
    {
        mBase->EVENTS_ENDTX = 0;
        mTxActive = false;
        for(int i = 0; i < mBase->TXD.AMOUNT; i++) mFifoTx.get(&dummy);
        if(mFifoTx.isEmpty()) mFifoTx.clear();
        initiateTx();
    }
    if(mBase->EVENTS_ENDRX)
    {
        mBase->EVENTS_ENDRX = 0;
        mFifoRx.put((uint8_t*)mBase->RXD.PTR);
        mBase->TASKS_STARTRX = 1;
    }
    
}

void Uart::onIrq(uint32_t peripheralIndex)
{
    if(peripheralIndex < CPPLIB_BOARD_UART_INTERFACE_NUM && mActiveClassList[peripheralIndex])
    {
        mActiveClassList[peripheralIndex]->onIrqInstance();
    }
}

LogMessageHandler Uart::getDefaultLogHandler()
{
    mLogUart = this;
    return uartLogHandler;
}

void Uart::uartLogHandler(ulibLogSeverity severity, char *module, uint32_t errorCode, char *message)
{
    mLogUart->put("\r\nLOG OUT - ").put(ulibLogSeverityStrings[severity]).put(": ").put(module).put(": ").put(message).put("\r\n");
}

} // End of namespace CppLib
