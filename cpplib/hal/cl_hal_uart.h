#ifndef __ULIB_UART_H
#define __ULIB_UART_H

#include <stdint.h>
#include "nrf.h"
#include "util//ringbuffer.h"
#include "system//cl_system.h"
#include "boards//cl_board.h"

namespace CppLib {
    
typedef void (*UartCallbackRxDataReceived)(uint8_t byte);

class Uart
{
private:
    volatile NRF_UARTE_Type *mBase;
    uint32_t mUartPeripheralIndex;
    bool mOpen;
    bool mTxActive;
    uint8_t mTmpRxBuf;
    UartCallbackRxDataReceived mCallbackRxDataReceived;
    
    RingBuffer<uint8_t, 64> mFifoRx;
    RingBuffer<uint8_t, 256> mFifoTx;

    void initiateTx();

    static Uart *mActiveClassList[CPPLIB_BOARD_UART_INTERFACE_NUM];

    static Uart *mLogUart;
    static void uartLogHandler(LogSeverity severity, char *module, uint32_t errorCode, char *message);

public:  
    uint32_t pinTx, pinRx, pinCts, pinRts;
    uint32_t baudrate;
    uint32_t parity;
    uint32_t flowControl;
    
    Uart();

    void setCallbackRxDataReceived(UartCallbackRxDataReceived callback) { mCallbackRxDataReceived = callback; }
    LogMessageHandler getDefaultLogHandler();
    
    void open();
    void close();
    
    uint32_t bytesAvailable() { return mFifoRx.count(); }

    Uart &put(char valueByte);
    Uart &put(uint8_t valueByte);
    Uart &put(const char *string);
    Uart &put(uint8_t *valueBuf, uint32_t bufLength);
    Uart &put(float valueFloat);
    Uart &newline() { return put("\r\n"); }
    uint8_t get();
    
    void onIrqInstance(void);
    static void onIrq(uint32_t peripheralIndex);
};

} // End of namespace CppLib

#endif
