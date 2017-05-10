#ifndef __ULIB_TWIM_H
#define __ULIB_TWIM_H

#include <stdint.h>
#include "nrf.h"
#include "boards//cl_board.h"
#include "hal//cl_hal_spi.h"

namespace CppLib {
    
typedef void (*TwimCallbackTransactionComplete)(uint8_t *txData, uint32_t txDataLength, uint8_t *rxData, uint32_t rxDataLength);
typedef void (*TwimCallbackError)(uint32_t errorMask);


class TwiMaster
{
private:  
    volatile NRF_TWIM_Type *mBase;
    uint32_t mTwimPeripheralIndex;
    volatile bool mOpen;
    volatile bool mTwimActive;
    uint32_t mErrorLast;

    static TwiMaster *mActiveClassList[CPPLIB_BOARD_TWIM_INTERFACE_NUM];

public:

    // Public variables
    uint32_t            pinSda;
    uint32_t            pinScl;
    uint32_t            address;
    TwimCallbackTransactionComplete  callbackTransactionComplete;
    TwimCallbackError                callbackError;

    TwiMaster();

    void open();
    void close();
    bool isOpen() { return mOpen; }

    void tx(uint8_t *txData, uint32_t txLength);
    void txRx(uint8_t *txData, uint32_t txLength, uint8_t *rxData, uint32_t rxLength);

    uint32_t completeOperation();

    void onIrqInstance(void);
    static void onIrq(uint32_t peripheralIndex);
};

}

#endif
