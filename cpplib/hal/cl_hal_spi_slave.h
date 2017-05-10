#ifndef __ULIB_SPIS_H
#define __ULIB_SPIS_H

#include <stdint.h>
#include "nrf.h"
#include "boards//cl_board.h"
#include "hal//cl_hal_spi.h"

namespace CppLib {
    
typedef void (*SpisCallbackRxDataReceived)(uint8_t *txData, uint32_t txDataLength, uint8_t *rxData, uint32_t rxDataLength);

class SpiSlave
{
private:  
    volatile NRF_SPIS_Type *mBase;
    uint32_t mSpisPeripheralIndex;
    bool mOpen;
    bool mSpiActive;
    uint8_t mRxBuf[CPPLIB_BOARD_SPIS_RX_BUF_SIZE];

    static SpiSlave *mActiveClassList[CPPLIB_BOARD_SPIS_INTERFACE_NUM];

public:

    // Public variables
    uint32_t            pinSck;
    uint32_t            pinMosi;
    uint32_t            pinMiso;
    uint32_t            pinCsn;
    USpiClockPolarityT  clockPolarity;
    USpiClockPhaseT     clockPhase;
    USpiBitOrderT       bitOrder;
    SpisCallbackRxDataReceived  callbackRxDataReceived;

    SpiSlave();

    void open();
    void close();

    void setTxData(uint8_t *txData, uint32_t length);

    void onIrqInstance(void);
    static void onIrq(uint32_t peripheralIndex);
};

} // End of namespace CppLib

#endif
