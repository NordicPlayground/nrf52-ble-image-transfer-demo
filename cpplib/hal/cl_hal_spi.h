#ifndef __ULIB_SPIM_H
#define __ULIB_SPIM_H

#include <stdint.h>
#include "nrf.h"
#include "boards//cl_board.h"
#include "hal//cl_hal_gpio.h"

namespace CppLib {

#define CPPLIB_SPI_MAXLENGTH 255
    
typedef enum {USPI_CLOCK_POLARITY_HIGH, USPI_CLOCK_POLARITY_LOW} USpiClockPolarityT;
typedef enum {USPI_CLOCK_PHASE_LEADING, USPI_CLOCK_PHASE_TRAILING} USpiClockPhaseT;
typedef enum {USPI_BIT_ORDER_MSB_LSB, USPI_BIT_ORDER_LSB_MSB} USpiBitOrderT;

typedef void (*SpiCallbackT)(uint32_t txBytesSent, uint32_t rxBytesReceived);

class SpiMaster
{
private:  
    volatile NRF_SPIM_Type *mBase;
    uint32_t mSpimPeripheralIndex;
    volatile bool mOpen;
    volatile bool mSpiActive;
    volatile bool mInPartialTrans;
    PinOut mCsnPin;
    SpiCallbackT mCallback;

    static SpiMaster *mActiveClassList[CPPLIB_BOARD_SPIM_INTERFACE_NUM];

public:

    // Public variables
    uint32_t            frequency;
    uint32_t            pinSck;
    uint32_t            pinMosi;
    uint32_t            pinMiso;
    uint32_t            pinCsn;
    USpiClockPolarityT  clockPolarity;
    USpiClockPhaseT     clockPhase;
    USpiBitOrderT       bitOrder;
    bool                blocking;

    SpiMaster();

    void open();
    void close();
    bool isOpen() { return mOpen; }

    // Transaction functions
    void transfer(uint8_t *txBuf, uint8_t *rxBuf, uint32_t length);
    void transmit(uint8_t *txBuf, uint32_t txLength);
    void receive(uint8_t *rxBuf, uint32_t rxLength);
    bool isBusy() { return mSpiActive; }

    // Partial functions
    void chipSelect();
    void chipDeselect();
    uint8_t put(uint8_t dataOut);
    
    void registerCallback(SpiCallbackT callback) { mCallback = callback; }
    // IRQ config functions
    void onIrqInstance(void);
    static void onIrq(uint32_t peripheralIndex);
};

} // End of namespace CppLib

#endif
