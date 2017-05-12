#ifndef __APP_DISPLAY_H
#define __APP_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "ArduCAM.h"

typedef enum {ArducamFormatJpeg, ArducamFormatBmp} arducamFormatT;

class ArducamMini2MP
{
private:
    uint32_t    mBytesLeftInCamera;
    uint32_t    mAsyncBytesLeft;
    uint8_t    *mAsyncOutBuffer;
    
public:
    uint32_t pinCsn;
    uint32_t pinMosi;
    uint32_t pinMiso;
    uint32_t pinSck;
    uint32_t pinSda;
    uint32_t pinScl;

    ArducamMini2MP();

    void open();
    void setResolution(uint32_t res);
    void setFormat(arducamFormatT format);
    void startSingleCapture();
    uint32_t bytesAvailable();
    uint32_t fillBuffer(uint8_t *buffer, uint32_t bufferSize);

    uint32_t asyncFillBuffer(uint8_t *buffer, uint32_t bufferSize);
    void     onSpiInterrupt(uint32_t txBytes, uint32_t rxBytes);
    static ArducamMini2MP *activeInstance;
};

bool display_init(void);


#endif
