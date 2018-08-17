// ArduCAM Mini demo (C)2016 Lee
// web: http://www.ArduCAM.com
// This program is a demo of how to use most of the functions
// of the library with ArduCAM Mini camera, and can run on any Arduino platform.
// This demo was made for ArduCAM_Mini_5MP_Plus.
// It needs to be used in combination with PC software.
// It can take photo continuously as video streaming.
//
// The demo sketch will do the following tasks:
// 1. Set the camera to JEPG output mode.
// 2. Read data from Serial port and deal with it
// 3. If receive 0x00-0x08,the resolution will be changed.
// 4. If receive 0x10,camera will capture a JPEG photo and buffer the image to FIFO.Then write datas to Serial port.
// 5. If receive 0x20,camera will capture JPEG photo and write datas continuously.Stop when receive 0x21.
// 6. If receive 0x30,camera will capture a BMP  photo and buffer the image to FIFO.Then write datas to Serial port.
// 7. If receive 0x11 ,set camera to JPEG output mode.
// 8. If receive 0x31 ,set camera to BMP  output mode.
// This program requires the ArduCAM V4.0.0 (or later) library and ArduCAM_Mini_5MP_Plus
// and use Arduino IDE 1.5.2 compiler or above

#include "ArducamMini2MP.h"
//#include "ArduCAM.h"
#include "nrf_delay.h"
#include <stdio.h>

#define BMPIMAGEOFFSET 66

#define SPI_TRANSFER_MAX_LENGTH 240

// set pin 7 as the slave select for the digital pot:
bool is_header = false;
int mode = 0;
uint8_t start_capture = 0;

// Private:
static uint32_t    mBytesLeftInCamera;
static uint32_t    mAsyncBytesLeft;
static uint8_t    *mAsyncOutBuffer;
    
// public:

void ArducamMini2MP_init()
{
    mBytesLeftInCamera = 0;
    mAsyncBytesLeft = 0;
}

void spiCallback(uint32_t txBytesSent, uint32_t rxBytesReceived)
{
    arducam_mini_2mp_onSpiInterrupt(txBytesSent, rxBytesReceived);
}

void arducam_mini_2mp_open(arducam_mini_2mp_init_t *config)
{
        // put your setup code here, to run once:
    uint8_t vid, pid;
    uint8_t temp;
    arducam_init(OV2640, config->pinScl, config->pinSda, config->pinCsn, config->pinMosi, config->pinMiso, config->pinSck);

    //nrfSystem.registerError(LS_DEBUG, "ARDUCAM", 0, "Camera Start");
    
    arducam_spiRegisterCallback(spiCallback);
    
    //Check if the ArduCAM SPI bus is OK
    arducam_write_reg(ARDUCHIP_TEST1, 0x55);
    temp = arducam_read_reg(ARDUCHIP_TEST1);
    //Serial.println(temp);
    if (temp != 0x55)
    {
        //nrfSystem.registerError(LS_ERROR, "ARDUCAM", 0, "SPI interface non responsive");        
        while(1);
    }
    //Check if the camera module type is OV2640
    arducam_wrSensorReg8_8(0xff, 0x01);  
    arducam_rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    arducam_rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26) || (pid != 0x42))
    {
        //nrfSystem.registerError(LS_ERROR, "ARDUCAM", 0, "TWI interface non responsive");        
    }
    else
    {
        //nrfSystem.registerError(LS_DEBUG, "ARDUCAM", 0, "OV2640 module detected");        
    }

    arducam_set_format(JPEG);
    arducam_InitCAM();
    //arducam_OV2640_set_JPEG_size(OV2640_160x120);
    nrf_delay_ms(10);
    arducam_clear_fifo_flag();
}

void arducam_mini_2mp_setResolution(uint32_t res)
{
    if(res <= OV2640_1600x1200)
    {
        arducam_OV2640_set_JPEG_size(res);
        nrf_delay_ms(100);
    }    
}

void arducam_mini_2mp_setFormat(arducamFormatT format)
{
    if(format == ArducamFormatBmp) arducam_set_format(BMP);
    else arducam_set_format(JPEG);
    nrf_delay_ms(10);
}

void arducam_mini_2mp_startSingleCapture()
{
    if(mBytesLeftInCamera == 0)
    {
        while(0)
        {
            arducam_CS_LOW();
            nrf_delay_us(500);
            arducam_CS_HIGH();
            nrf_delay_us(500);   
        }
        arducam_flush_fifo();
        arducam_clear_fifo_flag();
        //Start capture
        arducam_start_capture();

        while(!arducam_get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
        
        mBytesLeftInCamera = arducam_read_fifo_length();// - 1;
        nrf_delay_us(500);
        arducam_CS_LOW();
        arducam_set_fifo_burst();
        //arducam_spiWrite(0);
    }
    else
    {
        //nrfSystem.registerError(LS_ERROR, "ARDUCAM: startSingleCapture()", 0, "Last capture not read out!");
    }
}
uint32_t arducam_mini_2mp_bytesAvailable()
{
    return mBytesLeftInCamera;
}

uint32_t arducam_mini_2mp_asyncFillBuffer(uint8_t *buffer, uint32_t bufferSize)
{
    while(mAsyncBytesLeft > 0);
    if(mBytesLeftInCamera == 0 || buffer == 0 || bufferSize == 0) return 0;    

    
    mAsyncBytesLeft = (mBytesLeftInCamera > bufferSize ? bufferSize : mBytesLeftInCamera);
    //mBytesLeftInCamera -= mAsyncBytesLeft;
    mAsyncOutBuffer = buffer;
    arducam_spiReadMulti(mAsyncOutBuffer, (mAsyncBytesLeft > SPI_TRANSFER_MAX_LENGTH ? SPI_TRANSFER_MAX_LENGTH : mAsyncBytesLeft));
    mAsyncOutBuffer += (mAsyncBytesLeft > SPI_TRANSFER_MAX_LENGTH ? SPI_TRANSFER_MAX_LENGTH : mAsyncBytesLeft);
    return mAsyncBytesLeft;
}

void arducam_mini_2mp_onSpiInterrupt(uint32_t txBytes, uint32_t rxBytes)
{
    if(mAsyncBytesLeft > 0)
    {
        if(rxBytes > mAsyncBytesLeft);// nrfSystem.registerError(LS_ERROR, "ArducamMini2MP", 0, "Internal error!");
        mAsyncBytesLeft -= rxBytes;
        mBytesLeftInCamera -= rxBytes;
        if(mAsyncBytesLeft > 0)
        {
            arducam_spiReadMulti(mAsyncOutBuffer, (mAsyncBytesLeft > SPI_TRANSFER_MAX_LENGTH ? SPI_TRANSFER_MAX_LENGTH : mAsyncBytesLeft));
            mAsyncOutBuffer += (mAsyncBytesLeft > SPI_TRANSFER_MAX_LENGTH ? SPI_TRANSFER_MAX_LENGTH : mAsyncBytesLeft);
        }
        if(mBytesLeftInCamera == 0)
        {
            arducam_CS_HIGH();
        }
    }
}

uint32_t arducam_mini_2mp_fillBuffer(uint8_t *buffer, uint32_t bufferSize)
{
    if(mBytesLeftInCamera == 0) return 0;
    if(mAsyncBytesLeft > 0) return 0;
    
    uint32_t bytesToRead = (mBytesLeftInCamera > bufferSize ? bufferSize : mBytesLeftInCamera);
    mBytesLeftInCamera -= bytesToRead;
    int transactionLength;
    for(int i = 0; i < bytesToRead; i += transactionLength)
    {
        transactionLength = (bytesToRead - i) > SPI_TRANSFER_MAX_LENGTH ? SPI_TRANSFER_MAX_LENGTH : (bytesToRead - i);
        arducam_spiReadMulti(buffer, transactionLength);
        
        buffer += transactionLength;
    }
    
    if(mBytesLeftInCamera == 0)
    {
        arducam_CS_HIGH();
    }
    return bytesToRead;
}
