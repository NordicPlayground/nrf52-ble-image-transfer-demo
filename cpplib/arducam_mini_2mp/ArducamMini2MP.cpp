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
#include "ArduCAM.h"
#include "nrf_delay.h"
#include "system//cl_system.h"
#include <stdio.h>

using namespace CppLib;

//This demo can only work on OV2640_MINI_2MP or OV5642_MINI_5MP or OV5642_MINI_5MP_BIT_ROTATION_FIXED
//or OV5640_MINI_5MP_PLUS or ARDUCAM_SHIELD_V2 platform.
#if !(defined OV5642_MINI_5MP || defined OV5642_MINI_5MP_BIT_ROTATION_FIXED || defined OV2640_MINI_2MP)
  #error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define BMPIMAGEOFFSET 66

// set pin 7 as the slave select for the digital pot:
bool is_header = false;
int mode = 0;
uint8_t start_capture = 0;

ArduCAM *myCAM;

ArducamMini2MP *ArducamMini2MP::activeInstance = 0;
  
ArducamMini2MP::ArducamMini2MP()
{
    mBytesLeftInCamera = 0;
    mAsyncBytesLeft = 0;
    activeInstance = this;
}

void spiCallback(uint32_t txBytesSent, uint32_t rxBytesReceived)
{
    ArducamMini2MP::activeInstance->onSpiInterrupt(txBytesSent, rxBytesReceived);
}

void ArducamMini2MP::open()
{
        // put your setup code here, to run once:
    uint8_t vid, pid;
    uint8_t temp;
    myCAM = new ArduCAM(OV2640, pinScl, pinSda, pinCsn, pinMosi, pinMiso, pinSck);

    nrfSystem.registerError(LS_DEBUG, "ARDUCAM", 0, "Camera Start");
    
    myCAM->spiRegisterCallback(spiCallback);
    //Check if the ArduCAM SPI bus is OK
    myCAM->write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM->read_reg(ARDUCHIP_TEST1);
    //Serial.println(temp);
    if (temp != 0x55)
    {
        nrfSystem.registerError(LS_ERROR, "ARDUCAM", 0, "SPI interface non responsive");        
        while(1);
    }
    //Check if the camera module type is OV2640
    myCAM->wrSensorReg8_8(0xff, 0x01);  
    myCAM->rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM->rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26) || (pid != 0x42))
        nrfSystem.registerError(LS_ERROR, "ARDUCAM", 0, "TWI interface non responsive");        
    else
        nrfSystem.registerError(LS_DEBUG, "ARDUCAM", 0, "OV2640 module detected");        

    myCAM->set_format(JPEG);
    myCAM->InitCAM();
    //myCAM->OV2640_set_JPEG_size(OV2640_160x120);
    nrf_delay_ms(10);
    myCAM->clear_fifo_flag();
}

void ArducamMini2MP::setResolution(uint32_t res)
{
    if(res <= OV2640_1600x1200)
    {
        myCAM->OV2640_set_JPEG_size(res);
        nrf_delay_ms(100);
    }    
}

void ArducamMini2MP::setFormat(arducamFormatT format)
{
    if(format == ArducamFormatBmp) myCAM->set_format(BMP);
    else myCAM->set_format(JPEG);
    nrf_delay_ms(10);
}

void ArducamMini2MP::startSingleCapture()
{
    if(mBytesLeftInCamera == 0)
    {
        myCAM->flush_fifo();
        myCAM->clear_fifo_flag();
        //Start capture
        myCAM->start_capture();

        while(!myCAM->get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
        
        mBytesLeftInCamera = myCAM->read_fifo_length();// - 1;
        myCAM->CS_LOW();
        myCAM->set_fifo_burst();
        //myCAM->spiWrite(0);
    }
    else nrfSystem.registerError(LS_ERROR, "ARDUCAM: startSingleCapture()", 0, "Last capture not read out!");
}
uint32_t ArducamMini2MP::bytesAvailable()
{
    return mBytesLeftInCamera;
}

uint32_t ArducamMini2MP::asyncFillBuffer(uint8_t *buffer, uint32_t bufferSize)
{
    while(mAsyncBytesLeft > 0);
    if(mBytesLeftInCamera == 0 || buffer == 0 || bufferSize == 0) return 0;    

    
    mAsyncBytesLeft = (mBytesLeftInCamera > bufferSize ? bufferSize : mBytesLeftInCamera);
    //mBytesLeftInCamera -= mAsyncBytesLeft;
    mAsyncOutBuffer = buffer;
    myCAM->spiReadMulti(mAsyncOutBuffer, (mAsyncBytesLeft > 255 ? 255 : mAsyncBytesLeft));
    mAsyncOutBuffer += (mAsyncBytesLeft > 255 ? 255 : mAsyncBytesLeft);
    return mAsyncBytesLeft;
}

void ArducamMini2MP::onSpiInterrupt(uint32_t txBytes, uint32_t rxBytes)
{
    if(mAsyncBytesLeft > 0)
    {
        if(rxBytes > mAsyncBytesLeft) nrfSystem.registerError(LS_ERROR, "ArducamMini2MP", 0, "Internal error!");
        mAsyncBytesLeft -= rxBytes;
        mBytesLeftInCamera -= rxBytes;
        if(mAsyncBytesLeft > 0)
        {
            myCAM->spiReadMulti(mAsyncOutBuffer, (mAsyncBytesLeft > 255 ? 255 : mAsyncBytesLeft));
            mAsyncOutBuffer += (mAsyncBytesLeft > 255 ? 255 : mAsyncBytesLeft);
        }
        if(mBytesLeftInCamera == 0)
        {
            myCAM->CS_HIGH();
        }
    }
}

uint32_t ArducamMini2MP::fillBuffer(uint8_t *buffer, uint32_t bufferSize)
{
    if(mBytesLeftInCamera == 0) return 0;
    if(mAsyncBytesLeft > 0) return 0;
    
    uint32_t bytesToRead = (mBytesLeftInCamera > bufferSize ? bufferSize : mBytesLeftInCamera);
    mBytesLeftInCamera -= bytesToRead;
    int transactionLength;
    for(int i = 0; i < bytesToRead; i += transactionLength)
    {
        transactionLength = (bytesToRead - i) > 255 ? 255 : (bytesToRead - i);
        myCAM->spiReadMulti(buffer, transactionLength);
        myCAM->spiFinalize();
        buffer += transactionLength;
        //nrf_delay_us(2);
    }
    
    if(mBytesLeftInCamera == 0)
    {
        myCAM->CS_HIGH();
    }
    return bytesToRead;
}
