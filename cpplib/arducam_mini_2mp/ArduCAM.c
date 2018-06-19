/*
  ArduCAM.cpp - Arduino library support for CMOS Image Sensor
  Copyright (C)2011-2015 ArduCAM.com. All right reserved

  Basic functionality of this library are based on the demo-code provided by
  ArduCAM.com. You can find the latest version of the library at
  http://www.ArduCAM.com

  Now supported controllers:
    - OV7670
    - MT9D111
    - OV7675
    - OV2640
    - OV3640
    - OV5642
    - OV7660
    - OV7725
    - MT9M112
    - MT9V111
    - OV5640
    - MT9M001
    - MT9T112
    - MT9D112

  We will add support for many other sensors in next release.

  Supported MCU platform
    - Theoretically support all Arduino families
      - Arduino UNO R3      (Tested)
      - Arduino MEGA2560 R3   (Tested)
      - Arduino Leonardo R3   (Tested)
      - Arduino Nano      (Tested)
      - Arduino DUE       (Tested)
      - Arduino Yun       (Tested)
      - Raspberry Pi      (Tested)
      - ESP8266-12        (Tested)

  If you make any modifications or improvements to the code, I would appreciate
  that you share the code with me so that I might include it in the next release.
  I can be contacted through http://www.ArduCAM.com

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*------------------------------------
  Revision History:
  2012/09/20  V1.0.0  by Lee  first release
  2012/10/23  V1.0.1  by Lee  Resolved some timing issue for the Read/Write Register
  2012/11/29  V1.1.0  by Lee  Add support for MT9D111 sensor
  2012/12/13  V1.2.0  by Lee  Add support for OV7675 sensor
  2012/12/28  V1.3.0  by Lee  Add support for OV2640,OV3640,OV5642 sensors
  2013/02/26  V2.0.0  by Lee  New Rev.B shield hardware, add support for FIFO control
                              and support Mega1280/2560 boards
  2013/05/28  V2.1.0  by Lee  Add support all drawing functions derived from UTFT library
  2013/08/24  V3.0.0  by Lee  Support ArudCAM shield Rev.C hardware, features SPI interface and low power mode.
                Support almost all series of Arduino boards including DUE.
  2014/02/06  V3.0.1  by Lee  Minor change to the library, fixed some bugs, add self test code to the sketches for easy debugging.
  2014/03/09  V3.1.0  by Lee  Add the more impressive example sketches.
                Optimise the OV5642 settings, improve image quality.
                Add live preview before JPEG capture.
                Add play back photos one by one after BMP capture.
  2014/05/01  V3.1.1  by Lee  Minor changes to add support Arduino IDE for linux distributions.
  2014/09/30  V3.2.0  by Lee  Improvement on OV5642 camera dirver.
  2014/10/06  V3.3.0  by Lee  Add OV7660,OV7725 camera support.
  2015/02/27  V3.4.0  by Lee  Add the support for Arduino Yun board, update the latest UTFT library for ArduCAM.
  2015/06/09  V3.4.1  by Lee  Minor changes and add some comments
  2015/06/19  V3.4.2  by Lee  Add support for MT9M112 camera.
  2015/06/20  V3.4.3  by Lee  Add support for MT9V111 camera.
  2015/06/22  V3.4.4  by Lee  Add support for OV5640 camera.
  2015/06/22  V3.4.5  by Lee  Add support for MT9M001 camera.
  2015/08/05  V3.4.6  by Lee  Add support for MT9T112 camera.
  2015/08/08  V3.4.7  by Lee  Add support for MT9D112 camera.
  2015/09/20  V3.4.8  by Lee  Add support for ESP8266 processor.
  2016/02/03  V3.4.9  by Lee  Add support for Arduino ZERO board.
  2016/06/07  V3.5.0  by Lee  Add support for OV5642_CAM_BIT_ROTATION_FIXED.
  2016/06/14  V3.5.1  by Lee  Add support for ArduCAM-Mini-5MP-Plus OV5640_CAM.
  2016/09/29  V3.5.2  by Lee  Optimize the OV5642 register settings
	2016/10/05	V4.0.0	by Lee	Add support for second generation hardware platforms like ArduCAM shield V2, ArduCAM-Mini-5MP-Plus(OV5642/OV5640).	  
	2016/10/17	V4.0.1	by Lee	Add support for Arduino Genuino 101 board	
  --------------------------------------*/
//#include "Arduino.h"
#include "ArduCAM.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrfx_spim_t cam_spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE); 

#define TWIM_INSTANCE 1
static const nrfx_twim_t cam_twi = NRFX_TWIM_INSTANCE(TWIM_INSTANCE);

// "Protected"
regtype *P_CS;
regsize B_CS;
uint8_t m_fmt;
uint8_t sensor_model;
uint8_t sensor_addr; 
    
// Private
uint32_t pinCsn;
uint32_t pinMosi;
uint32_t pinMiso;
uint32_t pinSck;
uint32_t pinSda;
uint32_t pinScl;

uint32_t err_code;

static volatile bool spi_busy = false;
static volatile bool twim_busy = false;

static arducam_spi_callback_t m_spi_callback = 0;

//Assert CS signal
void arducam_CS_LOW(void)
{
    nrf_gpio_pin_clear(pinCsn);
}

//Disable CS signal
void arducam_CS_HIGH(void)
{
    nrf_gpio_pin_set(pinCsn);
}

uint8_t arducam_spiWrite(uint8_t dataByte)
{
    static uint8_t return_value;
    nrfx_spim_xfer_desc_t spi_transfer;
    spi_transfer.p_tx_buffer = &dataByte;
    spi_transfer.tx_length = 1;
    spi_transfer.p_rx_buffer = &return_value;
    spi_transfer.rx_length = 1;
    spi_busy = true;
    nrfx_spim_xfer(&cam_spi, &spi_transfer, 0);
    while(spi_busy);
    return return_value;
}

void arducam_spiReadMulti(uint8_t *rxBuf, uint32_t length)
{
    nrfx_spim_xfer_desc_t spi_transfer = {0};
    spi_transfer.p_rx_buffer = rxBuf;
    spi_transfer.rx_length = length;
    spi_busy = true;
    nrfx_spim_xfer(&cam_spi, &spi_transfer, 0);
    while(spi_busy); 
}

//Set corresponding bit
void arducam_set_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = arducam_read_reg(addr);
  arducam_write_reg(addr, temp | bit);
}

//Clear corresponding bit
void arducam_clear_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = arducam_read_reg(addr);
  arducam_write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
uint8_t arducam_get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = arducam_read_reg(addr);
  temp = temp & bit;
  return temp;
}

//Set ArduCAM working mode
//MCU2LCD_MODE: MCU writes the LCD screen GRAM
//CAM2LCD_MODE: Camera takes control of the LCD screen
//LCD2MCU_MODE: MCU read the LCD screen GRAM
void arducam_set_mode(uint8_t mode)
{
  switch (mode)
  {
    case MCU2LCD_MODE:
      arducam_write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
    case CAM2LCD_MODE:
      arducam_write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
      break;
    case LCD2MCU_MODE:
      arducam_write_reg(ARDUCHIP_MODE, LCD2MCU_MODE);
      break;
    default:
      arducam_write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
  }
}

//Low level SPI write operation
int arducam_bus_write(int address, int value) 
{
    static uint8_t txCommand[2];
    txCommand[0] = address;
    txCommand[1] = value;
    nrfx_spim_xfer_desc_t spi_transfer;
    spi_transfer.p_tx_buffer = txCommand;
    spi_transfer.tx_length = 2;
    spi_transfer.p_rx_buffer = 0;
    spi_transfer.rx_length = 0;
    spi_busy = true;
    arducam_CS_LOW();
        nrf_delay_us(20);
    err_code = nrfx_spim_xfer(&cam_spi, &spi_transfer, 0);
    while(spi_busy);
    arducam_CS_HIGH();
    nrf_delay_us(50);
    return 1;
}

//Low level SPI read operation
uint8_t arducam_bus_read(int address) 
{
    uint8_t txBuf[2] = {address, 0}, rxBuf[2];
    nrfx_spim_xfer_desc_t spi_transfer;
    spi_transfer.p_tx_buffer = txBuf;
    spi_transfer.tx_length = 2;
    spi_transfer.p_rx_buffer = rxBuf;
    spi_transfer.rx_length = 2;
    spi_busy = true;
    arducam_CS_LOW();
        nrf_delay_us(20);
    nrfx_spim_xfer(&cam_spi, &spi_transfer, 0);
    while(spi_busy);
    arducam_CS_HIGH();
        nrf_delay_us(50);
    return rxBuf[1];
}

//Write ArduChip internal registers
void arducam_write_reg(uint8_t addr, uint8_t data)
{
  arducam_bus_write(addr | 0x80, data);
}

//Read ArduChip internal registers
uint8_t arducam_read_reg(uint8_t addr)
{
  uint8_t data;
  data = arducam_bus_read(addr & 0x7F);
  return data;
}

void arducam_init(uint8_t model, uint32_t scl, uint32_t sda, uint32_t csn, uint32_t mosi, uint32_t miso, uint32_t sck)
{ 
    pinScl = scl;
    pinSda = sda;
    pinCsn = csn;
    pinMosi = mosi;
    pinMiso = miso;
    pinSck = sck;    
    arducam_spiTwiInit();
    sensor_model = model;
    switch (sensor_model)
    {
        case OV2640:
            sensor_addr = 0x60 >> 1;
            break;
        default:
            sensor_addr = 0x42;
            break;
    }
}
    
void arducam_spiRegisterCallback(arducam_spi_callback_t callback)
{
    m_spi_callback = callback;
}

static void spim_evt_handler(nrfx_spim_evt_t const * p_event, void *p)
{
    switch(p_event->type)
    {
        case NRFX_SPIM_EVENT_DONE:
            spi_busy = false;
            if(m_spi_callback)
            {
                m_spi_callback(p_event->xfer_desc.tx_length, p_event->xfer_desc.rx_length);
            }
            break;
            
        default:
            break;         
    }
}

static void twim_evt_handler(nrfx_twim_evt_t const * p_event, void *p)
{
    switch(p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            twim_busy = false;
            break;
            
        case NRFX_TWIM_EVT_DATA_NACK:
        case NRFX_TWIM_EVT_ADDRESS_NACK:
            break;
    }
    
}

void arducam_spiTwiInit()
{
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.ss_pin   = NRFX_SPIM_PIN_NOT_USED;
    spi_config.miso_pin = pinMiso;
    spi_config.mosi_pin = pinMosi;
    spi_config.sck_pin  = pinSck;
    spi_config.bit_order = 0;
    spi_config.mode = 0;
    spi_config.frequency = NRF_SPIM_FREQ_1M;
    nrf_gpio_cfg_output(pinCsn);
    nrf_gpio_pin_set(pinCsn);

    err_code = nrfx_spim_init(&cam_spi, &spi_config, spim_evt_handler, NULL);
  
    nrfx_twim_config_t twi_config = NRFX_TWIM_DEFAULT_CONFIG;
    twi_config.sda = pinSda;
    twi_config.scl = pinScl;
    err_code = nrfx_twim_init(&cam_twi, &twi_config, twim_evt_handler, NULL);
    nrfx_twim_enable(&cam_twi);
}

static void arducam_twi_tx_rx(uint8_t *tx_data, uint32_t tx_len, uint8_t *rx_data, uint32_t rx_len)
{
    while(twim_busy);
    nrfx_twim_xfer_desc_t twim_xfer_config;
    twim_xfer_config.address = sensor_addr;
    twim_xfer_config.p_primary_buf = tx_data;
    twim_xfer_config.primary_length = tx_len;
    twim_xfer_config.p_secondary_buf = rx_data;
    twim_xfer_config.secondary_length = rx_len;
    twim_xfer_config.type = (rx_len > 0) ? NRFX_TWIM_XFER_TXRX : NRFX_TWIM_XFER_TX;
    twim_busy = true;
    nrfx_twim_xfer(&cam_twi, &twim_xfer_config, (rx_len > 0) ? NRFX_TWIM_FLAG_TX_NO_STOP : 0);
}

void arducam_spiEnable(bool spiEnable)
{
//    if(mSpiMaster.isOpen() == spiEnable) return;
//    if(spiEnable)
//    {
//        mTwiMaster.close();
//        mSpiMaster.open();
//    }
//    else
//    {
//        mSpiMaster.close();
//        mTwiMaster.open();
//    }
}

//Reset the FIFO pointer to ZERO
void arducam_flush_fifo(void)
{
  arducam_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

//Send capture command
void arducam_start_capture(void)
{
  arducam_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

//Clear FIFO Complete flag
void arducam_clear_fifo_flag(void)
{
  arducam_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

//Read FIFO single
uint8_t arducam_read_fifo(void)
{
  uint8_t data;
  data = arducam_bus_read(SINGLE_FIFO_READ);
  return data;
}

//Read Write FIFO length
//Support ArduCAM Mini only
uint32_t arducam_read_fifo_length(void)
{
  uint32_t len1, len2, len3, length = 0;
  len1 = arducam_read_reg(FIFO_SIZE1);
  len2 = arducam_read_reg(FIFO_SIZE2);
  len3 = arducam_read_reg(FIFO_SIZE3) & 0x7f;
  length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
  return length;
}

//Send read fifo burst command
//Support ArduCAM Mini only
void arducam_set_fifo_burst()
{
    uint8_t cmd = BURST_FIFO_READ;
    arducam_spiWrite(cmd);
}

//I2C Write 8bit address, 8bit data
uint8_t arducam_wrSensorReg8_8(int regID, int regDat)
{
    static uint8_t twiBuf[2];
    twiBuf[0] = regID & 0x00FF;
    twiBuf[1] = regDat & 0x00FF;
    arducam_twi_tx_rx(twiBuf, 2, 0, 0);
    while(twim_busy);
    nrf_delay_ms(1);
    return (1);
}

//I2C Read 8bit address, 8bit data
uint8_t arducam_rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{
    arducam_twi_tx_rx(&regID, 1, regDat, 1);
    while(twim_busy);
    nrf_delay_ms(1);
    return (1);
}

//I2C Write 8bit address, 16bit data
uint8_t arducam_wrSensorReg8_16(int regID, int regDat)
{
    uint8_t txBuf[3] = {regID & 0x00FF, regDat >> 8, regDat & 0x00FF};
    arducam_twi_tx_rx(txBuf, 3, 0, 0);
    while(twim_busy);
    nrf_delay_ms(1);
    return (1);
}

//I2C Read 8bit address, 16bit data
uint8_t arducam_rdSensorReg8_16(uint8_t regID, uint16_t* regDat)
{
    uint8_t rxBuf[2];
    arducam_twi_tx_rx(regID, 1, rxBuf, 2);
    while(twim_busy);
    *regDat = rxBuf[0] << 8 | rxBuf[1];
    nrf_delay_ms(1);
    return (1);
}

//I2C Write 16bit address, 8bit data
uint8_t arducam_wrSensorReg16_8(int regID, int regDat)
{
    uint8_t txBuf[3] = {regID >> 8, regID & 0x00FF, regDat & 0x00FF};
    arducam_twi_tx_rx(txBuf, 3, 0, 0);
    while(twim_busy);    
    nrf_delay_ms(1);
    return (1);
}

//I2C Read 16bit address, 8bit data
uint8_t arducam_rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
    uint8_t txBuf[2] = {regID >> 8, regID & 0x00FF};
    arducam_twi_tx_rx(txBuf, 2, regDat, 1);
    while(twim_busy); 
    nrf_delay_ms(1);
    return (1);
}

//I2C Write 16bit address, 16bit data
uint8_t arducam_wrSensorReg16_16(int regID, int regDat)
{
    uint8_t txBuf[4] = {regID >> 8, regID & 0x00FF, regDat >> 8, regDat & 0x00FF};
    arducam_twi_tx_rx(txBuf, 4, 0, 0);
    while(twim_busy);
    nrf_delay_ms(1);
    return (1);
}

//I2C Read 16bit address, 16bit data
uint8_t arducam_rdSensorReg16_16(uint16_t regID, uint16_t* regDat)
{
    uint8_t txBuf[2] = {regID >> 8, regID & 0x00FF};
    uint8_t rxBuf[2];
    arducam_spiEnable(false);
    arducam_twi_tx_rx(txBuf, 2, rxBuf, 2);
    while(twim_busy);
    *regDat = rxBuf[0] << 8 | rxBuf[1];
    nrf_delay_ms(1);
    return (1);
}

//I2C Array Write 8bit address, 8bit data
int arducam_wrSensorRegs8_8(const struct sensor_reg reglist[])
{
  uint16_t reg_addr = 0;
  uint16_t reg_val = 0;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xff) | (reg_val != 0xff))
  {
    reg_addr = pgm_read_word(&next->reg);
    reg_val = pgm_read_word(&next->val);
    arducam_wrSensorReg8_8(reg_addr, reg_val);
    next++;
#if defined(ESP8266)
    yield();
#endif
  }

  return 1;
}

//I2C Array Write 8bit address, 16bit data
int arducam_wrSensorRegs8_16(const struct sensor_reg reglist[])
{
  unsigned int reg_addr =0, reg_val = 0;
  const struct sensor_reg *next = reglist;

  while ((reg_addr != 0xff) | (reg_val != 0xffff))
  {
    reg_addr = pgm_read_word(&next->reg);
    reg_val = pgm_read_word(&next->val);
    arducam_wrSensorReg8_16(reg_addr, reg_val);
    //  if (!err)
    //return err;
    next++;
#if defined(ESP8266)
    yield();
#endif
  }

  return 1;
}

//I2C Array Write 16bit address, 8bit data
int arducam_wrSensorRegs16_8(const struct sensor_reg reglist[])
{
  unsigned int reg_addr = 0;
  unsigned char reg_val = 0;
  const struct sensor_reg *next = reglist;

  while ((reg_addr != 0xffff) | (reg_val != 0xff))
  {
    reg_addr = pgm_read_word(&next->reg);
    reg_val = pgm_read_word(&next->val);
    arducam_wrSensorReg16_8(reg_addr, reg_val);
    //if (!err)
    //return err;
    next++;
#if defined(ESP8266)
    yield();
#endif
  }

  return 1;
}

//I2C Array Write 16bit address, 16bit data
int arducam_wrSensorRegs16_16(const struct sensor_reg reglist[])
{

  unsigned int reg_addr, reg_val;
  const struct sensor_reg *next = reglist;
  reg_addr = pgm_read_word(&next->reg);
  reg_val = pgm_read_word(&next->val);
  while ((reg_addr != 0xffff) | (reg_val != 0xffff))
  {
    arducam_wrSensorReg16_16(reg_addr, reg_val);
    //if (!err)
    //   return err;
    next++;
    reg_addr = pgm_read_word(&next->reg);
    reg_val = pgm_read_word(&next->val);
#if defined(ESP8266)
    yield();
#endif
  }


  return 1;
}

void arducam_OV2640_set_JPEG_size(uint8_t size)
{

  #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP))
  switch (size)
  {  
    case OV2640_160x120:
      arducam_wrSensorRegs8_8(OV2640_160x120_JPEG);
      break;
    case OV2640_176x144:
      arducam_wrSensorRegs8_8(OV2640_176x144_JPEG);
      break;
    case OV2640_320x240:
      arducam_wrSensorRegs8_8(OV2640_320x240_JPEG);
      break;
    case OV2640_352x288:
      arducam_wrSensorRegs8_8(OV2640_352x288_JPEG);
      break;
    case OV2640_640x480:
      arducam_wrSensorRegs8_8(OV2640_640x480_JPEG);
      break;
    case OV2640_800x600:
      arducam_wrSensorRegs8_8(OV2640_800x600_JPEG);
      break;
    case OV2640_1024x768:
      arducam_wrSensorRegs8_8(OV2640_1024x768_JPEG);
      break;
    case OV2640_1280x1024:
      arducam_wrSensorRegs8_8(OV2640_1280x1024_JPEG);
      break;
    case OV2640_1600x1200:
      arducam_wrSensorRegs8_8(OV2640_1600x1200_JPEG);
      break;
    default:
      arducam_wrSensorRegs8_8(OV2640_320x240_JPEG);
      break;
  }
#endif
}

void arducam_set_format(uint8_t fmt)
{
  if (fmt == BMP)
    m_fmt = BMP;
  else
    m_fmt = JPEG;
}

void arducam_InitCAM()
{
#if !(defined(__CPU_ARC__))
 #endif
  switch (sensor_model)
  {
    case OV2640:
      {
#if (defined(OV2640_CAM) || defined(OV2640_MINI_2MP))
        arducam_wrSensorReg8_8(0xff, 0x01);
        arducam_wrSensorReg8_8(0x12, 0x80);
        nrf_delay_ms(100);
        if (m_fmt == JPEG)
        {
          arducam_wrSensorRegs8_8(OV2640_JPEG_INIT);
          arducam_wrSensorRegs8_8(OV2640_YUV422);
          arducam_wrSensorRegs8_8(OV2640_JPEG);
          arducam_wrSensorReg8_8(0xff, 0x01);
          arducam_wrSensorReg8_8(0x15, 0x00);
          arducam_wrSensorRegs8_8(OV2640_320x240_JPEG);
          //wrSensorReg8_8(0xff, 0x00);
          //wrSensorReg8_8(0x44, 0x32);
        }
        else
        {
          arducam_wrSensorRegs8_8(OV2640_QVGA);
        }
#endif
        break;
      }
    default:

      break;
  }
}
