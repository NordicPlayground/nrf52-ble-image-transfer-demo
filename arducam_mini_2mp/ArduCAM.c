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
    err_code = nrfx_spim_xfer(&cam_spi, &spi_transfer, 0);
    while(spi_busy);
    arducam_CS_HIGH();
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
    nrfx_spim_xfer(&cam_spi, &spi_transfer, 0);
    while(spi_busy);
    arducam_CS_HIGH();
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
    spi_config.frequency = NRF_SPIM_FREQ_4M;
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
    arducam_twi_tx_rx(&regID, 1, rxBuf, 2);
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

const struct sensor_reg OV2640_QVGA[] =
{
	{0xff, 0x0}, 
	{0x2c, 0xff}, 
	{0x2e, 0xdf}, 
	{0xff, 0x1}, 
	{0x3c, 0x32}, 
	{0x11, 0x0}, 
	{0x9, 0x2}, 
	{0x4, 0xa8}, 
	{0x13, 0xe5}, 
	{0x14, 0x48}, 
	{0x2c, 0xc}, 
	{0x33, 0x78}, 
	{0x3a, 0x33}, 
	{0x3b, 0xfb}, 
	{0x3e, 0x0}, 
	{0x43, 0x11}, 
	{0x16, 0x10}, 
	{0x39, 0x2}, 
	{0x35, 0x88}, 

	{0x22, 0xa}, 
	{0x37, 0x40}, 
	{0x23, 0x0}, 
	{0x34, 0xa0}, 
	{0x6, 0x2}, 
	{0x6, 0x88}, 
	{0x7, 0xc0}, 
	{0xd, 0xb7}, 
	{0xe, 0x1}, 
	{0x4c, 0x0}, 
	{0x4a, 0x81}, 
	{0x21, 0x99}, 
	{0x24, 0x40}, 
	{0x25, 0x38}, 
	{0x26, 0x82}, 
	{0x5c, 0x0}, 
	{0x63, 0x0}, 
	{0x46, 0x22}, 
	{0xc, 0x3a}, 
	{0x5d, 0x55}, 
	{0x5e, 0x7d}, 
	{0x5f, 0x7d}, 
	{0x60, 0x55}, 
	{0x61, 0x70}, 
	{0x62, 0x80}, 
	{0x7c, 0x5}, 
	{0x20, 0x80}, 
	{0x28, 0x30}, 
	{0x6c, 0x0}, 
	{0x6d, 0x80}, 
	{0x6e, 0x0}, 
	{0x70, 0x2}, 
	{0x71, 0x94}, 
	{0x73, 0xc1}, 
	{0x3d, 0x34}, 
	{0x12, 0x4}, 
	{0x5a, 0x57}, 
	{0x4f, 0xbb}, 
	{0x50, 0x9c}, 
	{0xff, 0x0}, 
	{0xe5, 0x7f}, 
	{0xf9, 0xc0}, 
	{0x41, 0x24}, 
	{0xe0, 0x14}, 
	{0x76, 0xff}, 
	{0x33, 0xa0}, 
	{0x42, 0x20}, 
	{0x43, 0x18}, 
	{0x4c, 0x0}, 
	{0x87, 0xd0}, 
	{0x88, 0x3f}, 
	{0xd7, 0x3}, 
	{0xd9, 0x10}, 
	{0xd3, 0x82}, 
	{0xc8, 0x8}, 
	{0xc9, 0x80}, 
	{0x7c, 0x0}, 
	{0x7d, 0x0}, 
	{0x7c, 0x3}, 
	{0x7d, 0x48}, 
	{0x7d, 0x48}, 
	{0x7c, 0x8}, 
	{0x7d, 0x20}, 
	{0x7d, 0x10}, 
	{0x7d, 0xe}, 
	{0x90, 0x0}, 
	{0x91, 0xe}, 
	{0x91, 0x1a}, 
	{0x91, 0x31}, 
	{0x91, 0x5a}, 
	{0x91, 0x69}, 
	{0x91, 0x75}, 
	{0x91, 0x7e}, 
	{0x91, 0x88}, 
	{0x91, 0x8f}, 
	{0x91, 0x96}, 
	{0x91, 0xa3}, 
	{0x91, 0xaf}, 
	{0x91, 0xc4}, 
	{0x91, 0xd7}, 
	{0x91, 0xe8}, 
	{0x91, 0x20}, 
	{0x92, 0x0}, 

	{0x93, 0x6}, 
	{0x93, 0xe3}, 
	{0x93, 0x3}, 
	{0x93, 0x3}, 
	{0x93, 0x0}, 
	{0x93, 0x2}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x96, 0x0}, 
	{0x97, 0x8}, 
	{0x97, 0x19}, 
	{0x97, 0x2}, 
	{0x97, 0xc}, 
	{0x97, 0x24}, 
	{0x97, 0x30}, 
	{0x97, 0x28}, 
	{0x97, 0x26}, 
	{0x97, 0x2}, 
	{0x97, 0x98}, 
	{0x97, 0x80}, 
	{0x97, 0x0}, 
	{0x97, 0x0}, 
	{0xa4, 0x0}, 
	{0xa8, 0x0}, 
	{0xc5, 0x11}, 
	{0xc6, 0x51}, 
	{0xbf, 0x80}, 
	{0xc7, 0x10}, 
	{0xb6, 0x66}, 
	{0xb8, 0xa5}, 
	{0xb7, 0x64}, 
	{0xb9, 0x7c}, 
	{0xb3, 0xaf}, 
	{0xb4, 0x97}, 
	{0xb5, 0xff}, 
	{0xb0, 0xc5}, 
	{0xb1, 0x94}, 
	{0xb2, 0xf}, 
	{0xc4, 0x5c}, 
	{0xa6, 0x0}, 
	{0xa7, 0x20}, 
	{0xa7, 0xd8}, 
	{0xa7, 0x1b}, 
	{0xa7, 0x31}, 
	{0xa7, 0x0}, 
	{0xa7, 0x18}, 
	{0xa7, 0x20}, 
	{0xa7, 0xd8}, 
	{0xa7, 0x19}, 
	{0xa7, 0x31}, 
	{0xa7, 0x0}, 
	{0xa7, 0x18}, 
	{0xa7, 0x20}, 
	{0xa7, 0xd8}, 
	{0xa7, 0x19}, 
	{0xa7, 0x31}, 
	{0xa7, 0x0}, 
	{0xa7, 0x18}, 
	{0x7f, 0x0}, 
	{0xe5, 0x1f}, 
	{0xe1, 0x77}, 
	{0xdd, 0x7f}, 
	{0xc2, 0xe}, 
	
	{0xff, 0x0}, 
	{0xe0, 0x4}, 
	{0xc0, 0xc8}, 
	{0xc1, 0x96}, 
	{0x86, 0x3d}, 
	{0x51, 0x90}, 
	{0x52, 0x2c}, 
	{0x53, 0x0}, 
	{0x54, 0x0}, 
	{0x55, 0x88}, 
	{0x57, 0x0}, 
	
	{0x50, 0x92}, 
	{0x5a, 0x50}, 
	{0x5b, 0x3c}, 
	{0x5c, 0x0}, 
	{0xd3, 0x4}, 
	{0xe0, 0x0}, 
	
	{0xff, 0x0}, 
	{0x5, 0x0}, 
	
	{0xda, 0x0}, //{0xda, 0x8}, 
	{0xd7, 0x3}, 
	{0xe0, 0x0}, 
	
	{0x5, 0x0}, 

	
	{0xff,0xff},
};        

const struct sensor_reg OV2640_JPEG_INIT[]  =
{
  { 0xff, 0x00 },
  { 0x2c, 0xff },
  { 0x2e, 0xdf },
  { 0xff, 0x01 },
  { 0x3c, 0x32 },
  { 0x11, 0x00 },	
  { 0x09, 0x02 },
  { 0x04, 0x28 },
  { 0x13, 0xe5 },
  { 0x14, 0x48 },
  { 0x2c, 0x0c },
  { 0x33, 0x78 },
  { 0x3a, 0x33 },
  { 0x3b, 0xfB },
  { 0x3e, 0x00 },
  { 0x43, 0x11 },
  { 0x16, 0x10 },
  { 0x39, 0x92 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0x48, 0x00 },
  { 0x5B, 0x00 },
  { 0x42, 0x03 },
  { 0x4a, 0x81 },
  { 0x21, 0x99 },
  { 0x24, 0x40 },
  { 0x25, 0x38 },
  { 0x26, 0x82 },
  { 0x5c, 0x00 },
  { 0x63, 0x00 },
  { 0x61, 0x70 },
  { 0x62, 0x80 },
  { 0x7c, 0x05 },
  { 0x20, 0x80 },
  { 0x28, 0x30 },
  { 0x6c, 0x00 },
  { 0x6d, 0x80 },
  { 0x6e, 0x00 },
  { 0x70, 0x02 },
  { 0x71, 0x94 },
  { 0x73, 0xc1 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x37, 0xc0 },
  { 0x4f, 0x60 },
  { 0x50, 0xa8 },
  { 0x6d, 0x00 },
  { 0x3d, 0x38 },
  { 0x46, 0x3f },
  { 0x4f, 0x60 },
  { 0x0c, 0x3c },
  { 0xff, 0x00 },
  { 0xe5, 0x7f },
  { 0xf9, 0xc0 },
  { 0x41, 0x24 },
  { 0xe0, 0x14 },
  { 0x76, 0xff },
  { 0x33, 0xa0 },
  { 0x42, 0x20 },
  { 0x43, 0x18 },
  { 0x4c, 0x00 },
  { 0x87, 0xd5 },
  { 0x88, 0x3f },
  { 0xd7, 0x03 },
  { 0xd9, 0x10 },
  { 0xd3, 0x82 },
  { 0xc8, 0x08 },
  { 0xc9, 0x80 },
  { 0x7c, 0x00 },
  { 0x7d, 0x00 },
  { 0x7c, 0x03 },
  { 0x7d, 0x48 },
  { 0x7d, 0x48 },
  { 0x7c, 0x08 },
  { 0x7d, 0x20 },
  { 0x7d, 0x10 },
  { 0x7d, 0x0e },
  { 0x90, 0x00 },
  { 0x91, 0x0e },
  { 0x91, 0x1a },
  { 0x91, 0x31 },
  { 0x91, 0x5a },
  { 0x91, 0x69 },
  { 0x91, 0x75 },
  { 0x91, 0x7e },
  { 0x91, 0x88 },
  { 0x91, 0x8f },
  { 0x91, 0x96 },
  { 0x91, 0xa3 },
  { 0x91, 0xaf },
  { 0x91, 0xc4 },
  { 0x91, 0xd7 },
  { 0x91, 0xe8 },
  { 0x91, 0x20 },
  { 0x92, 0x00 },
  { 0x93, 0x06 },
  { 0x93, 0xe3 },
  { 0x93, 0x05 },
  { 0x93, 0x05 },
  { 0x93, 0x00 },
  { 0x93, 0x04 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x96, 0x00 },
  { 0x97, 0x08 },
  { 0x97, 0x19 },
  { 0x97, 0x02 },
  { 0x97, 0x0c },
  { 0x97, 0x24 },
  { 0x97, 0x30 },
  { 0x97, 0x28 },
  { 0x97, 0x26 },
  { 0x97, 0x02 },
  { 0x97, 0x98 },
  { 0x97, 0x80 },
  { 0x97, 0x00 },
  { 0x97, 0x00 },
  { 0xc3, 0xed },
  { 0xa4, 0x00 },
  { 0xa8, 0x00 },
  { 0xc5, 0x11 },
  { 0xc6, 0x51 },
  { 0xbf, 0x80 },
  { 0xc7, 0x10 },
  { 0xb6, 0x66 },
  { 0xb8, 0xA5 },
  { 0xb7, 0x64 },
  { 0xb9, 0x7C },
  { 0xb3, 0xaf },
  { 0xb4, 0x97 },
  { 0xb5, 0xFF },
  { 0xb0, 0xC5 },
  { 0xb1, 0x94 },
  { 0xb2, 0x0f },
  { 0xc4, 0x5c },
  { 0xc0, 0x64 },
  { 0xc1, 0x4B },
  { 0x8c, 0x00 },
  { 0x86, 0x3D },
  { 0x50, 0x00 },
  { 0x51, 0xC8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x5a, 0xC8 },
  { 0x5b, 0x96 },
  { 0x5c, 0x00 },
  { 0xd3, 0x00 },	//{ 0xd3, 0x7f },
  { 0xc3, 0xed },
  { 0x7f, 0x00 },
  { 0xda, 0x00 },
  { 0xe5, 0x1f },
  { 0xe1, 0x67 },
  { 0xe0, 0x00 },
  { 0xdd, 0x7f },
  { 0x05, 0x00 },
               
  { 0x12, 0x40 },
  { 0xd3, 0x04 },	//{ 0xd3, 0x7f },
  { 0xc0, 0x16 },
  { 0xC1, 0x12 },
  { 0x8c, 0x00 },
  { 0x86, 0x3d },
  { 0x50, 0x00 },
  { 0x51, 0x2C },
  { 0x52, 0x24 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x5A, 0x2c },
  { 0x5b, 0x24 },
  { 0x5c, 0x00 },
  { 0xff, 0xff },
};             

const struct sensor_reg OV2640_YUV422[]  =
{
  { 0xFF, 0x00 },
  { 0x05, 0x00 },
  { 0xDA, 0x10 },
  { 0xD7, 0x03 },
  { 0xDF, 0x00 },
  { 0x33, 0x80 },
  { 0x3C, 0x40 },
  { 0xe1, 0x77 },
  { 0x00, 0x00 },
  { 0xff, 0xff },
};

const struct sensor_reg OV2640_JPEG[]  =  
{
  { 0xe0, 0x14 },
  { 0xe1, 0x77 },
  { 0xe5, 0x1f },
  { 0xd7, 0x03 },
  { 0xda, 0x10 },
  { 0xe0, 0x00 },
  { 0xFF, 0x01 },
  { 0x04, 0x08 },
  { 0xff, 0xff },
}; 

/* JPG 160x120 */
const struct sensor_reg OV2640_160x120_JPEG[]  =  
{
  { 0xff, 0x01 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x4f, 0xca },
  { 0x50, 0xa8 },
  { 0x5a, 0x23 },
  { 0x6d, 0x00 },
  { 0x39, 0x12 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0xff, 0x00 },
  { 0xe0, 0x04 },
  { 0xc0, 0x64 },
  { 0xc1, 0x4b },
  { 0x86, 0x35 },
  { 0x50, 0x92 },
  { 0x51, 0xc8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x57, 0x00 },
  { 0x5a, 0x28 },
  { 0x5b, 0x1e },
  { 0x5c, 0x00 },
  { 0xe0, 0x00 },
  { 0xff, 0xff },
};

/* JPG, 0x176x144 */

const struct sensor_reg OV2640_176x144_JPEG[]  =  
{
  { 0xff, 0x01 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x4f, 0xca },
  { 0x50, 0xa8 },
  { 0x5a, 0x23 },
  { 0x6d, 0x00 },
  { 0x39, 0x12 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0xff, 0x00 },
  { 0xe0, 0x04 },
  { 0xc0, 0x64 },
  { 0xc1, 0x4b },
  { 0x86, 0x35 },
  { 0x50, 0x92 },
  { 0x51, 0xc8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x57, 0x00 },
  { 0x5a, 0x2c },
  { 0x5b, 0x24 },
  { 0x5c, 0x00 },
  { 0xe0, 0x00 },
  { 0xff, 0xff },
};

/* JPG 320x240 */

const struct sensor_reg OV2640_320x240_JPEG[]  =  
{
  { 0xff, 0x01 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x4f, 0xca },
  { 0x50, 0xa8 },
  { 0x5a, 0x23 },
  { 0x6d, 0x00 },
  { 0x39, 0x12 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0xff, 0x00 },
  { 0xe0, 0x04 },
  { 0xc0, 0x64 },
  { 0xc1, 0x4b },
  { 0x86, 0x35 },
  { 0x50, 0x89 },
  { 0x51, 0xc8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x57, 0x00 },
  { 0x5a, 0x50 },
  { 0x5b, 0x3c },
  { 0x5c, 0x00 },
  { 0xe0, 0x00 },
  { 0xff, 0xff },
};

/* JPG 352x288 */

const struct sensor_reg OV2640_352x288_JPEG[]  =  

{
  { 0xff, 0x01 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x4f, 0xca },
  { 0x50, 0xa8 },
  { 0x5a, 0x23 },
  { 0x6d, 0x00 },
  { 0x39, 0x12 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0xff, 0x00 },
  { 0xe0, 0x04 },
  { 0xc0, 0x64 },
  { 0xc1, 0x4b },
  { 0x86, 0x35 },
  { 0x50, 0x89 },
  { 0x51, 0xc8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x57, 0x00 },
  { 0x5a, 0x58 },
  { 0x5b, 0x48 },
  { 0x5c, 0x00 },
  { 0xe0, 0x00 },  
  { 0xff, 0xff },
};

/* JPG 640x480 */
const struct sensor_reg OV2640_640x480_JPEG[]  =  
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02为彩条
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	
	{0xff, 0x00}, 		      
	{0xe0, 0x04},       
	{0xc0, 0xc8},       
	{0xc1, 0x96},       
	{0x86, 0x3d},       
	{0x50, 0x89},       
	{0x51, 0x90},       
	{0x52, 0x2c},       
	{0x53, 0x00},       
	{0x54, 0x00},       
	{0x55, 0x88},       
	{0x57, 0x00},       
	{0x5a, 0xa0},       
	{0x5b, 0x78},       
	{0x5c, 0x00},       
	{0xd3, 0x04},       
	{0xe0, 0x00},       
                      
  	{0xff, 0xff},
};     
    
/* JPG 800x600 */
const struct sensor_reg OV2640_800x600_JPEG[]  =  
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02为彩条
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	
	{0xff, 0x00},
	{0xe0, 0x04},
	{0xc0, 0xc8},
	{0xc1, 0x96},
	{0x86, 0x35},
	{0x50, 0x89},
	{0x51, 0x90},
	{0x52, 0x2c},
	{0x53, 0x00},
	{0x54, 0x00},
	{0x55, 0x88},
	{0x57, 0x00},
	{0x5a, 0xc8},
	{0x5b, 0x96},
	{0x5c, 0x00},
	{0xd3, 0x02},
	{0xe0, 0x00},
                      
  	{0xff, 0xff},
};     
       
/* JPG 1024x768 */
const struct sensor_reg OV2640_1024x768_JPEG[]  =  
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02为彩条
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	
	{0xff, 0x00},		  
	{0xc0, 0xC8},          
	{0xc1, 0x96},          
	{0x8c, 0x00},          
	{0x86, 0x3D},          
	{0x50, 0x00},          
	{0x51, 0x90},          
	{0x52, 0x2C},          
	{0x53, 0x00},          
	{0x54, 0x00},          
	{0x55, 0x88},          
	{0x5a, 0x00},          
	{0x5b, 0xC0},          
	{0x5c, 0x01},          
	{0xd3, 0x02},          

                      
  {0xff, 0xff},
};  

   /* JPG 1280x1024 */
const struct sensor_reg OV2640_1280x1024_JPEG[]  =  
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02为彩条
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	
	{0xff, 0x00},     		      
	{0xe0, 0x04},           
	{0xc0, 0xc8},           
	{0xc1, 0x96},           
	{0x86, 0x3d},           
	{0x50, 0x00},           
	{0x51, 0x90},           
	{0x52, 0x2c},           
	{0x53, 0x00},           
	{0x54, 0x00},           
	{0x55, 0x88},           
	{0x57, 0x00},           
	{0x5a, 0x40},           
	{0x5b, 0xf0},           
	{0x5c, 0x01},           
	{0xd3, 0x02},           
	{0xe0, 0x00},           
                      
  	{0xff, 0xff},
};         
       
   /* JPG 1600x1200 */
const struct sensor_reg OV2640_1600x1200_JPEG[]  =  
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02为彩条
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	
	{0xff, 0x00},        	                              
	{0xe0, 0x04},                                   
	{0xc0, 0xc8},                                   
	{0xc1, 0x96},                                   
	{0x86, 0x3d},                                   
	{0x50, 0x00},                                   
	{0x51, 0x90},                                   
	{0x52, 0x2c},                                   
	{0x53, 0x00},                                   
	{0x54, 0x00},                                   
	{0x55, 0x88},                                   
	{0x57, 0x00},                                   
	{0x5a, 0x90},                                   
	{0x5b, 0x2C},                                   
	{0x5c, 0x05},              //bit2->1;bit[1:0]->1
	{0xd3, 0x02},                                   
	{0xe0, 0x00},                                   
                      
  	{0xff, 0xff},
}; 
