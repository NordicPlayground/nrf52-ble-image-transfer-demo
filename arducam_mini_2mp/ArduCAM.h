/*
   ArduCAM.h - Arduino library support for CMOS Image Sensor
   Copyright (C)2011-2015 ArduCAM.com. All right reserved

   Basic functionality of this library are based on the demo-code provided by
   ArduCAM.com. You can find the latest version of the library at
   http://www.ArduCAM.com

   Now supported controllers:
    -	OV7670
    -	MT9D111
    -	OV7675
    -	OV2640
    -	OV3640
    -	OV5642
    -	OV7660
    -	OV7725
    - MT9M112
    - MT9V111
    - OV5640
    - MT9M001
    - MT9T112
    - MT9D112

   We will add support for many other sensors in next release.

   Supported MCU platform
    -	Theoretically support all Arduino families
      -	Arduino UNO R3			(Tested)
      -	Arduino MEGA2560 R3		(Tested)
      -	Arduino Leonardo R3		(Tested)
      -	Arduino Nano			(Tested)
      -	Arduino DUE				(Tested)
      - Arduino Yun				(Tested)
      -	Raspberry Pi			(Tested)
      - ESP8266-12				(Tested)

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
   2012/09/20   V1.0.0	by Lee	first release
   2012/10/23  V1.0.1  by Lee  Resolved some timing issue for the Read/Write Register
   2012/11/29	V1.1.0	by Lee  Add support for MT9D111 sensor
   2012/12/13	V1.2.0	by Lee	Add support for OV7675 sensor
   2012/12/28  V1.3.0	by Lee	Add support for OV2640,OV3640,OV5642 sensors
   2013/02/26	V2.0.0	by Lee	New Rev.B shield hardware, add support for FIFO control
                              and support Mega1280/2560 boards
   2013/05/28	V2.1.0	by Lee	Add support all drawing functions derived from UTFT library
   2013/08/24	V3.0.0	by Lee	Support ArudCAM shield Rev.C hardware, features SPI interface and low power mode.
                Support almost all series of Arduino boards including DUE.
   2014/03/09  V3.1.0  by Lee  Add the more impressive example sketches.
                Optimise the OV5642 settings, improve image quality.
                Add live preview before JPEG capture.
                Add play back photos one by one	after BMP capture.
   2014/05/01  V3.1.1  by Lee  Minor changes to add support Arduino IDE for linux distributions.
   2014/09/30  V3.2.0  by Lee  Improvement on OV5642 camera dirver.
   2014/10/06  V3.3.0  by Lee  Add OV7660,OV7725 camera support.
   2015/02/27  V3.4.0  by Lee  Add the support for Arduino Yun board, update the latest UTFT library for ArduCAM.
   2015/06/09  V3.4.1  by Lee	Minor changes and add some comments
   2015/06/19  V3.4.2  by Lee	Add support for MT9M112 camera.
   2015/06/20  V3.4.3  by Lee	Add support for MT9V111 camera.
   2015/06/22  V3.4.4  by Lee	Add support for OV5640 camera.
   2015/06/22  V3.4.5  by Lee	Add support for MT9M001 camera.
   2015/08/05  V3.4.6  by Lee	Add support for MT9T112 camera.
   2015/08/08  V3.4.7  by Lee	Add support for MT9D112 camera.
   2015/09/20  V3.4.8  by Lee	Add support for ESP8266 processor.
   2016/02/03	V3.4.9	by Lee	Add support for Arduino ZERO board.
   2016/06/07  V3.5.0  by Lee	Add support for OV5642_CAM_BIT_ROTATION_FIXED.
   2016/06/14  V3.5.1  by Lee	Add support for ArduCAM-Mini-5MP-Plus OV5640_CAM.
   2016/09/29	V3.5.2	by Lee	Optimize the OV5642 register settings
   2016/10/05	V4.0.0	by Lee	Add support for second generation of ArduCAM shield V2, ArduCAM-Mini-5MP-Plus(OV5642/OV5640).
   2016/10/17	V4.0.1	by Lee	Add support for Arduino Genuino 101 board
   --------------------------------------*/


#ifndef __ArduCAM_H
#define __ArduCAM_H

//#include "Arduino.h"
//#include <pins_arduino.h>
#include <stdint.h>
#include "nrfx_spim.h"
#include "nrfx_twim.h"

#if defined (__AVR__)
#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
#define cport(port, data) port &= data
#define sport(port, data) port |= data
#define swap(type, i, j) {type t = i; i = j; j = t;}
#define fontbyte(x) pgm_read_byte(&cfont.font[x])
#define regtype volatile uint8_t
#define regsize uint8_t
#endif

#if defined(__arm__)

#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);

#define cport(port, data) port &= data
#define sport(port, data) port |= data

#define swap(type, i, j) {type t = i; i = j; j = t;}

#define fontbyte(x) cfont.font[x]

#define regtype volatile uint32_t
#define regsize uint32_t

    #define pgm_read_byte(x)        (*((char *)x))
//  #define pgm_read_word(x)        (*((short *)(x & 0xfffffffe)))
    #define pgm_read_word(x)        ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x)))
    #define pgm_read_byte_near(x)   (*((char *)x))
    #define pgm_read_byte_far(x)    (*((char *)x))
//  #define pgm_read_word_near(x)   (*((short *)(x & 0xfffffffe))
//  #define pgm_read_word_far(x)    (*((short *)(x & 0xfffffffe)))
    #define pgm_read_word_near(x)   ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x)))
    #define pgm_read_word_far(x)    ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x))))
    #define PSTR(x)  x
  #if defined F
    #undef F
  #endif
  #define F(X) (X)


#endif

#if defined(ESP8266)

#define cbi(reg, bitmask) digitalWrite(bitmask, LOW)
#define sbi(reg, bitmask) digitalWrite(bitmask, HIGH)
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);

#define cport(port, data) port &= data
#define sport(port, data) port |= data

#define swap(type, i, j) {type t = i; i = j; j = t;}

#define fontbyte(x) cfont.font[x]

#define regtype volatile uint32_t
#define regsize uint32_t
#endif

#if defined(__CPU_ARC__)
#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
#define cport(port, data) port &= data
#define sport(port, data) port |= data
#define swap(type, i, j) {type t = i; i = j; j = t;}
#define fontbyte(x) pgm_read_byte(&cfont.font[x])
#define regtype volatile uint32_t
#define regsize uint32_t
#endif

/****************************************************/
/* Sensor related definition                        */
/****************************************************/
#define BMP   0
#define JPEG  1

#define OV7670    0
#define MT9D111_A 1
#define OV7675    2
#define OV5642    3
#define OV3640    4
#define OV2640    5
#define OV9655    6
#define MT9M112   7
#define OV7725    8
#define OV7660    9
#define MT9M001   10
#define OV5640    11
#define MT9D111_B 12
#define OV9650    13
#define MT9V111   14
#define MT9T112   15
#define MT9D112   16

#define OV2640_160x120    0 //160x120
#define OV2640_176x144    1 //176x144
#define OV2640_320x240    2 //320x240
#define OV2640_352x288    3 //352x288
#define OV2640_640x480    4 //640x480
#define OV2640_800x600    5 //800x600
#define OV2640_1024x768   6 //1024x768
#define OV2640_1280x1024  7 //1280x1024
#define OV2640_1600x1200  8 //1600x1200

#define OV5642_320x240    0 //320x240
#define OV5642_640x480    1 //640x480
#define OV5642_1024x768   2 //1024x768
#define OV5642_1280x960   3 //1280x960
#define OV5642_1600x1200  4 //1600x1200
#define OV5642_2048x1536  5 //2048x1536
#define OV5642_2592x1944  6 //2592x1944


#define OV5640_320x240    0 //320x240
#define OV5640_352x288    1 //352x288
#define OV5640_640x480    2 //640x480
#define OV5640_800x480    3 //800x480
#define OV5640_1024x768   4 //1024x768
#define OV5640_1280x960   5 //1280x960
#define OV5640_1600x1200  6  //1600x1200
#define OV5640_2048x1536  7  //2048x1536
#define OV5640_2592x1944  8  //2592x1944

/****************************************************/
/* I2C Control Definition                           */
/****************************************************/
#define I2C_ADDR_8BIT 0
#define I2C_ADDR_16BIT 1
#define I2C_REG_8BIT 0
#define I2C_REG_16BIT 1
#define I2C_DAT_8BIT 0
#define I2C_DAT_16BIT 1

/* Register initialization tables for SENSORs */
/* Terminating list entry for reg */
#define SENSOR_REG_TERM_8BIT                0xFF
#define SENSOR_REG_TERM_16BIT               0xFFFF
/* Terminating list entry for val */
#define SENSOR_VAL_TERM_8BIT                0xFF
#define SENSOR_VAL_TERM_16BIT               0xFFFF

//Define maximum frame buffer size
#if (defined OV2640_MINI_2MP)
#define MAX_FIFO_SIZE   0x5FFFF     //384KByte
#elif (defined OV2640_MINI_2MP_PLUS)
#define MAX_FIFO_SIZE   0x5FFFF     //384KByte
#elif (defined OV5642_MINI_5MP || defined OV5642_MINI_5MP_BIT_ROTATION_FIXED || defined ARDUCAM_SHIELD_REVC)
#define MAX_FIFO_SIZE   0x7FFFF     //512KByte
#else
#define MAX_FIFO_SIZE   0x7FFFFF    //8MByte
#endif

/****************************************************/
/* ArduChip registers definition                      */
/****************************************************/
#define RWBIT                 0x80  //READ AND WRITE BIT IS BIT[7]

#define ARDUCHIP_TEST1        0x00  //TEST register

#if !(defined OV2640_MINI_2MP)
  #define ARDUCHIP_FRAMES       0x01  //FRAME control register, Bit[2:0] = Number of frames to be captured
                                      //On 5MP_Plus platforms bit[2:0] = 7 means continuous capture until frame buffer is full
#endif

#define ARDUCHIP_MODE         0x02  //Mode register
#define MCU2LCD_MODE          0x00
#define CAM2LCD_MODE          0x01
#define LCD2MCU_MODE          0x02

#define ARDUCHIP_TIM          0x03  //Timming control
#if !(defined OV2640_MINI_2MP)
  #define HREF_LEVEL_MASK       0x01  //0 = High active ,     1 = Low active
  #define VSYNC_LEVEL_MASK      0x02  //0 = High active ,     1 = Low active
  #define LCD_BKEN_MASK         0x04  //0 = Enable,           1 = Disable
  #if (defined ARDUCAM_SHIELD_V2)
    #define PCLK_REVERSE_MASK   0x08  //0 = Normal PCLK,    1 = REVERSED PCLK
  #else
    #define PCLK_DELAY_MASK     0x08  //0 = data no delay,		1 = data delayed one PCLK
  #endif
//#define MODE_MASK             0x10  //0 = LCD mode,         1 = FIFO mode
#endif
//#define FIFO_PWRDN_MASK	      0x20    //0 = Normal operation, 1 = FIFO power down
//#define LOW_POWER_MODE			  0x40    //0 = Normal mode,      1 = Low power mode

#define ARDUCHIP_FIFO         0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK       0x01
#define FIFO_START_MASK       0x02
#define FIFO_RDPTR_RST_MASK     0x10
#define FIFO_WRPTR_RST_MASK     0x20

#define ARDUCHIP_GPIO       0x06  //GPIO Write Register
#if !(defined (ARDUCAM_SHIELD_V2) || defined (ARDUCAM_SHIELD_REVC))
#define GPIO_RESET_MASK     0x01  //0 = Sensor reset,							1 =  Sensor normal operation
#if !( defined(OV5642_CAM) || defined(OV5642_MINI_5MP) || defined(OV5642_MINI_5MP_BIT_ROTATION_FIXED) || defined(OV5642_MINI_5MP_PLUS) )
#define GPIO_PWDN_MASK      0x02  //0 = Sensor normal operation,  1 = Sensor standby
#endif
#define GPIO_PWREN_MASK     0x04  //0 = Sensor LDO disable,       1 = sensor LDO enable

#endif

#define BURST_FIFO_READ     0x3C  //Burst FIFO read operation
#define SINGLE_FIFO_READ    0x3D  //Single FIFO read operation

#define ARDUCHIP_REV          0x40  //ArduCHIP revision
#define VER_LOW_MASK          0x3F
#define VER_HIGH_MASK         0xC0

#define ARDUCHIP_TRIG         0x41  //Trigger source
#define VSYNC_MASK            0x01
#define SHUTTER_MASK          0x02
#define CAP_DONE_MASK         0x08

#define FIFO_SIZE1        0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2        0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3        0x44  //Camera write FIFO size[18:16]


/****************************************************/


/****************************************************************/
/* define a structure for sensor register initialization values */
/****************************************************************/
struct sensor_reg {
        uint16_t reg;
        uint16_t val;
};

typedef void (*arducam_spi_callback_t)(uint32_t tx_bytes, uint32_t rx_bytes);

void arducam_init(uint8_t model, uint32_t scl, uint32_t sda, uint32_t csn, uint32_t mosi, uint32_t miso, uint32_t sck);
void arducam_InitCAM(void);

void arducam_CS_HIGH(void);
void arducam_CS_LOW(void);
uint8_t arducam_spiWrite(uint8_t dataByte);
void arducam_spiFinalize(void);
void arducam_spiReadMulti(uint8_t *rxBuf, uint32_t length);
void arducam_spiRegisterCallback(arducam_spi_callback_t callback);

void arducam_flush_fifo(void);
void arducam_start_capture(void);
void arducam_clear_fifo_flag(void);
uint8_t arducam_read_fifo(void);

uint8_t arducam_read_reg(uint8_t addr);
void arducam_write_reg(uint8_t addr, uint8_t data);

uint32_t arducam_read_fifo_length(void);
void arducam_set_fifo_burst(void);
void arducam_set_bit(uint8_t addr, uint8_t bit);
void arducam_clear_bit(uint8_t addr, uint8_t bit);
uint8_t arducam_get_bit(uint8_t addr, uint8_t bit);
void arducam_set_mode(uint8_t mode);

int arducam_wrSensorRegs(const struct sensor_reg*);
int arducam_wrSensorRegs8_8(const struct sensor_reg*);
int arducam_wrSensorRegs8_16(const struct sensor_reg*);
int arducam_wrSensorRegs16_8(const struct sensor_reg*);
int arducam_wrSensorRegs16_16(const struct sensor_reg*);

uint8_t arducam_wrSensorReg(int regID, int regDat);
uint8_t arducam_wrSensorReg8_8(int regID, int regDat);
uint8_t arducam_wrSensorReg8_16(int regID, int regDat);
uint8_t arducam_wrSensorReg16_8(int regID, int regDat);
uint8_t arducam_wrSensorReg16_16(int regID, int regDat);

uint8_t arducam_rdSensorReg8_8(uint8_t regID, uint8_t* regDat);
uint8_t arducam_rdSensorReg16_8(uint16_t regID, uint8_t* regDat);
uint8_t arducam_rdSensorReg8_16(uint8_t regID, uint16_t* regDat);
uint8_t arducam_rdSensorReg16_16(uint16_t regID, uint16_t* regDat);

void arducam_OV2640_set_JPEG_size(uint8_t size);
void arducam_OV5642_set_JPEG_size(uint8_t size);
void arducam_OV5640_set_JPEG_size(uint8_t size);
void arducam_set_format(uint8_t fmt);

void arducam_transferBytes_(uint8_t * out, uint8_t * in, uint8_t size);
void arducam_transferBytes(uint8_t * out, uint8_t * in, uint32_t size);

int arducam_bus_write(int address, int value);
uint8_t arducam_bus_read(int address);

void arducam_spiTwiInit(void);

#if (defined(OV2640_CAM) || defined(OV2640_MINI_2MP) || defined(OV2640_MINI_2MP_PLUS))
  #include "ov2640_regs.h"
#endif

#endif
