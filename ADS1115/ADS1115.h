#ifndef __ADS1115_H__
#define __ADS1115_H__
// 使用正点原子STM32F407征服者开发板iic接入多个ADS1115模块

#include "stm32f4xx.h"
#include "./BSP/IIC/myiic.h"

#ifndef ADS1115_ADDR
#define ADS1115_ADDR 0x48 // 上拉地址引脚接地，地址为0x48 默认地址， 1001000高七位，低一位用于写、读
#endif

#define ADS1115_OK                        0
#define ADS1115_INVALID_VOLTAGE           -100
#define ADS1115_INVALID_GAIN              0xFF
#define ADS1115_INVALID_MODE              0xFE

#define ADS1115_CONVERSION_DELAY    8

//  REGISTERS
#define ADS1115_REG_CONVERT         0x00
#define ADS1115_REG_CONFIG          0x01
#define ADS1115_REG_LOW_THRESHOLD   0x02
#define ADS1115_REG_HIGH_THRESHOLD  0x03


//  CONFIG REGISTER

//  BIT 15      Operational Status           // 1 << 15
#define ADS1115_OS_BUSY             0x0000
#define ADS1115_OS_NOT_BUSY         0x8000
#define ADS1115_OS_START_SINGLE     0x8000

//  BIT 12-14   read differential
#define ADS1115_MUX_DIFF_0_1        0x0000
#define ADS1115_MUX_DIFF_0_3        0x1000
#define ADS1115_MUX_DIFF_1_3        0x2000
#define ADS1115_MUX_DIFF_2_3        0x3000
//              read single
#define ADS1115_READ_0              0x4000   //  pin << 12
#define ADS1115_READ_1              0x5000   //  pin = 0..3
#define ADS1115_READ_2              0x6000
#define ADS1115_READ_3              0x7000


//  BIT 9-11    gain                         //  (0..5) << 9
#define ADS1115_PGA_6_144V          0x0000   //  voltage
#define ADS1115_PGA_4_096V          0x0200   //
#define ADS1115_PGA_2_048V          0x0400   //  default
#define ADS1115_PGA_1_024V          0x0600
#define ADS1115_PGA_0_512V          0x0800
#define ADS1115_PGA_0_256V          0x0A00

//  BIT 8       mode                         //  1 << 8
#define ADS1115_MODE_CONTINUE       0x0000
#define ADS1115_MODE_SINGLE         0x0100

//  BIT 5-7     data rate sample per second  // (0..7) << 5
/*
differs for different devices, check datasheet or readme.md

|  data rate  |  ADS101x  |  ADS111x  |   Notes   |
|:-----------:|----------:|----------:|:---------:|
|     0       |   128     |    8      |  slowest  |
|     1       |   250     |    16     |           |
|     2       |   490     |    32     |           |
|     3       |   920     |    64     |           |
|     4       |   1600    |    128    |  default  |
|     5       |   2400    |    250    |           |
|     6       |   3300    |    475    |           |
|     7       |   3300    |    860    |  fastest  |
*/

//  BIT 4 comparator modi                    // 1 << 4
#define ADS1115_COMP_MODE_TRADITIONAL   0x0000
#define ADS1115_COMP_MODE_WINDOW        0x0010

//  BIT 3 ALERT active value                 // 1 << 3
#define ADS1115_COMP_POL_ACTIV_LOW      0x0000
#define ADS1115_COMP_POL_ACTIV_HIGH     0x0008

//  BIT 2 ALERT latching                     // 1 << 2
#define ADS1115_COMP_NON_LATCH          0x0000
#define ADS1115_COMP_LATCH              0x0004

//  BIT 0-1 ALERT mode                       // (0..3)
#define ADS1115_COMP_QUE_1_CONV         0x0000  //  trigger alert after 1 convert
#define ADS1115_COMP_QUE_2_CONV         0x0001  //  trigger alert after 2 converts
#define ADS1115_COMP_QUE_4_CONV         0x0002  //  trigger alert after 4 converts
#define ADS1115_COMP_QUE_NONE           0x0003  //  disable comparator


// _CONFIG masks
//
//  |  bit  |  description           |
//  |:-----:|:-----------------------|
//  |   0   |  # channels            |
//  |   1   |  -                     |
//  |   2   |  resolution            |
//  |   3   |  -                     |
//  |   4   |  GAIN supported        |
//  |   5   |  COMPARATOR supported  |
//  |   6   |  -                     |
//  |   7   |  -                     |
//
#define ADS_CONF_CHAN_1  0x00
#define ADS_CONF_CHAN_4  0x01
#define ADS_CONF_RES_12  0x00
#define ADS_CONF_RES_16  0x04
#define ADS_CONF_NOGAIN  0x00
#define ADS_CONF_GAIN    0x10
#define ADS_CONF_NOCOMP  0x00
#define ADS_CONF_COMP    0x20
/*define a type for ADS1115 Parameters to communicate with multiple ADS1115*/
typedef struct 
{
    uint8_t address;
    uint8_t addr_write;
    uint8_t addr_read;
    uint8_t config;
    uint8_t maxPorts;
    uint8_t conversionDelay;
    uint8_t bitShift;
    uint8_t datarate;
    uint16_t gain;
    uint16_t mode;
    int8_t err;
    uint8_t compMode;
    uint8_t compPol;
    uint8_t compLatch;
    uint8_t compQueConvert;
} ADS1115;

/* All functions defined in ADS1115.c */
uint8_t iic_ReadBytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t iic_RequestFrom(uint8_t addr, uint8_t length);

void ADS1115_build(ADS1115 *ads, uint8_t addr);
void ADS1115_begin(uint8_t addr);
uint8_t ADS1115_WriteReg(ADS1115 *ads, uint8_t reg, uint16_t data);
uint16_t ADS1115_ReadReg(ADS1115 *ads, uint8_t reg);
uint8_t ADS1115_isReady(ADS1115 *ads);
uint8_t ADS1115_isBusy(ADS1115 *ads);
uint8_t ADS1115_isConnected(ADS1115 *ads);
void ADS1115_SetDataRate(ADS1115 *ads, uint8_t rate);
uint8_t ADS1115_GetDataRate(ADS1115 *ads);
void ADS1115_SetGain(ADS1115 *ads, uint8_t gain);
uint8_t ADS1115_GetGain(ADS1115 *ads);
void ADS1115_SetResolution(ADS1115 *ads, uint8_t res);
uint8_t ADS1115_GetResolution(ADS1115 *ads);
float ADS1115_ToVoltage(ADS1115 *ads, int16_t value);
float ADS1115_GetMaxVoltage(ADS1115 *ads);

void ADS1115_SetMode(ADS1115 *ads, uint8_t mode);
uint8_t ADS1115_GetMode(ADS1115 *ads);
void ADS1115_RequestADC(ADS1115 *ads, uint16_t readmode);
int16_t ADS1115_GetValue(ADS1115 *ads);
int16_t ADS1115_ReadADC(ADS1115 *ads, uint16_t readmode);



#endif
