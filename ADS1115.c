#include "ADS1115.h"
#include "./BSP/IIC/myiic.h"
#include "delay.h"
#include "usart.h"

// iic代码直接调用正点原子库myiic
// 该ADS1115代码可以实现多个ADS1115模块的iic通信，只需修改地址即可
// 该代码由Arduino库移植而来，注意Arduino库的地址是7位地址，而STM32库的地址是8位地址,加入了读写位

/*
ADS1115 ADS0; // GND
ADS1115 ADS1; // VDD
ADS1115 ADS2; // SDA
ADS1115 ADS3; // SCL
*/

uint8_t iic_ReadBytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data){
    uint8_t i = 0;
    uint8_t ack_flag = 0;
    iic_start();
    iic_send_byte(addr);
    while(iic_wait_ack());
    iic_send_byte(reg);
    while(iic_wait_ack());
    iic_start();
    iic_send_byte(addr + 1);
    ack_flag = iic_wait_ack();
    for(i = 0; i < length; i++)
    {
        data[i] = iic_read_byte();
        if(i == length - 1) iic_nack();
        else iic_ack();
    }
    iic_stop();
    return 1;
}

//读取指定字节数据，读取成功返回字节数，失败返回0
uint8_t iic_RequestFrom(uint8_t addr, uint8_t length){
    uint8_t receivedBytes = 0;
    iic_start(); // send start signal
    iic_send_byte(addr); // 高八位写入读操作
    if(iic_wait_ack()) // wait ack等候时间过久，失败返1
    {
        iic_stop(); // 未等到应答，发送停止信号
        return 0;
    }

    for(receivedBytes = 0; receivedBytes < length; receivedBytes++)
    {
        if(receivedBytes == length - 1) iic_nack(); // 最后一个字节不应答
        else iic_ack(); // 其他字节应答
    }
    iic_stop(); // 发送停止信号
    return receivedBytes; // 返回接收到的字节数

}

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

void ADS1115_build(ADS1115 *ads, uint8_t addr){ // set up the ADS1115 with addr and default config, must be done in setup
    ads->address = addr; // 7位地址
    ads->addr_write = ads -> addr << 1 | 0; // 7位地址+1位读写位，写操作为0
    ads->addr_read = ads -> addr << 1 | 1; // 7位地址+1位读写位，读操作为1
    ads->config = ADS_CONF_COMP | ADS_CONF_GAIN | ADS_CONF_RES_16 | ADS_CONF_CHAN_4;
    ads->conversionDelay = ADS1115_CONVERSION_DELAY;
    ads->bitShift = 0;
    ads->maxPorts = 4;
    ads->gain = ADS1115_PGA_6_144V;
    ads->mode = ADS1115_MODE_SINGLE;
    ads->datarate = 4;
    ads->err = ADS1115_OK;
    ads->compMode = 0;
    ads->compPol = 1;
    ads->compLatch = 0;
    ads->compQueConvert = 3;
    ADS1115_begin(addr);
}

void ADS1115_begin(uint8_t addr){
    IIC_Init(); //wire.begin() in arduino to init scl & sda pin
    //HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET);
    if(addr < 0x48 || addr > 0x4B) 
    {
        printf("ADS1115_begin error: invalid address\r\n");
        return;
    }
    printf("ADS1115_begin: address = 0x%x\r\n", addr);
}

// according to the quick guide of ADS1115, 3 steps are needed to read data
// 1. write to config which is the function of ADS1115_WriteReg using address (first 7-bit I2C address followed by a low R/W bit
//    1). first byte: 7-bit I2C address followed by a low R/W bit : buf[0] = ads->addr_write
//    2). second byte: 0x01(points to config register) : buf[1] = ADS1115_REG_CONFIG
//    3). third byte: MSB of config register : buf[2] = (uint8_t)(data >> 8)
//    4). fourth byte: LSB of config register : buf[3] = (uint8_t)(data & 0xFF)

uint8_t ADS1115_WriteReg(ADS1115 *ads, uint8_t reg, uint16_t data){
    uint8_t i = 0;
    uint8_t buf[4];
    uint8_t ack_flag = 0;
    //7位地址+1位读写位，写操作
    buf[0] = ads -> addr_write; // 7位地址+1位读写位，写操作
    buf[1] = reg; // 设置iic寄存器指针 0x01
    buf[2] = (uint8_t)(data >> 8); //MSB: 写入寄存器的高8位
    buf[3] = (uint8_t)(data & 0xFF); //LSB: 写入寄存器的低8位
    HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET); // CONFIRM scl is high at beginning
    iic_start();
    for(i = 0; i < 4; i++)
    {
        iic_send_byte(buf[i]);
        ack_flag = iic_wait_ack();
    }
    iic_stop();
    return ack_flag;
}

// 2. write to pointer register which is the function of ADS1115_ReadReg using address and register
//    1). first byte: 7-bit I2C address followed by a low R/W bit : buf[0] = ads->addr_write
//    2). second byte: 0x00(points to conversion register) : buf[1] = ADS1115_REG_CONVERT

// 3. read the data from the conversion register
//    1). first byte: 7-bit I2C address followed by a high R/W bit : buf[0] = ads->addr_read
//    2). second byte: ADS1115 response with MSB of the conversion register : buf[1] = iic_read_byte() >> 8
//    3). third byte: ADS1115 response with LSB of the conversion register : buf[2] = iic_read_byte()

uint16_t ADS1115_ReadReg(ADS1115 *ads, uint8_t reg){
    uint8_t i = 0;
    uint8_t buf[2];
    uint16_t data = 0;
    buf[0] = ads->addr_write;
    buf[1] = reg;
    HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET); // CONFIRM scl is high at beginning
    iic_start();
    for(i = 0; i < 2; i++) // 2. write to pointer register
    {
        iic_send_byte(buf[i]);
        iic_wait_ack();
    }
    iic_stop();

    int rv = iic_RequestFrom(ads->addr_read, 2); // 3. read data from conversion reg : send the first byte to read in function iic_RequestFrom, not sure if it is correct
    if(rv == 2)
    {
        data = iic_read_byte() << 8; // MSB
        data += iic_read_byte();    // LSB
        return data;
    }
    return 0x0000;
}

uint8_t ADS1115_isReady(ADS1115 *ads){
    uint16_t val = ADS1115_ReadReg(ads->addr_read, ADS1115_REG_CONFIG);
    return ((val & ADS1X15_OS_NOT_BUSY) > 0);
}

uint8_t ADS1115_isBusy(ADS1115 *ads){
    if(ADS1115_isReady(ads) == 0) return 1;
    else return 0;
}

uint8_t ADS1115_isConnected(ADS1115 *ads){
    iic_start();
    iic_send_byte(ads->addr_write);
    return (iic_wait_ack() == 0);
}


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

void ADS1115_SetDataRate(ADS1115 *ads, uint8_t rate){
    ads->datarate = rate;
    if(rate > 7) rate = 7;
    ads->datarate = rate <<= 5;
}

uint8_t ADS1115_GetDataRate(ADS1115 *ads){
    return (ads->datarate >> 5) & 0x07;
}

void ADS1115_SetGain(ADS1115 *ads, uint8_t gain){
    if(gain > 5) gain = 0;
    switch(gain){
        default:
        case 0: ads->gain = ADS1115_PGA_6_144V; break;
        case 1: ads->gain = ADS1115_PGA_4_096V; break;
        case 2: ads->gain = ADS1115_PGA_2_048V; break;
        case 4: ads->gain = ADS1115_PGA_1_024V; break;
        case 8: ads->gain = ADS1115_PGA_0_512V; break;
        case 16: ads->gain = ADS1115_PGA_0_256V; break;
    }
}

uint8_t ADS1115_GetGain(ADS1115 *ads){
    if(!(ads->config & ADS_CONF_GAIN)) return 0;
    switch(ads->gain){
        case ADS1115_PGA_6_144V: return 0;
        case ADS1115_PGA_4_096V: return 1;
        case ADS1115_PGA_2_048V: return 2;
        case ADS1115_PGA_1_024V: return 4;
        case ADS1115_PGA_0_512V: return 8;
        case ADS1115_PGA_0_256V: return 16;
    }
    ads->err = ADS1115_INVALID_GAIN;
    return ads->err;
}

void ADS1115_SetResolution(ADS1115 *ads, uint8_t res){
    if(res != 12 && res != 16) res = 16; // default 16 bit mode
    switch(res)
    {
        case 12: ads->config |= ADS_CONF_RES_12; break; // 12 bit mode
        case 16: ads->config |= ADS_CONF_RES_16; break; // 16 bit mode
    }
}
uint8_t ADS1115_GetResolution(ADS1115 *ads){
    if(ads->config & ADS_CONF_RES_12) return 12;
    else return 16;
}

float ADS1115_ToVoltage(ADS1115 *ads, int16_t value){
    if (value == 0) return 0;

    float volts = ADS1115_GetMaxVoltage(ads);
    if(volts < 0) return volts;

    volts *= value;
    if(ads->config & ADS_CONF_RES_16){
        volts /= 32767; // value = 16 bits - sign bit = 15 bits mantissa
    }
    else{
        volts /= 2047; // value = 12 bits - sign bit = 11 bit mantissa
    }
    return volts;
}

float ADS1115_GetMaxVoltage(ADS1115 *ads){
    switch(ads->gain){
        case ADS1115_PGA_6_144V: return 6.144;
        case ADS1115_PGA_4_096V: return 4.096;
        case ADS1115_PGA_2_048V: return 2.048;
        case ADS1115_PGA_1_024V: return 1.024;
        case ADS1115_PGA_0_512V: return 0.512;
        case ADS1115_PGA_0_256V: return 0.256;
    }
    ads->err = ADS1115_INVALID_VOLTAGE;
    return ads->err;
}

void ADS1115_SetMode(ADS1115 *ads, uint8_t mode){
    switch(mode)
    {
        case 0: ads->mode = ADS1115_MODE_CONTINUE; break;
        case 1: ads->mode = ADS1115_MODE_SINGLE; break;
        default: ads->mode = ADS1115_MODE_SINGLE; break;
    }
}

uint8_t ADS1115_GetMode(ADS1115 *ads){
    switch(ads->mode){
        case ADS1115_MODE_CONTINUE: return 0;
        case ADS1115_MODE_SINGLE: return 1;
    }
    ads->err = ADS1115_INVALID_MODE;
    return ads->err;
}

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

void ADS1115_RequestADC(ADS1115 *ads, uint16_t readmode){
    // write to register is needed in continuous mode as other flags can be changed
    uint16_t config = ADS1115_OS_START_SINGLE; // bit 15 force wake up if needed
    config |= readmode;                        // bit 12-14
    config |= ads->gain;                       // bit 9-11
    config |= ads->mode;                       // bit 8  mode bit
    config |= ads->datarate;                   // bit 5-7   set the datarate bits
    if (ads->compMode)  config |= ADS1115_COMP_MODE_WINDOW; // bit 4 comparator modi
    else                config |= ADS1115_COMP_MODE_TRADITIONAL;
    if (ads->compPol)   config |= ADS1115_COMP_POL_ACTIV_HIGH; // bit 3 ALERT active value
    else                config |= ADS1115_COMP_POL_ACTIV_LOW;
    if (ads->compLatch) config |= ADS1115_COMP_LATCH;
    else                config |= ADS1115_COMP_NON_LATCH; // bit 2 ALERT latching
    config |= ads->compQueConvert; // bit 0..1 ALERT mode
    ADS1115_WriteReg(ads, ADS1115_REG_CONFIG, config);
}
int16_t ADS1115_GetValue(ADS1115 *ads){
    int16_t raw = ADS1115_ReadReg(ads, ADS1115_REG_CONVERT);
    if(ads->config & ADS_CONF_RES_16){ // 16 bit mode
        return raw;
    }
    else{
        return (raw >> 4);   // 12 bit mode
    }

}


/*wait for further modify*/
int16_t ADS1115_ReadADC(ADS1115 *ads, uint16_t readmode){
    ADS1115_RequestADC(ads, readmode);
    if(ads->mode == ADS1115_MODE_SINGLE){
        while(ADS1115_isBusy(ads) == 0); // add yield afterwards if freeRTOS is used
    }
    else{
        delay_ms(ads->conversionDelay);
    }
    return ADS1115_GetValue(ads);
}

