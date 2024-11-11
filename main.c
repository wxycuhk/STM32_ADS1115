/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-23
 * @brief       IIC ʵ��
 * @license     Copyright (c) 2020-2032, �������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"

#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/ADS1115/ADS1115.h"
#include "./BSP/IIC/myiic.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LCD/lcd.h"
#include "./USMART/usmart.h"
#include "./BSP/KEY/key.h"

ADS1115 ADS0; // 0X48
ADS1115 ADS1; // 0X49
int idx = 0;
int16_t ADS0_value[4] = {0};
int16_t ADS1_value[4] = {0};

int main(void)
{
	HAL_Init();                                 /* ��ʼ��HAL�� */
    sys_stm32_clock_init(336, 8, 2, 7);         /* ����ʱ��,168Mhz */
    delay_init(168);                            /* ��ʱ��ʼ�� */
    usart_init(115200);                         /* ���ڳ�ʼ��Ϊ115200 */
	iic_init();                                 /* ��ʼ��iic gpio */
	ADS1115_build(&ADS0, 0x48);
	ADS1115_build(&ADS1, 0x49);
    ADS1115_SetDataRate(&ADS0, 7); //highest speed
    ADS1115_SetDataRate(&ADS1, 7); //highest speed

    while(1)
    {
        ADS_request_all();
        while(ADS_read_all());
        ADS_print_all();
        delay_ms(1000);
    }

		
}	
	
void ADS_request_all()
{
    if(ADS1115_isConnected(&ADS0))   ADS1115_RequestADC(&ADS0);
    if(ADS1115_isConnected(&ADS1))   ADS1115_RequestADC(&ADS1);
    
}

uint8_t ADS_read_all()
{
    if(ADS1115_isConnected(&ADS0))   ADS0_value[idx] = ADS1115_GetValue(&ADS0);
    if(ADS1115_isConnected(&ADS1))   ADS1_value[idx] = ADS1115_GetValue(&ADS1);
    idx++;
    if (idx < 4)
    {
        ADS_request_all();
        return 1;
    }
    idx = 0;
    return 0;
}

void ADS_print_all()
{
    printf("ADS0: ");
    for(int i = 0; i < 4; i++)
    {
        printf("0x%x ", ADS0_value[i]);
    }
    printf("\r\n");
    printf("ADS1: ");
    for(int i = 0; i < 4; i++)
    {
        printf("0x%x ", ADS1_value[i]);
    }
    printf("\r\n");
}