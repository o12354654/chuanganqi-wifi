/***************STM32F103C8T6**********************
 * 文件名  ：led.c
 * 功能    ：LED指示灯驱动
 * 硬件连接：LED端口PB8
 * 备注    ：修改LED端口时需同步修改GPIO_Pin

********************LIGEN*************************/

#include "led.h"

#define LED GPIO_Pin_8

void LED_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, LED);
}

void LED_Toggle(void)
{
    GPIO_WriteBit(GPIOB, LED, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, LED)));
}
