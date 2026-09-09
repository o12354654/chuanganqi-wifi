/***************STM32F103C8T6**********************
 * 文件名  ：delay.c
 * 功能    ：精确延时
 * 备注    ：使用系统定时器SysTick

********************LIGEN*************************/

#include "delay.h"
#include "misc.h"

static u8  fac_us = 0;
static u16 fac_ms = 0;

void Delay_init(u8 SYSCLK)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    fac_us = SYSCLK / 8;
    fac_ms = (u16)fac_us * 1000;
}

void Delay_ms(u16 nms)
{
    u32 temp;
    SysTick->LOAD = (u32)nms * fac_ms;
    SysTick->VAL = 0x00;
    SysTick->CTRL = 0x01;
    do
    {
        temp = SysTick->CTRL;
    }
    while (temp & 0x01 && !(temp & (1 << 16)));
    SysTick->CTRL = 0x00;
    SysTick->VAL = 0X00;
}

void Delay_us(u32 nus)
{
    u32 temp;
    SysTick->LOAD = nus * fac_us;
    SysTick->VAL = 0x00;
    SysTick->CTRL = 0x01;
    do
    {
        temp = SysTick->CTRL;
    }
    while (temp & 0x01 && !(temp & (1 << 16)));
    SysTick->CTRL = 0x00;
    SysTick->VAL = 0X00;
}
