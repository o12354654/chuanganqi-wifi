#include "ds1302.h"

#define RST_L()     GPIO_ResetBits(DS1302_RST_PORT, DS1302_RST_PIN)
#define RST_H()     GPIO_SetBits(DS1302_RST_PORT, DS1302_RST_PIN)
#define CLK_L()     GPIO_ResetBits(DS1302_CLK_PORT, DS1302_CLK_PIN)
#define CLK_H()     GPIO_SetBits(DS1302_CLK_PORT, DS1302_CLK_PIN)
#define DAT_L()     GPIO_ResetBits(DS1302_IO_PORT, DS1302_IO_PIN)
#define DAT_H()     GPIO_SetBits(DS1302_IO_PORT, DS1302_IO_PIN)
#define DAT_READ()  GPIO_ReadInputDataBit(DS1302_IO_PORT, DS1302_IO_PIN)

static void DS1302_Delay(void)
{
    uint8_t i = 5;
    while (i--);
}

static void DS1302_IO_OUT(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = DS1302_IO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DS1302_IO_PORT, &gpio);
}

static void DS1302_IO_IN(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = DS1302_IO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(DS1302_IO_PORT, &gpio);
}

static void DS1302_Write_Byte(uint8_t dat)
{
    uint8_t i;
    DS1302_IO_OUT();
    for (i = 0; i < 8; i++)
    {
        CLK_L();
        DS1302_Delay();
        if (dat & 0x01) DAT_H();
        else DAT_L();
        dat >>= 1;
        CLK_H();
        DS1302_Delay();
    }
}

static uint8_t DS1302_Read_Byte(void)
{
    uint8_t i, dat = 0;
    DS1302_IO_IN();
    for (i = 0; i < 8; i++)
    {
        dat >>= 1;
        CLK_L();
        DS1302_Delay();
        if (DAT_READ()) dat |= 0x80;
        CLK_H();
        DS1302_Delay();
    }
    return dat;
}

static void DS1302_Write_Reg(uint8_t addr, uint8_t dat)
{
    RST_L(); CLK_L();
    RST_H();
    DS1302_Write_Byte(addr);
    DS1302_Write_Byte(dat);
    RST_L();
}

static uint8_t DS1302_Read_Reg(uint8_t addr)
{
    uint8_t dat;
    RST_L(); CLK_L();
    RST_H();
    DS1302_Write_Byte(addr);
    dat = DS1302_Read_Byte();
    RST_L();
    return dat;
}

static uint8_t BCD_TO_DEC(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0f);
}

static uint8_t DEC_TO_BCD(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

void DS1302_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = DS1302_RST_PIN | DS1302_CLK_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    RST_L();
    CLK_L();
    DS1302_Write_Reg(0x8E, 0x00);   /* 关写保护 */
    DS1302_Write_Reg(0x8E, 0x80);   /* 恢复写保护，防误写(要写时间时 SetTime 会自己再打开) */
}

/* RTC 是否停振：秒寄存器 bit7(CH)=1 表示时钟停走，新片/掉电后常见 */
uint8_t DS1302_IsHalt(void)
{
    return (DS1302_Read_Reg(0x81) & 0x80) ? 1 : 0;
}

void DS1302_SetTime(DS1302_TIME *time)
{
    DS1302_Write_Reg(0x8E, 0x00);
    DS1302_Write_Reg(0x80, DEC_TO_BCD(time->second));
    DS1302_Write_Reg(0x82, DEC_TO_BCD(time->minute));
    DS1302_Write_Reg(0x84, DEC_TO_BCD(time->hour));
    DS1302_Write_Reg(0x86, DEC_TO_BCD(time->day));
    DS1302_Write_Reg(0x88, DEC_TO_BCD(time->month));
    DS1302_Write_Reg(0x8A, DEC_TO_BCD(time->week));
    DS1302_Write_Reg(0x8C, DEC_TO_BCD(time->year));
    DS1302_Write_Reg(0x8E, 0x80);
}

void DS1302_GetTime(DS1302_TIME *time)
{
    time->second  = BCD_TO_DEC(DS1302_Read_Reg(0x81) & 0x7F);
    time->minute  = BCD_TO_DEC(DS1302_Read_Reg(0x83));
    time->hour    = BCD_TO_DEC(DS1302_Read_Reg(0x85));
    time->day     = BCD_TO_DEC(DS1302_Read_Reg(0x87));
    time->month   = BCD_TO_DEC(DS1302_Read_Reg(0x89));
    time->week    = BCD_TO_DEC(DS1302_Read_Reg(0x8B));
    time->year    = BCD_TO_DEC(DS1302_Read_Reg(0x8D));
}
