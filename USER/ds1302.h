#ifndef __DS1302_H
#define __DS1302_H

#include "stm32f10x.h"

#define DS1302_RST_PIN     GPIO_Pin_7
#define DS1302_RST_PORT    GPIOB
#define DS1302_RST_RCC     RCC_APB2Periph_GPIOB

#define DS1302_CLK_PIN     GPIO_Pin_8
#define DS1302_CLK_PORT    GPIOB
#define DS1302_CLK_RCC     RCC_APB2Periph_GPIOB

#define DS1302_IO_PIN      GPIO_Pin_9
#define DS1302_IO_PORT     GPIOB
#define DS1302_IO_RCC      RCC_APB2Periph_GPIOB

typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t week;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} DS1302_TIME;

void DS1302_Init(void);
void DS1302_SetTime(DS1302_TIME *time);
void DS1302_GetTime(DS1302_TIME *time);
uint8_t DS1302_IsHalt(void);

#endif
