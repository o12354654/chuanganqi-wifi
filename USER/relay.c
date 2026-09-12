/***************STM32F103C8T6**********************
 * 文件名  ：relay.c
 * 功能    ：继电器驱动（成品光耦模块，单路，非阻塞）
 * 备注    ：只做引脚控制，吸合逻辑（阈值/云端命令）由上层决定
 * 接口    ：PB5 - 模块 IN

********************LIGEN*************************/

#include "relay.h"

/* 模块"吸合/断开"对应的引脚电平，由 RELAY_ACTIVE_LEVEL 推导 */
#define RELAY_ON_LEVEL    (RELAY_ACTIVE_LEVEL ? 0 : 1)
#define RELAY_OFF_LEVEL   (RELAY_ACTIVE_LEVEL ? 1 : 0)

static uint8_t s_on = 0;

static void RELAY_WRITE_LEVEL(uint8_t level)
{
    if (level) {
        GPIO_SetBits(RELAY_PORT, RELAY_PIN);
    } else {
        GPIO_ResetBits(RELAY_PORT, RELAY_PIN);
    }
}

/**
 * @brief  继电器初始化：引脚配推挽输出，并停在上电默认状态"断开"
 * @note   先把 ODR 写成"断开"电平再切输出模式，
 *         避免配置成输出的那一瞬间引脚上的旧值把继电器抽一下
 */
void Relay_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RELAY_RCC, ENABLE);

    RELAY_WRITE_LEVEL(RELAY_OFF_LEVEL);

    gpio.GPIO_Pin   = RELAY_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RELAY_PORT, &gpio);

    s_on = 0;
    Relay_Off();
}

void Relay_On(void)
{
    RELAY_WRITE_LEVEL(RELAY_ON_LEVEL);
    s_on = 1;
}

void Relay_Off(void)
{
    RELAY_WRITE_LEVEL(RELAY_OFF_LEVEL);
    s_on = 0;
}

void Relay_Set(uint8_t on)
{
    if (on) {
        Relay_On();
    } else {
        Relay_Off();
    }
}

uint8_t Relay_IsOn(void)
{
    return s_on;
}
