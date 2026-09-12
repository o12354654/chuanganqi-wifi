#ifndef __RELAY_H
#define __RELAY_H

#include "sys.h"

/* ============== 继电器驱动（成品光耦模块，单路）==============
 * 接线：模块 VCC -> 5V(多数模块) 或 3.3V，GND -> GND，IN -> PB5
 * 触发极性：市面成品模块绝大多数是"低电平触发"（IN 拉低吸合）。
 *           按你模块丝印改下面这个宏：0 = 低电平吸合，1 = 高电平吸合。
 * 上电安全：MCU 复位期间引脚是浮空输入，模块输入端若没有上下拉，
 *           可能上电瞬间误吸合。低电平触发的模块建议在 IN 与 VCC 之间
 *           并一只 10k 电阻（等于硬件默认断开）。
 */
#define RELAY_PORT          GPIOB
#define RELAY_PIN           GPIO_Pin_5
#define RELAY_RCC           RCC_APB2Periph_GPIOB
#define RELAY_ACTIVE_LEVEL  0            /* 0 = 低电平吸合   1 = 高电平吸合 */

void    Relay_Init(void);                /* 初始化，并保持断开 */
void    Relay_On(void);
void    Relay_Off(void);
void    Relay_Set(uint8_t on);           /* 1 = 吸合   0 = 断开 */
uint8_t Relay_IsOn(void);

#endif
