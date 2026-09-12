#ifndef __RELAY_CTRL_H
#define __RELAY_CTRL_H

#include "sys.h"

/* ================== 本地阈值联动（温湿度 → 继电器）==================
 * 【控制逻辑】
 *   温度 >= RCTRL_TEMP_HIGH  或  湿度 >= RCTRL_HUMI_HIGH  → 吸合（或逻辑，谁越限都动作）
 *   吸合之后要降回 (阈值 - 回差) 才断开 —— 这就是回差/滞回，
 *   不加回差的话，值在阈值上下抖一点，继电器会一秒一次地咔哒，触点很快烧
 * 【防抖】一次动作后至少保持 RCTRL_MIN_HOLD_MS 才允许再变（保护触点）
 * 【异常】这一轮 DHT11 读失败/校验错 → 本轮不动作，保持现状，不拿陈旧数据做判断
 * 【扩展】将来接云端控制：Relay_Ctrl_SetLocalEnable(0) 让本地联动让位，
 *         云端自己调 Relay_On/Relay_Off，两边不会互相抢
 *
 * 阈值按你的用途改：降温（风机）用温度高值吸合；只关心一路的话把另一个 USE 置 0
 */
#define RCTRL_USE_TEMP       1        /* 启用温度联动 1/0 */
#define RCTRL_TEMP_HIGH     28        /* 温度吸合阈值：℃（超过就开风扇） */
#define RCTRL_TEMP_HYST      2        /* 温度回差：℃（吸合后降到 26 才断开，防卡在 28 上反复咔哒） */

#define RCTRL_USE_HUMI       1        /* 启用湿度联动 1/0 */
#define RCTRL_HUMI_HIGH     80        /* 湿度吸合阈值：%RH */
#define RCTRL_HUMI_HYST      5        /* 湿度回差：%（吸合后降到 75 才断开） */

#define RCTRL_MIN_HOLD_MS   10000UL   /* 一次动作后最短保持时间：ms */

void    Relay_Ctrl_Init(void);
void    Relay_Ctrl_Task(uint8_t dht_ok, uint8_t temp, uint8_t humi);
uint8_t Relay_Ctrl_IsOn(void);

/* 云端接管的接口 */
void    Relay_Ctrl_SetLocalEnable(uint8_t en);
uint8_t Relay_Ctrl_IsLocalEnabled(void);
void    Relay_Ctrl_SyncFromHw(uint8_t hw_on);   /* 交还控制权前先对齐硬件实际状态 */

#endif
