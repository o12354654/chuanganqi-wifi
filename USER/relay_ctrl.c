/***************STM32F103C8T6**********************
 * 文件名  ：relay_ctrl.c
 * 功能    ：继电器本地联动（温湿度越限 → 吸合），带回差 + 最短保持时间
 * 备注    ：只决定"何时该吸合"，实际开关交给 relay.c；
 *           将来云端控制挂在同一个 relay.c 上，两边用 local_en 分权
 * 接口    ：主循环周期调用 Relay_Ctrl_Task()（当前约 1 秒一次）

********************LIGEN*************************/

#include "relay_ctrl.h"
#include "relay.h"
#include "clock.h"

static uint8_t  s_on       = 0;   /* 联动判定的输出状态 */
static uint8_t  s_local_en = 1;   /* 本地联动是否生效（云端接管时置 0） */
static uint32_t s_last_ms  = 0;   /* 上一次状态改变的时刻 */

/**
 * @brief  单个量是否越限
 * @param  value 当前值
 * @param  high  吸合阈值
 * @param  hyst  回差
 * @param  on    继电器当前是否已吸合
 * @note   已吸合 → 要低于 (阈值 - 回差) 才算不再越限；
 *         未吸合 → 到阈值就算越限。
 *         一进一出两个门限就是滞回，避免卡在阈值上反复动作。
 */
static uint8_t ctrl_over_limit(uint8_t value, uint8_t high, uint8_t hyst, uint8_t on)
{
    if (on) {
        return (uint8_t)(value >= (uint8_t)(high - hyst));
    }
    return (uint8_t)(value >= high);
}

/* 综合温湿度，判断"现在该不该吸合" */
static uint8_t ctrl_want_on(uint8_t temp, uint8_t humi)
{
    uint8_t want = 0;

#if RCTRL_USE_TEMP
    if (ctrl_over_limit(temp, RCTRL_TEMP_HIGH, RCTRL_TEMP_HYST, s_on)) {
        want = 1;
    }
#else
    (void)temp;
#endif

#if RCTRL_USE_HUMI
    if (ctrl_over_limit(humi, RCTRL_HUMI_HIGH, RCTRL_HUMI_HYST, s_on)) {
        want = 1;
    }
#else
    (void)humi;
#endif

    return want;
}

/**
 * @brief  联动初始化：上电默认断开，并记下起始时刻
 * @note   要在 Clock_Init() 之后调用（要用毫秒计数）
 */
void Relay_Ctrl_Init(void)
{
    s_on       = 0;
    s_local_en = 1;
    s_last_ms  = Clock_Millis() - RCTRL_MIN_HOLD_MS;   /* 往前拨一个保持周期：
                                                          开机读到越限就能立即动作，
                                                          不被"最短保持时间"挡住 */

    Relay_Off();
}

/**
 * @brief  本地联动任务（主循环调用）
 * @param  dht_ok 本次 DHT11 读数是否有效：0 = 无效，本轮跳过
 * @param  temp   温度 ℃
 * @param  humi   湿度 %RH
 */
void Relay_Ctrl_Task(uint8_t dht_ok, uint8_t temp, uint8_t humi)
{
    uint32_t now;
    uint8_t  want;

    if (!s_local_en) {
        return;                                 /* 云端接管中，本地不插手 */
    }
    if (!dht_ok) {
        return;                                 /* 这轮数据不可信 → 保持现状 */
    }

    want = ctrl_want_on(temp, humi);
    if (want == s_on) {
        return;                                 /* 该保持，不动 */
    }

    now = Clock_Millis();
    if ((uint32_t)(now - s_last_ms) < RCTRL_MIN_HOLD_MS) {
        return;                                 /* 最短保持时间没到，压住 */
    }

    if (want) {
        Relay_On();
    } else {
        Relay_Off();
    }

    s_on      = want;
    s_last_ms = now;
}

uint8_t Relay_Ctrl_IsOn(void)
{
    return s_on;
}

/**
 * @brief  本地联动开关（将来云端接管时用）
 * @note   关掉本地联动后，继电器的状态由调用方自己负责
 */
void Relay_Ctrl_SetLocalEnable(uint8_t en)
{
    s_local_en = en ? 1 : 0;

    if (!s_local_en) {
        s_last_ms = Clock_Millis();             /* 交还控制权时重新计时 */
    }
}

uint8_t Relay_Ctrl_IsLocalEnabled(void)
{
    return s_local_en;
}
