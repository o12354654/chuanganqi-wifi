/**
 * @file    clock.c
 * @brief   系统时钟：TIM2 毫秒节拍 + 软时钟 + 网络授时管理
 * @note    TIM2 中断只做一件事：s_ms++（1ms 一次），
 *          ISR 极短，不会影响 DHT11 的时序。
 */

#include "clock.h"
#include "timeparse.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include <stdio.h>

/* ===================== 内部数据 ===================== */
static volatile uint32_t s_ms      = 0;      /* 开机毫秒计数（1ms 中断里 +1） */
static DS1302_TIME       s_time;             /* 当前时间（软时钟） */
static ClockSource       s_src     = CLOCK_SRC_RTC;
static uint8_t           s_synced  = 0;      /* 成功网络授时过 */
static uint8_t           s_tried   = 0;      /* 发起过授时尝试 */
static uint32_t          s_last_ms = 0;      /* 上次进位到整秒的时刻 */
static uint32_t          s_last_try = 0;     /* 上次发起授时 */
static uint32_t          s_last_ok  = 0;     /* 上次授时成功 */

/* ===================== 1ms 节拍 ===================== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        s_ms++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

static void Clock_Tick_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef        nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* 72MHz / 7200 = 10kHz，计 10 个数 = 1ms */
    tim.TIM_Prescaler         = 7200 - 1;
    tim.TIM_Period            = 10 - 1;
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* 优先级比 USART2(0) 低，不抢串口中断 */
    nvic.NVIC_IRQChannel                   = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
}

uint32_t Clock_Millis(void)
{
    return s_ms;
}

/* ===================== 初始化 ===================== */
void Clock_Init(void)
{
    Clock_Tick_Init();
    DS1302_Init();
    DS1302_GetTime(&s_time);

    /* DS1302 时间明显不对(新片/掉电/停振) → 写一个默认时间进去 */
    if (!TimeParse_IsValid(&s_time) || DS1302_IsHalt()) {
        s_time.year   = CLOCK_DEFAULT_YEAR;
        s_time.month  = CLOCK_DEFAULT_MONTH;
        s_time.day    = CLOCK_DEFAULT_DAY;
        s_time.hour   = CLOCK_DEFAULT_HOUR;
        s_time.minute = CLOCK_DEFAULT_MINUTE;
        s_time.second = CLOCK_DEFAULT_SECOND;
        s_time.week   = TimeParse_Weekday((uint16_t)(2000 + s_time.year),
                                          s_time.month, s_time.day);
        if (s_time.week == 0) s_time.week = 7;
        DS1302_SetTime(&s_time);      /* 顺带清 CH 停振位，让 RTC 走起来 */
        printf("CLOCK: RTC invalid, set default 20%02d/%02d/%02d %02d:%02d:%02d\r\n",
               s_time.year, s_time.month, s_time.day,
               s_time.hour, s_time.minute, s_time.second);
    }

    s_src      = CLOCK_SRC_RTC;
    s_last_ms  = Clock_Millis();
    s_last_ok  = 0;

    printf("CLOCK: start from RTC 20%02d/%02d/%02d %02d:%02d:%02d\r\n",
           s_time.year, s_time.month, s_time.day,
           s_time.hour, s_time.minute, s_time.second);
}

/* ===================== 走时（非阻塞） ===================== */
void Clock_Task(void)
{
    uint32_t now     = Clock_Millis();
    uint32_t elapsed = now - s_last_ms;   /* 无符号差值，天然处理回绕 */
    uint32_t secs;

    if (elapsed < 1000) return;

    secs = elapsed / 1000;
    if (secs > 3600) {          /* 被异常阻塞超过 1 小时：不空转，直接对齐 */
        secs      = 3600;
        s_last_ms = now;
    } else {
        s_last_ms += secs * 1000;
    }

    while (secs--) {
        TimeParse_AddSecond(&s_time);
    }
}

/* ===================== 查询 ===================== */
void Clock_GetTime(DS1302_TIME *t)
{
    if (t) *t = s_time;
}

ClockSource Clock_GetSource(void)
{
    return s_src;
}

uint8_t Clock_IsSynced(void)
{
    return s_synced;
}

/* ===================== 授时调度 ===================== */
uint8_t Clock_NetSyncDue(void)
{
    uint32_t now = Clock_Millis();

    if (!s_tried)  return 1;                              /* 上电第一次 */

    /* 退避先行：只判"距上次成功"的话，失败路径每一轮都满足条件 ——
       SNTP 拿不到应答（超时或解析失败）时变成探测风暴，
       而探测窗口内 MQTT 收发要让开，上报/心跳一起停摆，
       60s 后 keepalive 到期被 broker 踢，表现成"WiFi 明明是好的却反复重连"。
       所以不论成功失败，两次尝试之间至少隔 CLOCK_SYNC_RETRY_MS */
    if ((now - s_last_try) < CLOCK_SYNC_RETRY_MS) return 0;

    if (s_synced)  return ((now - s_last_ok) >= CLOCK_SYNC_PERIOD_MS) ? 1 : 0;
    return 1;
}

void Clock_NetSyncTry(void)
{
    s_tried    = 1;
    s_last_try = Clock_Millis();
}

void Clock_NetSyncOk(const DS1302_TIME *t)
{
    if (!TimeParse_IsValid(t)) {
        printf("CLOCK: net time invalid, ignore\r\n");
        return;
    }

    s_time    = *t;
    s_src     = CLOCK_SRC_NET;
    s_synced  = 1;
    s_last_ok = Clock_Millis();
    s_last_ms = s_last_ok;

    DS1302_SetTime(&s_time);   /* 写回本机 RTC：断网后接着走，也顺便校准 */

    printf("CLOCK: net time OK 20%02d/%02d/%02d %02d:%02d:%02d (week %d)\r\n",
           s_time.year, s_time.month, s_time.day,
           s_time.hour, s_time.minute, s_time.second, s_time.week);
}

void Clock_NetSyncFail(void)
{
    s_last_try = Clock_Millis();
    printf("CLOCK: net time fail, keep %s\r\n",
           (s_src == CLOCK_SRC_NET) ? "net time" : "RTC time");
}
