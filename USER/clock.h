#ifndef __CLOCK_H
#define __CLOCK_H

#include "sys.h"
#include "ds1302.h"

/**
 * @file    clock.h
 * @brief   系统时钟：毫秒节拍 + 软时钟 + 网络授时管理
 *
 * 【时间来源优先级】
 *   1. 网络授时（ESP8266：先 AT+CIPSNTPCFG/AT+CIPSNTPTIME，失败再试 HTTP 的 Date 头）
 *   2. 本机 DS1302（网络拿不到时用）
 *
 *   网络授时成功后立即把时间写回 DS1302，
 *   所以断网 / 下电再上电，RTC 也是准的。
 *
 * 【防卡死】
 *   全模块非阻塞：只有状态推进和计时，没有 while(1) 死等；
 *   网络侧每一步都有 tick 超时，超时就走 DS1302，绝不吊死主循环。
 */

typedef enum {
    CLOCK_SRC_RTC = 0,   /* 时间来自本机 DS1302 */
    CLOCK_SRC_NET        /* 时间来自网络授时 */
} ClockSource;

/* 授时成功后的再校周期 / 失败后的重试间隔 */
#define CLOCK_SYNC_PERIOD_MS   (30UL * 60UL * 1000UL)   /* 30 分钟 */
#define CLOCK_SYNC_RETRY_MS    (60UL * 1000UL)          /* 60 秒   */

/* DS1302 时间无效时写入的默认时间（改成你要的出厂时间） */
#define CLOCK_DEFAULT_YEAR     26
#define CLOCK_DEFAULT_MONTH    5
#define CLOCK_DEFAULT_DAY      29
#define CLOCK_DEFAULT_HOUR     1
#define CLOCK_DEFAULT_MINUTE   16
#define CLOCK_DEFAULT_SECOND   0

void        Clock_Init(void);            /* 毫秒节拍 + 载入 DS1302 + 有效性检查 */
void        Clock_Task(void);            /* 主循环调用，推进软时钟（非阻塞） */
uint32_t    Clock_Millis(void);          /* 开机至今毫秒数（不回绕判断用差值） */

void        Clock_GetTime(DS1302_TIME *t);   /* 取当前显示时间 */
ClockSource Clock_GetSource(void);
uint8_t     Clock_IsSynced(void);        /* 是否成功网络授时过 */

uint8_t     Clock_NetSyncDue(void);      /* 现在需要授时吗 */
void        Clock_NetSyncTry(void);      /* 记一次"开始尝试" */
void        Clock_NetSyncOk(const DS1302_TIME *t);   /* 授时成功，时间生效 */
void        Clock_NetSyncFail(void);     /* 授时失败，继续用 RTC（并退避 CLOCK_SYNC_RETRY_MS 再试） */

#endif
