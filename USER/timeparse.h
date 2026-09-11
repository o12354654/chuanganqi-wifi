#ifndef __TIMEPARSE_H
#define __TIMEPARSE_H

#include "sys.h"
#include "ds1302.h"

/**
 * @file    timeparse.h
 * @brief   网络时间字符串解析（纯逻辑，不碰硬件）
 * @note    单独成模块的目的：可以拿到 PC 上用 gcc 单机测试，
 *          不用烧板子就能验证解析算法。
 *
 *   SNTP : "+CIPSNTPTIME:Fri Sep 11 12:34:56 2026"   已是本地时间(时区已在 AT 里配好)
 *   HTTP : "Date: Fri, 11 Sep 2026 04:52:14 GMT"     服务器给的是 GMT，需 +8 小时
 */

/* 时间是否合理（月/日/时/分/秒范围） */
uint8_t TimeParse_IsValid(const DS1302_TIME *t);

/* 某年某月的天数（含闰年） */
uint8_t TimeParse_DaysInMonth(uint16_t year, uint8_t month);

/* 星期：0=周日 … 6=周六（Sakamoto 算法） */
uint8_t TimeParse_Weekday(uint16_t year, uint8_t month, uint8_t day);

/* 解析 SNTP 时间串，成功返回 1 */
uint8_t TimeParse_SntpTime(const char *s, DS1302_TIME *t);

/* 解析 HTTP 响应里的 Date 头，自动转北京时间(+8)，成功返回 1 */
uint8_t TimeParse_HttpDate(const char *s, DS1302_TIME *t);

/* 时间加 1 小时（带日/月/年进位，供时区转换用） */
void TimeParse_AddHour(DS1302_TIME *t);

/* 时间加 1 秒（带分/时/日/月/年进位，软时钟走时用） */
void TimeParse_AddSecond(DS1302_TIME *t);

#endif
