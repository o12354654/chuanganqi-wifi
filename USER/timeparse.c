/**
 * @file    timeparse.c
 * @brief   网络时间字符串解析（纯逻辑，不碰硬件，可单机测试）
 * @note    全文件没有 while(1)、没有硬件寄存器操作，永远不会卡死
 */

#include "timeparse.h"
#include <string.h>

/* 月份英文名表 */
static const char *const s_mon_name[12] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

/* 每月天数（闰年 2 月另行 +1） */
static const uint8_t s_mon_days[12] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

static uint8_t is_digit(char c)
{
    return (uint8_t)(c >= '0' && c <= '9');
}

static uint8_t is_alpha(char c)
{
    return (uint8_t)((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'));
}

static uint8_t is_alnum(char c)
{
    return (uint8_t)(is_digit(c) || is_alpha(c));
}

uint8_t TimeParse_DaysInMonth(uint16_t year, uint8_t month)
{
    uint8_t d;

    if (month < 1 || month > 12) return 0;
    d = s_mon_days[month - 1];

    if (month == 2) {
        /* 闰年：能整除 4 且（不能整除 100 或能整除 400） */
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
            d = 29;
        }
    }
    return d;
}

uint8_t TimeParse_Weekday(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    uint16_t y = year;

    if (month < 3) y -= 1;
    return (uint8_t)((y + y / 4 - y / 100 + y / 400 + t[month - 1] + day) % 7);
}

uint8_t TimeParse_IsValid(const DS1302_TIME *t)
{
    if (t == 0) return 0;
    if (t->year > 99)                 return 0;
    if (t->month < 1 || t->month > 12) return 0;
    if (t->day < 1)                    return 0;
    if (t->day > TimeParse_DaysInMonth((uint16_t)(2000 + t->year), t->month)) return 0;
    if (t->hour > 23)                  return 0;
    if (t->minute > 59)                return 0;
    if (t->second > 59)                return 0;
    return 1;
}

/* 日进位：星期翻转 + 日/月/年进位 */
static void bump_day(DS1302_TIME *t)
{
    t->week = (uint8_t)(t->week % 7 + 1);
    t->day++;
    if (t->day <= TimeParse_DaysInMonth((uint16_t)(2000 + t->year), t->month)) return;

    t->day = 1;
    t->month++;
    if (t->month <= 12) return;

    t->month = 1;
    t->year = (uint8_t)((t->year + 1) % 100);
}

void TimeParse_AddHour(DS1302_TIME *t)
{
    t->hour++;
    if (t->hour < 24) return;

    t->hour = 0;
    bump_day(t);
}

void TimeParse_AddSecond(DS1302_TIME *t)
{
    t->second++;
    if (t->second < 60) return;

    t->second = 0;
    t->minute++;
    if (t->minute < 60) return;

    t->minute = 0;
    t->hour++;
    if (t->hour < 24) return;

    t->hour = 0;
    bump_day(t);
}

/* 取下一个单词（字母数字连续串），返回长度 */
static uint8_t next_token(const char **pp, char *out, uint8_t maxlen)
{
    const char *p = *pp;
    uint8_t     n = 0;

    while (*p && !is_alnum(*p)) p++;
    while (*p && is_alnum(*p)) {
        if (n < (uint8_t)(maxlen - 1)) out[n++] = *p;
        p++;
    }
    out[n] = '\0';
    *pp = p;
    return n;
}

/* "Sep" → 9，不匹配返回 0 */
static uint8_t month_by_name(const char *tok)
{
    uint8_t i;

    for (i = 0; i < 12; i++) {
        if (tok[0] == s_mon_name[i][0] &&
            tok[1] == s_mon_name[i][1] &&
            tok[2] == s_mon_name[i][2]) {
            return (uint8_t)(i + 1);
        }
    }
    return 0;
}

/* 在串里抠出 hh:mm:ss，找到返回 1 */
static uint8_t pick_clock(const char *s, uint8_t *h, uint8_t *m, uint8_t *sec)
{
    const char *p = s;
    uint8_t     guard = 0;

    while (*p && guard++ < 200) {
        if (is_digit(p[0]) && is_digit(p[1]) && p[2] == ':' &&
            is_digit(p[3]) && is_digit(p[4]) && p[5] == ':' &&
            is_digit(p[6]) && is_digit(p[7]) && !is_digit(p[8])) {
            *h   = (uint8_t)((p[0] - '0') * 10 + (p[1] - '0'));
            *m   = (uint8_t)((p[3] - '0') * 10 + (p[4] - '0'));
            *sec = (uint8_t)((p[6] - '0') * 10 + (p[7] - '0'));
            return 1;
        }
        p++;
    }
    return 0;
}

/**
 * @brief  从一堆 token 里挑出 月 / 日 / 年
 * @note   两种格式的 token 顺序不同（SNTP 是 月 日 年，HTTP 是 日 月 年），
 *         所以这里不看位置、只看内容：三字母且是月份名 → 月；
 *         4 位数字 → 年；1~2 位数字 → 日。
 *         配了 max_tok 上限，避免在长响应里被后面的数字带偏。
 */
static uint8_t pick_date(const char *s, uint8_t *mo, uint8_t *dy, uint16_t *yr)
{
    const char *p = s;
    char        tok[10];
    uint8_t     n, guard = 0, has_mo = 0, has_dy = 0, has_yr = 0;

    while (*p && guard++ < 12) {
        n = next_token(&p, tok, sizeof(tok));
        if (n == 0) break;

        if (n == 3 && !has_mo) {
            uint8_t m = month_by_name(tok);
            if (m) { *mo = m; has_mo = 1; continue; }
        }
        if (n == 4 && !has_yr) {
            *yr = (uint16_t)((tok[0] - '0') * 1000 + (tok[1] - '0') * 100 +
                             (tok[2] - '0') * 10 + (tok[3] - '0'));
            has_yr = 1;
            continue;
        }
        if ((n == 1 || n == 2) && !has_dy) {
            *dy = (n == 1) ? (uint8_t)(tok[0] - '0')
                           : (uint8_t)((tok[0] - '0') * 10 + (tok[1] - '0'));
            has_dy = 1;
            continue;
        }
    }
    return (uint8_t)(has_mo && has_dy && has_yr);
}

/* 解析入口：返回 1 表示填好了 t */
static uint8_t parse_common(const char *s, DS1302_TIME *t, uint8_t is_gmt)
{
    uint8_t  mo = 0, dy = 0, hh = 0, mi = 0, ss = 0;
    uint16_t yr = 0;
    uint8_t  i;

    if (s == 0 || t == 0) return 0;

    if (!pick_clock(s, &hh, &mi, &ss))              return 0;
    if (!pick_date(s, &mo, &dy, &yr))               return 0;
    if (yr < 2020 || yr > 2099)                     return 0;   /* 年份不合理直接丢 */
    if (hh > 23 || mi > 59 || ss > 59)              return 0;

    t->year   = (uint8_t)(yr % 100);
    t->month  = mo;
    t->day    = dy;
    t->hour   = hh;
    t->minute = mi;
    t->second = ss;
    t->week   = (uint8_t)(TimeParse_Weekday(yr, mo, dy) == 0
                          ? 7 : TimeParse_Weekday(yr, mo, dy));

    if (!TimeParse_IsValid(t)) return 0;

    if (is_gmt) {
        for (i = 0; i < 8; i++) TimeParse_AddHour(t);   /* GMT → 北京时间 */
    }
    return 1;
}

uint8_t TimeParse_SntpTime(const char *s, DS1302_TIME *t)
{
    /* "+CIPSNTPTIME:Fri Sep 11 12:34:56 2026"
       时区已经在 AT+CIPSNTPCFG 里配成 +8，拿到的就是本地时间。
       先定位 "+CIPSNTPTIME"，缓冲区里前面的杂字符（AT 回显等）不会干扰解析 */
    const char *p = s;
    const char *q;

    if (p) {
        q = strstr(p, "+CIPSNTPTIME");
        if (q) p = q + 12;
    }
    return parse_common(p, t, 0);
}

uint8_t TimeParse_HttpDate(const char *s, DS1302_TIME *t)
{
    /* "Date: Fri, 11 Sep 2026 04:52:14 GMT" 是 GMT，需要 +8 */
    const char *p = s;
    const char *q;

    if (p) {
        q = strstr(p, "Date:");
        if (q) p = q + 5;
    }
    return parse_common(p, t, 1);
}
