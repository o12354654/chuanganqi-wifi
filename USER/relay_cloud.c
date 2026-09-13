/***************STM32F103C8T6**********************
 * 文件名  ：relay_cloud.c
 * 功能    ：云端（OneNET 属性下发）控制继电器 + 与本地联动的分权
 * 备注    ：只解析 JSON、决定开关，实际动作交给 relay.c；
 *           云端接管/交还的开关状态由 relay_ctrl.c 维护
 * 接口    ：esp8266.c 收到 property/set 后调 Relay_Cloud_HandleSet()

****************

****LIGEN

****************

*********/

#include "relay_cloud.h"
#include "relay.h"
#include "relay_ctrl.h"
#include <string.h>

static uint8_t s_remote_active = 0;   /* 云端是否已接管 */

/* ================= 受限搜索：全部按 [p, end) 范围操作，不依赖 '\0' =================
 * 调用方给的是接收缓冲里的一段（可能不带结尾符），所以不能用 strstr/strncmp
 * 那套"遇到 '\0' 才停"的函数 —— 越界读就是这么来的
 */

static const char *skip_ws(const char *p, const char *end)
{
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) p++;
    return p;
}

static const char *mem_find(const char *hay, const char *end, const char *needle)
{
    uint16_t    nlen = (uint16_t)strlen(needle);
    const char *p;

    if (nlen == 0 || (uint32_t)(end - hay) < nlen) return NULL;
    for (p = hay; (uint32_t)(end - p) >= nlen; p++) {
        if (memcmp(p, needle, nlen) == 0) return p;
    }
    return NULL;
}

/**
 * @brief  大小写不敏感地匹配一个字面量，并要求它到此结束
 * @note   结束判定是重点：不检查的话 "offline" 会被当成 off、"one" 会被当成 on、
 *         12 会被当成 1、0.5 会被当成 0 —— 语义完全变了
 */
static uint8_t match_literal(const char **pp, const char *end, const char *lit)
{
    const char *p = *pp;

    while (*lit) {
        char c;
        if (p >= end) return 0;
        c = *p++;
        if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        if (c != *lit++) return 0;
    }
    if (p < end && !(*p == ',' || *p == '}' || *p == ' ' || *p == '\t' ||
                     *p == '\r' || *p == '\n' || *p == '"')) {
        return 0;
    }
    *pp = p;
    return 1;
}

/**
 * @brief  定位"作为键出现的 key"（形状不限），返回它的位置；找不到返回 NULL
 * @note   键名必须完整：前一个非空白字符是 { 或 , ——
 *         否则 {"mode":{"value":"relay"}} 这种"值里出现 relay"也会被当成命中
 */
static const char *find_key(const char *json, const char *end, const char *key)
{
    static char pat[64];      /* 栈紧张，一律 static（本文件这几个函数不会重入） */
    const char *p = json;
    uint16_t    klen = (uint16_t)strlen(key);

    if (!key || klen == 0) return NULL;
    if ((uint16_t)(klen + 3) > sizeof(pat)) return NULL;

    pat[0] = '\"';
    memcpy(pat + 1, key, klen);
    pat[klen + 1] = '\"';
    pat[klen + 2] = '\0';

    for (;;) {
        const char *hit = mem_find(p, end, pat);
        const char *back;

        if (!hit) return NULL;

        back = hit;
        while (back > json && (back[-1] == ' ' || back[-1] == '\t')) back--;
        if (back > json && (back[-1] == '{' || back[-1] == ',')) return hit;   /* 是完整的键 */
        p = hit + 1;                       /* 只是字符串值里出现了同名文字，继续往后找 */
    }
}

/**
 * @brief  形状①：带 value 一层（设备上报/部分平台版本用）
 *              "key" : { ... "value" : <true|false|1|0|on|off> ... }
 * @note   value 必须落在该属性自己的 {} 里面 —— 不会拿后面别的属性的 value 顶替
 */
static uint8_t cloud_get_bool_wrapped(const char *json, const char *end,
                                      const char *key, uint8_t *out)
{
    const char *p, *q, *v, *obj, *obj_end;
    uint16_t    klen = (uint16_t)strlen(key);
    uint32_t    depth;

    p = find_key(json, end, key);
    if (!p) return 0;

    q = skip_ws(p + klen + 2, end);
    if (q >= end || *q != ':') return 0;
    q = skip_ws(q + 1, end);
    if (q >= end || *q != '{') return 0;             /* 不是 {\"key\":{...}} 形状，交给形状② */

    obj     = q + 1;
    depth   = 1;
    obj_end = obj;
    while (obj_end < end && depth > 0) {
        if (*obj_end == '{') depth++;
        else if (*obj_end == '}') depth--;
        if (depth == 0) break;
        obj_end++;
    }
    if (depth != 0) return 0;                        /* 对象没收全，不猜 */

    v = mem_find(obj, obj_end, "\"value\"");
    if (!v) return 0;
    v = skip_ws(v + 7, obj_end);
    if (v >= obj_end || *v != ':') return 0;
    v = skip_ws(v + 1, obj_end);
    if (v < obj_end && *v == '\"') v = skip_ws(v + 1, obj_end);   /* \"value\":\"true\" 这种写法 */

    if (match_literal(&v, obj_end, "true")  ||
        match_literal(&v, obj_end, "on")    ||
        match_literal(&v, obj_end, "1")) {
        *out = 1;
        return 1;
    }
    if (match_literal(&v, obj_end, "false") ||
        match_literal(&v, obj_end, "off")   ||
        match_literal(&v, obj_end, "0")) {
        *out = 0;
        return 1;
    }

    return 0;
}

/**
 * @brief  形状②：扁平（OneNET 平台的 property/set 下发就是这个形状）
 *              "key" : <true|false|1|0|on|off>     或   "key" : "true"
 * @note   平台下发和上报的形状不一样：上报 params 里是 {"value":...}，
 *         下发 params 里直接就是值。只认一种会"通信通了但继电器不动"
 */
static uint8_t cloud_get_bool_flat(const char *json, const char *end,
                                   const char *key, uint8_t *out)
{
    const char *p, *v;
    uint16_t    klen = (uint16_t)strlen(key);

    p = find_key(json, end, key);
    if (!p) return 0;

    v = skip_ws(p + klen + 2, end);
    if (v >= end || *v != ':') return 0;
    v = skip_ws(v + 1, end);
    if (v < end && *v == '\"') v = skip_ws(v + 1, end);   /* \"relay\": \"true\" */
    if (v < end && *v == '{') return 0;                   /* 是带 value 那层，交给形状① */

    if (match_literal(&v, end, "true")  ||
        match_literal(&v, end, "on")    ||
        match_literal(&v, end, "1")) {
        *out = 1;
        return 1;
    }
    if (match_literal(&v, end, "false") ||
        match_literal(&v, end, "off")   ||
        match_literal(&v, end, "0")) {
        *out = 0;
        return 1;
    }

    return 0;                        /* 认了键但值不是布尔：不猜，宁可回 404 */
}

/**
 * @brief  取某个属性的布尔值（两种形状都认）
 * @param  json/len 负载（len = 有效字节数；只在这个范围内搜索，不会越读）
 * @param  key      属性标识符，如 "relay"
 * @param  out      出参：1 = 吸合，0 = 断开
 * @return 1 = 找到并解析成功
 * @note   形状①（上报那种）: "key" : { ... "value" : <布尔> ... }
 *         形状②（下发那种）: "key" : <布尔>
 *         字面量大小写不敏感，但必须完整匹配（TRUE/On 认，truey/offline/12 不认）；
 *         同一个键出现两次时取先出现的那个（保守一侧）
 */
static uint8_t cloud_json_get_bool(const char *json, uint16_t len,
                                   const char *key, uint8_t *out)
{
    const char *end;

    if (!json || !key || !*key) return 0;
    if (len == 0) len = (uint16_t)strlen(json);   /* 兼容不传长度的调用 */
    end = json + len;

    if (cloud_get_bool_wrapped(json, end, key, out)) return 1;
    return cloud_get_bool_flat(json, end, key, out);
}

/**
 * @brief  按候选标识符表逐个试（RELAY_CLOUD_PROP_KEYS，用 | 分隔）
 * @note   候选名超过 key[] 长度就直接失败：宁可不动作，也不要拿被截断的名字去匹配
 */
static uint8_t cloud_lookup(const char *json, uint16_t len, uint8_t *out)
{
    const char *p = RELAY_CLOUD_PROP_KEYS;
    static char key[48];
    uint8_t     n;

    while (*p) {
        n = 0;
        while (*p && *p != '|' && n < (uint8_t)(sizeof(key) - 1)) {
            key[n++] = *p++;
        }
        key[n] = '\0';

        if (*p && *p != '|') return 0;    /* 候选名太长（配置错误），不再往下试 */
        if (*p == '|') p++;

        if (n && cloud_json_get_bool(json, len, key, out)) return 1;
    }
    return 0;
}

void Relay_Cloud_Init(void)
{
    s_remote_active = 0;      /* 上电本地联动说了算，等云端真的下发了再让位 */
}

uint8_t Relay_Cloud_HandleSet(const char *json, uint16_t len)
{
    uint8_t v;

    if (!json) return 0;
    if (!cloud_lookup(json, len, &v)) return 0;

    /* 先让位再动作：本地联动这一秒内就算读到越限也不会来抢 */
    Relay_Ctrl_SetLocalEnable(0);
    Relay_Set(v);
    s_remote_active = 1;

    return 1;
}

void Relay_Cloud_OnMqttLost(void)
{
    if (!s_remote_active) return;

    s_remote_active = 0;

    /* 交还之前先把本地缓存的状态对齐到硬件真实状态 ——
       云端接管期间继电器被直接开关过，本地完全不知情；不同步的话交还后
       本地会拿旧状态比较而卡住：该转不转（温度越限风扇不起），
       或者不该转一直转（环境恢复了风扇不停） */
    Relay_Ctrl_SyncFromHw(Relay_IsOn());
    Relay_Ctrl_SetLocalEnable(1);
}

uint8_t Relay_Cloud_IsRemoteActive(void)
{
    return s_remote_active;
}
