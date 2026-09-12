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
 * @brief  取某个属性的布尔值
 * @param  json/len 负载（len = 有效字节数；只在这个范围内搜索，不会越读）
 * @param  key      属性标识符，如 "relay"
 * @param  out      出参：1 = 吸合，0 = 断开
 * @return 1 = 找到并解析成功
 * @note   只认这一种形状：..."key" : { ... "value" : <true|false|1|0|on|off> ... }
 *         ① 键名必须是完整的键（前一个非空白字符是 { 或 ,）——
 *            这样 {"mode":{"value":"relay"}} 这种"值里出现 relay"不会误命中
 *         ② value 必须落在该属性自己的 {} 里面 ——
 *            不会拿后面别的属性的 value 顶替（原先是"key 之后第一处 value"，会串台）
 *         ③ 字面量大小写不敏感，但必须完整匹配（TRUE/On 认，truey/offline/12 不认）
 *         ④ 同一个键出现两次时取先出现的那个（保守一侧）
 */
static uint8_t cloud_json_get_bool(const char *json, uint16_t len,
                                   const char *key, uint8_t *out)
{
    static char pat[64];      /* 栈紧张，一律 static（这两个函数不会重入） */
    const char *end;
    const char *p, *q, *v;
    const char *obj, *obj_end;
    uint16_t    klen = (uint16_t)strlen(key);
    uint32_t    depth;

    if (!json || !key || klen == 0) return 0;
    if ((uint16_t)(klen + 3) > sizeof(pat)) return 0;
    if (len == 0) len = (uint16_t)strlen(json);   /* 兼容不传长度的调用 */
    end = json + len;

    pat[0] = '"';
    memcpy(pat + 1, key, klen);
    pat[klen + 1] = '"';
    pat[klen + 2] = '\0';

    /* ① 找"作为键"出现的 key */
    p = json;
    for (;;) {
        const char *hit = mem_find(p, end, pat);
        const char *back;

        if (!hit) return 0;

        back = hit;
        while (back > json && (back[-1] == ' ' || back[-1] == '\t')) back--;
        if (back > json && (back[-1] == '{' || back[-1] == ',')) {
            p = hit;                       /* 认这个键：从它的位置往下解析（漏了这句会跑去解析别的属性） */
            break;
        }
        p = hit + 1;                       /* 只是字符串值里出现了同名文字，继续往后找 */
    }

    /* ② 定位到该属性自己的对象： "key" : { ... } */
    q = skip_ws(p + klen + 2, end);
    if (q >= end || *q != ':') return 0;
    q = skip_ws(q + 1, end);
    if (q >= end || *q != '{') return 0;             /* 不是 {"key":{"value":...}} 形状 */

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

    /* ③ 在对象内部找 value 键 */
    v = mem_find(obj, obj_end, "\"value\"");
    if (!v) return 0;
    v = skip_ws(v + 7, obj_end);
    if (v >= obj_end || *v != ':') return 0;
    v = skip_ws(v + 1, obj_end);
    if (v < obj_end && *v == '"') v = skip_ws(v + 1, obj_end);   /* "value":"true" 这种写法 */

    /* ④ 取值 */
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
