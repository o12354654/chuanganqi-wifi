/***************STM32F103C8T6**********************
 * 文件名  ：relay_cloud.c
 * 功能    ：云端（OneNET 属性下发）控制继电器 + 与本地联动的分权
 * 备注    ：只解析 JSON、决定开关，实际动作交给 relay.c；
 *           云端接管/交还的开关状态由 relay_ctrl.c 维护
 * 接口    ：esp8266.c 收到 property/set 后调 Relay_Cloud_HandleSet()

********************LIGEN*************************/

#include "relay_cloud.h"
#include "relay.h"
#include "relay_ctrl.h"
#include <string.h>

static uint8_t s_remote_active = 0;   /* 云端是否已接管 */

/**
 * @brief  在 JSON 里取某个属性的值，只认布尔语义
 * @param  json OneJSON 片段，形如 {"id":"1","params":{"relay":{"value":true}}}
 * @param  key  属性标识符，如 "relay"
 * @param  out  出参：1 = 吸合，0 = 断开
 * @return 1 = 找到并解析成功
 * @note   找 key 之后第一处 "value"，跳过 ':' 和空白再取值；
 *         兼容 true/false、1/0、on/off，也兼容带引号的字符串形式
 */
static uint8_t cloud_json_get_bool(const char *json, const char *key, uint8_t *out)
{
    char        pat[24];
    const char *p;
    const char *v;
    uint16_t    klen = (uint16_t)strlen(key);

    if (!json || !key || klen == 0) return 0;
    if ((uint16_t)(klen + 3) > sizeof(pat)) return 0;

    pat[0] = '"';
    memcpy(pat + 1, key, klen);
    pat[klen + 1] = '"';
    pat[klen + 2] = '\0';

    p = strstr(json, pat);
    if (!p) return 0;
    p += klen + 2;

    v = strstr(p, "value");
    if (!v) return 0;
    v += 5;
    /* OneJSON 里是 {"value":true}，value 后面依次是 " 和 : ，
       两个都要跳；带引号的字符串形式（"value":"true"）同样跳得过去 */
    while (*v == ' ' || *v == '\t' || *v == ':' || *v == '"') v++;

    if (strncmp(v, "true",  4) == 0 || *v == '1' || strncmp(v, "on",  2) == 0) { *out = 1; return 1; }
    if (strncmp(v, "false", 5) == 0 || *v == '0' || strncmp(v, "off", 3) == 0) { *out = 0; return 1; }

    return 0;
}

/**
 * @brief  按候选标识符表逐个试（RELAY_CLOUD_PROP_KEYS，用 | 分隔）
 */
static uint8_t cloud_lookup(const char *json, uint8_t *out)
{
    const char *p = RELAY_CLOUD_PROP_KEYS;
    char        key[16];
    uint8_t     n;

    while (*p) {
        n = 0;
        while (*p && *p != '|' && n < (uint8_t)(sizeof(key) - 1)) {
            key[n++] = *p++;
        }
        key[n] = '\0';
        if (*p == '|') p++;

        if (n && cloud_json_get_bool(json, key, out)) return 1;
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

    (void)len;

    if (!json) return 0;
    if (!cloud_lookup(json, &v)) return 0;

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
    Relay_Ctrl_SetLocalEnable(1);   /* 交还本地联动：断网了也得能按阈值自动动作 */
}

uint8_t Relay_Cloud_IsRemoteActive(void)
{
    return s_remote_active;
}
