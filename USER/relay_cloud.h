#ifndef __RELAY_CLOUD_H
#define __RELAY_CLOUD_H

#include "sys.h"

/* ============== 云端控制继电器（OneNET 属性下发）==============
 * 链路：OneNET 控制台/APP 下发
 *         → topic $sys/<pid>/<dev>/thing/property/set
 *         → esp8266.c 收到 +IPD，mqtt.c 解出 JSON
 *         → 本模块取值、分权 → relay.c 开关
 *
 * 【分权】云端下发过一次之后，本地温湿度联动就让位（Relay_Ctrl_SetLocalEnable(0)），
 *         两边不会互相抢同一个继电器；
 *         MQTT 掉线/重连 → 立刻把控制权交还本地联动（fail-safe），
 *         避免断网后继电器卡死在云端最后一条命令上。
 *
 * 【属性标识符】云端物模型里"继电器开关"那个属性的标识符，
 *         在下面按 | 分隔列候选，命中任意一个都认（默认 relay）；
 *         你在 OneNET 建属性时叫什么，留一个在这儿就行。
 */
#define RELAY_CLOUD_PROP_KEYS   "relay|power|switch|fan"

/* 下发值 true/1/on → 吸合；false/0/off → 断开 */

void    Relay_Cloud_Init(void);

/**
 * @brief  处理一条云端属性设置 JSON
 * @param  json/len 负载（OneJSON：{"id":"1","params":{"relay":{"value":true}}}）
 * @return 1 = 里面确实有本设备认的继电器属性，已执行；0 = 无关/解析不了
 * @note   只认领 + 执行，不负责回 set_reply 和 PUBACK（那是 esp8266.c 的事）
 */
uint8_t Relay_Cloud_HandleSet(const char *json, uint16_t len);

/* MQTT 掉线时调用：把控制权交还本地联动（可重复调用） */
void    Relay_Cloud_OnMqttLost(void);

/* 现在是不是云端在管 */
uint8_t Relay_Cloud_IsRemoteActive(void);

#endif
