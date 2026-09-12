#ifndef __MQTT_H
#define __MQTT_H

#include "sys.h"

#define MQTT_CONNECT     0x10
#define MQTT_CONNACK     0x20
#define MQTT_PUBLISH_Q0  0x30
#define MQTT_PUBACK      0x40
#define MQTT_SUBSCRIBE   0x82
#define MQTT_SUBACK      0x90
#define MQTT_PINGREQ     0xC0
#define MQTT_PINGRESP    0xD0
#define MQTT_DISCONNECT  0xE0

uint16_t MQTT_BuildConnect(uint8_t *buf,
                           const char *client_id,
                           const char *username,
                           const char *password,
                           uint16_t keepalive);

uint16_t MQTT_BuildPublish(uint8_t *buf,
                           const char *topic,
                           const uint8_t *payload,
                           uint16_t payload_len);

uint16_t MQTT_BuildSubscribe(uint8_t *buf, const char *topic, uint16_t pkt_id);
uint16_t MQTT_BuildPubAck(uint8_t *buf, uint16_t pkt_id);

/**
 * @brief  解析下行 PUBLISH 报文
 * @param  data/len          一帧 MQTT 数据（来自 +IPD 的载荷）
 * @param  topic/topic_len   出参：topic（指向 data 内部，不拷贝，用完前别动缓冲）
 * @param  payload/len       出参：负载 JSON（指向 data 内部）
 * @param  pkt_id            出参：报文标识符（QoS0 时为 0）
 * @param  qos               出参：QoS 等级
 * @return 1 = 是 PUBLISH 且解析成功，0 = 不是/不完整
 * @note   返回的指针直接指进调用方的接收缓冲，调用方在发下一帧（会清缓冲）之前必须用完
 */
uint8_t  MQTT_ParsePublish(const uint8_t *data, uint16_t len,
                           const char **topic, uint16_t *topic_len,
                           const uint8_t **payload, uint16_t *payload_len,
                           uint16_t *pkt_id, uint8_t *qos);

uint8_t  MQTT_BuildPingReq(uint8_t *buf);
uint8_t  MQTT_BuildDisconnect(uint8_t *buf);
uint8_t  MQTT_ParseConnack(uint8_t *data, uint16_t len);
uint8_t  MQTT_CheckType(uint8_t *data, uint8_t expected_type);

#endif
