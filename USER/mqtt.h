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

uint8_t  MQTT_BuildPingReq(uint8_t *buf);
uint8_t  MQTT_BuildDisconnect(uint8_t *buf);
uint8_t  MQTT_ParseConnack(uint8_t *data, uint16_t len);
uint8_t  MQTT_CheckType(uint8_t *data, uint8_t expected_type);

#endif
