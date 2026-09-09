/**
 * @file    mqtt.c
 * @brief   MQTT v3.1.1 协议栈实现
 */

#include "mqtt.h"
#include <string.h>

static uint8_t mqtt_encode_len(uint32_t len, uint8_t *buf)
{
    uint8_t n = 0;
    do {
        uint8_t b = (uint8_t)(len % 128);
        len /= 128;
        if (len > 0) b |= 0x80;
        buf[n++] = b;
    } while (len > 0);
    return n;
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

uint16_t MQTT_BuildConnect(uint8_t *buf,
                           const char *client_id,
                           const char *username,
                           const char *password,
                           uint16_t keepalive)
{
    uint8_t  *p = buf;
    uint16_t  clen, ulen, plen;
    uint8_t   flags = 0x02;
    uint32_t  remaining;
    uint8_t   lebuf[4], lesz;

    clen = (uint16_t)strlen(client_id);
    ulen = username ? (uint16_t)strlen(username) : 0;
    plen = password ? (uint16_t)strlen(password) : 0;

    if (ulen) flags |= 0x80;
    if (plen) flags |= 0x40;

    remaining = 10;
    remaining += 2 + clen;
    if (ulen) remaining += 2 + ulen;
    if (plen) remaining += 2 + plen;

    lesz = mqtt_encode_len(remaining, lebuf);
    *p++ = MQTT_CONNECT;
    memcpy(p, lebuf, lesz); p += lesz;

    put16(p, 4);  p += 2;
    *p++ = 'M'; *p++ = 'Q'; *p++ = 'T'; *p++ = 'T';
    *p++ = 0x04;
    *p++ = flags;
    put16(p, keepalive); p += 2;

    put16(p, clen); p += 2;
    memcpy(p, client_id, clen); p += clen;

    if (ulen) {
        put16(p, ulen); p += 2;
        memcpy(p, username, ulen); p += ulen;
    }

    if (plen) {
        put16(p, plen); p += 2;
        memcpy(p, password, plen); p += plen;
    }

    return (uint16_t)(p - buf);
}

uint16_t MQTT_BuildPublish(uint8_t *buf,
                           const char *topic,
                           const uint8_t *payload,
                           uint16_t payload_len)
{
    uint8_t  *p = buf;
    uint16_t  tlen = (uint16_t)strlen(topic);
    uint32_t  remaining = 2 + tlen + payload_len;
    uint8_t   lebuf[4], lesz;

    lesz = mqtt_encode_len(remaining, lebuf);
    *p++ = MQTT_PUBLISH_Q0;
    memcpy(p, lebuf, lesz); p += lesz;

    put16(p, tlen); p += 2;
    memcpy(p, topic, tlen); p += tlen;

    if (payload && payload_len) {
        memcpy(p, payload, payload_len);
        p += payload_len;
    }

    return (uint16_t)(p - buf);
}

uint8_t MQTT_BuildPingReq(uint8_t *buf)
{
    buf[0] = MQTT_PINGREQ;
    buf[1] = 0x00;
    return 2;
}

uint8_t MQTT_BuildDisconnect(uint8_t *buf)
{
    buf[0] = MQTT_DISCONNECT;
    buf[1] = 0x00;
    return 2;
}

uint8_t MQTT_ParseConnack(uint8_t *data, uint16_t len)
{
    if (len < 4)             return 99;
    if (data[0] != 0x20)     return 1;
    if (data[3] != 0)        return data[3];
    return 0;
}

uint8_t MQTT_CheckType(uint8_t *data, uint8_t expected_type)
{
    if (!data) return 0;
    return ((data[0] & 0xF0) == (expected_type & 0xF0)) ? 1 : 0;
}
