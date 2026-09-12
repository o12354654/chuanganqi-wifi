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

/**
 * @brief  构造 SUBSCRIBE 报文（订阅，请求 QoS0）
 * @param  pkt_id 报文标识符，由调用方给，同一条连接内别重复
 * @note   固定头 0x82 已经含"必须置 1"的保留位和 QoS1 的 SUBSCRIBE 语义，
 *         这里请求的 QoS 填 0：云端下发的属性设置走 QoS0 就够了，
 *         省掉 SUBACK/PUBACK 之外的握手，逻辑更简单
 */
uint16_t MQTT_BuildSubscribe(uint8_t *buf, const char *topic, uint16_t pkt_id)
{
    uint8_t  *p = buf;
    uint16_t  tlen = (uint16_t)strlen(topic);
    uint32_t  remaining = 2 + 2 + tlen + 1;   /* 报文标识符 + topic 长度 + topic + 请求 QoS */
    uint8_t   lebuf[4], lesz;

    lesz = mqtt_encode_len(remaining, lebuf);
    *p++ = MQTT_SUBSCRIBE;                    /* 0x82 */
    memcpy(p, lebuf, lesz); p += lesz;

    put16(p, pkt_id); p += 2;
    put16(p, tlen);   p += 2;
    memcpy(p, topic, tlen); p += tlen;
    *p++ = 0x00;                              /* 请求 QoS0 */

    return (uint16_t)(p - buf);
}

/* PUBACK：QoS1 收到 PUBLISH 必须回，不然服务端会一直重发 */
uint16_t MQTT_BuildPubAck(uint8_t *buf, uint16_t pkt_id)
{
    buf[0] = MQTT_PUBACK;
    buf[1] = 0x02;
    put16(buf + 2, pkt_id);
    return 4;
}

/* 解 MQTT 变长剩余长度字段 */
static uint8_t mqtt_decode_len(const uint8_t *buf, uint16_t buf_len,
                               uint32_t *out, uint16_t *used)
{
    uint32_t mult = 1, val = 0;
    uint16_t i = 0;
    uint8_t  b;

    do {
        if (i >= buf_len || i >= 4) return 0;
        b = buf[i++];
        val += (uint32_t)(b & 0x7F) * mult;
        mult *= 128;
    } while (b & 0x80);

    *out  = val;
    *used = i;
    return 1;
}

uint8_t MQTT_ParsePublish(const uint8_t *data, uint16_t len,
                          const char **topic, uint16_t *topic_len,
                          const uint8_t **payload, uint16_t *payload_len,
                          uint16_t *pkt_id, uint8_t *qos)
{
    uint32_t rl, frame_end;
    uint16_t used, pos, tlen;
    uint8_t  q;

    if (!data || len < 4)                                    return 0;
    if ((data[0] & 0xF0) != MQTT_PUBLISH_Q0)                 return 0;

    q = (uint8_t)((data[0] >> 1) & 0x03);
    if (q > 1) return 0;          /* QoS2/保留值：不支持。宁可丢弃，也不要执行了却不确认 */

    if (!mqtt_decode_len(data + 1, (uint16_t)(len - 1), &rl, &used)) return 0;

    /* 整帧必须落在缓冲里（frame_end 用 uint32 算，避免 uint16 相加回绕绕过检查） */
    frame_end = (uint32_t)(1 + used) + rl;
    if (frame_end > (uint32_t)len) return 0;

    pos = (uint16_t)(1 + used);

    /* ===== MQTT 3.1.1：变量头顺序是 Topic Name → Packet Identifier → Payload =====
       （原先写成先读报文标识符再读 topic，于是任何 QoS1 帧都会解析失败、PUBACK 也发不出去） */
    if ((uint32_t)pos + 2 > frame_end) return 0;
    tlen = (uint16_t)((data[pos] << 8) | data[pos + 1]);
    pos  = (uint16_t)(pos + 2);

    if ((uint32_t)pos + tlen > frame_end) return 0;
    if (topic)     *topic     = (const char *)(data + pos);
    if (topic_len) *topic_len = tlen;
    pos = (uint16_t)(pos + tlen);

    if (q > 0) {                                   /* 报文标识符在 topic 之后 */
        if ((uint32_t)pos + 2 > frame_end) return 0;
        if (pkt_id) *pkt_id = (uint16_t)((data[pos] << 8) | data[pos + 1]);
        pos = (uint16_t)(pos + 2);
    } else if (pkt_id) {
        *pkt_id = 0;
    }

    /* 负载 = 帧尾之前剩下的全部（帧边界已封闭，不需要再和缓冲余额取小） */
    if (payload)     *payload     = data + pos;
    if (payload_len) *payload_len = (uint16_t)(frame_end - pos);

    if (qos) *qos = q;

    return 1;
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
