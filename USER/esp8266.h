#ifndef __ESP8266_H
#define __ESP8266_H

#include "sys.h"

#define WIFI_SSID       "MGZ"
#define WIFI_PASSWORD   "o12354654"
#define WIFI_BAUDRATE   115200

#define ONENET_IP       "mqtts.heclouds.com"
#define ONENET_PORT     1883
#define ONENET_PID      "OB4ClwDjY0"
#define ONENET_DEVICE   "wkhchuanganqi"
#define ONENET_AUTH     "version=2018-10-31&res=products%2FOB4ClwDjY0%2Fdevices%2Fwkhchuanganqi&et=1811808000&method=md5&sign=7Rhzdhp4e3v7HaNHdOIWOw%3D%3D"

#define MQTT_CLIENT_ID  ONENET_DEVICE

#define TOPIC_UPLOAD    "$sys/" ONENET_PID "/" ONENET_DEVICE "/thing/property/post"
#define MQTT_KEEPALIVE  60

#define TICK_INIT_WAIT         100
#define TICK_AT_TIMEOUT        150
#define TICK_CWJAP_TIMEOUT     750
#define TICK_RECONNECT_WIFI    250
#define TICK_CHECK_INTERVAL    500
#define TICK_TCP_TIMEOUT       500
#define TICK_CONNACK_TIMEOUT   300
#define TICK_MQTT_PUB          250
#define TICK_MQTT_PING         1500
#define TICK_CIPCLOSE_WAIT      25

/* ===== 网络授时（时间同步）===== */
#define TIME_NTP_SERVER    "ntp.aliyun.com"        /* 第 1 层：SNTP 授时 */
#define TIME_NTP_TZ        8                       /* 时区：东八区 */
#define TIME_HTTP_HOST     "acs.m.taobao.com"      /* 第 2 层：HTTP 响应头里的 Date */
#define TIME_HTTP_PORT     80
#define TIME_HTTP_PATH     "/gw/mtop.common.getTimestamp/"

#define TICK_SNTP_FIRST_WAIT   150   /* ~3s  等 SNTP 完成首次同步 */
#define TICK_SNTP_QUERY_GAP    100   /* ~2s  两次查询的间隔 */
#define SNTP_MAX_TRY           3     /* 最多问 3 次 */
#define TICK_TIME_HTTP_WAIT    300   /* ~6s  HTTP 授时总超时 */
#define TICK_SNTP_PROBE_WAIT   150   /* ~3s  MQTT 在线时周期性校时的等待 */

typedef enum {
    ESP_STATE_INIT,
    ESP_STATE_WAIT_AT,
    ESP_STATE_WAIT_CWMODE,
    ESP_STATE_WAIT_CWJAP,
    ESP_STATE_CONNECTED,
    ESP_STATE_CHECK,
    ESP_STATE_WAIT_RECONNECT,
    ESP_STATE_TIME_SNTP_CFG,
    ESP_STATE_TIME_SNTP_GET,
    ESP_STATE_TIME_HTTP_CONN,
    ESP_STATE_TIME_HTTP_GET,
    ESP_STATE_TIME_DONE,
    ESP_STATE_TCP_START,
    ESP_STATE_TCP_WAIT,
    ESP_STATE_TCP_SEND_CONNECT,
    ESP_STATE_TCP_WAIT_CONNACK,
    ESP_STATE_MQTT_ONLINE,
    ESP_STATE_TCP_DISCONNECT
} ESP8266_State;

void ESP8266_Init(void);
void ESP8266_Process(void);
uint8_t ESP8266_IsConnected(void);
uint8_t ESP8266_IsMQTTOnline(void);
ESP8266_State ESP8266_GetState(void);
uint8_t ESP8266_IsTransparent(void);

#endif
