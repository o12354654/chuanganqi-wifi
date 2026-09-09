/**
 * @file    esp8266.c
 * @brief   ESP8266 WiFi + OneNET MQTT 驱动
 * @note    PA2=USART2_TX  PA3=USART2_RX  波特率 115200
 *
 * 状态流转：
 *   INIT → AT → CWMODE → CWJAP → CONNECTED
 *     → TCP_START → TCP_WAIT → SEND_CONNECT → WAIT_CONNACK → MQTT_ONLINE
 *     ↓ 断连/失败
 *   TCP_DISCONNECT → TCP_START（重试） / WAIT_RECONNECT（WiFi断了）
 *
 * 【透传】USART1 输入 +++ 进入，--- 退出
 */

#include "esp8266.h"
#include "mqtt.h"
#include "delay.h"
#include "usart1.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>

/* ===================== 接收缓冲 ===================== */
#define RX_BUF_SIZE  512
static char     rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_len      = 0;
static uint16_t last_rx_len          = 0;

/* ===================== 发送缓冲 ===================== */
#define TX_BUF_SIZE  512
static uint8_t  tx_buf[TX_BUF_SIZE];

/* ===================== 状态机变量 ===================== */
static ESP8266_State  esp_state       = ESP_STATE_INIT;
static uint16_t       state_timer     = 0;
static uint32_t       abs_ticks       = 0;   // 绝对滴答（每 20ms 递增，防死锁）
static uint8_t        wifi_connected  = 0;
static uint8_t        mqtt_online     = 0;
static uint32_t       mqtt_timer      = 0;   // MQTT 在线计时

/* ===================== 透传变量 ===================== */
static uint8_t  transparent_mode      = 0;
static uint8_t  plus_count            = 0;
static uint16_t plus_timer            = 0;

/* ===================== DHT11 数据（外部引用） ===================== */
extern volatile u8 temp;
extern volatile u8 humi;

/* ===================== 内部函数声明 ===================== */
static void   ESP8266_USART2_Init(uint32_t baud);
static void   ESP8266_SendCmd(const char *cmd);
static void   ESP8266_ClearRx(void);
static uint8_t ESP8266_RxHas(const char *str);
static void   ESP8266_TransparentLoop(void);
static void   ESP8266_TransparentEnter(void);
static void   ESP8266_TransparentExit(void);
static uint8_t ESP8266_TCPSend(uint8_t *data, uint16_t len);
static uint8_t ESP8266_ParseIPD(char *buf, uint16_t *plen, uint8_t **pdata);

/* ===================== USART2 初始化 ===================== */
static void ESP8266_USART2_Init(uint32_t baud)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_2;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART2, ENABLE);
}

/* ===================== USART2 接收中断 ===================== */
void USART2_IRQHandler(void)
{
    /* 处理溢出错误（Overrun），清 ORE 才能继续收 */
    if (USART_GetITStatus(USART2, USART_IT_ORE) != RESET) {
        (void)USART_ReceiveData(USART2);
        USART_ClearITPendingBit(USART2, USART_IT_ORE);
    }
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint16_t ch = USART_ReceiveData(USART2);
        if (rx_len < RX_BUF_SIZE - 1) {
            rx_buf[rx_len++] = (char)ch;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/* ===================== 发送字符串 ===================== */
static void ESP8266_SendCmd(const char *cmd)
{
    while (*cmd) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, (uint16_t)*cmd++);
    }
}

/* ===================== 清空缓冲 ===================== */
static void ESP8266_ClearRx(void)
{
    rx_len      = 0;
    last_rx_len = 0;
    memset(rx_buf, 0, RX_BUF_SIZE);
}

/* ===================== 缓冲含字符串？ ===================== */
static uint8_t ESP8266_RxHas(const char *str)
{
    if (rx_len < RX_BUF_SIZE) {
        rx_buf[rx_len] = '\0';
    } else {
        rx_buf[RX_BUF_SIZE - 1] = '\0';
    }
    return (strstr(rx_buf, str) != NULL);
}

/* ===================== 解析 +IPD,<len>:<data> ===================== */
static uint8_t ESP8266_ParseIPD(char *buf, uint16_t *plen, uint8_t **pdata)
{
    char *p;
    int   n;

    p = strstr(buf, "+IPD,");
    if (!p) return 0;
    p += 5;

    n = 0;
    while (*p >= '0' && *p <= '9') {
        n = n * 10 + (*p - '0');
        p++;
    }
    *plen = (uint16_t)n;

    p = strchr(p, ':');
    if (!p) return 0;
    p++;

    *pdata = (uint8_t *)p;
    return 1;
}

/* ===================== TCP 发送（阻塞，带超时诊断） ===================== */
static uint8_t ESP8266_TCPSend(uint8_t *data, uint16_t len)
{
    char              cmd[32];
    volatile uint32_t timeout;
    uint16_t          i;

    ESP8266_ClearRx();
    Delay_ms(50);  /* 确保 ESP8266 消化完上一条命令 */

    sprintf(cmd, "AT+CIPSEND=%d\r\n", len);
    ESP8266_SendCmd(cmd);

    /* 等待 ">" 提示符 */
    timeout = 0;
    while (!ESP8266_RxHas(">") && timeout < 2000000) {
        if (ESP8266_RxHas("ERROR") || ESP8266_RxHas("busy")) {
            printf("ESP8266: TCPSend rejected (%d)\r\n", len);
            return 1;
        }
        timeout++;
    }
    if (timeout >= 2000000) {
        printf("ESP8266: TCPSend timeout, len=%d\r\n", len);
        /* 打印缓冲区内容帮助诊断 */
        rx_buf[rx_len] = '\0';
        printf("ESP8266: rx_buf[%d]=[%s]\r\n", rx_len, rx_buf);
        return 2;
    }

    Delay_ms(10);

    /* 发送数据 */
    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, (uint16_t)data[i]);
    }

    return 0;
}

/* ===================== 透传入口 ===================== */
static void ESP8266_TransparentEnter(void)
{
    transparent_mode = 1;
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    printf("\r\n========================================\r\n");
    printf("  透传模式 | 输入 --- 退出\r\n");
    printf("========================================\r\n");
}

static void ESP8266_TransparentExit(void)
{
    transparent_mode = 0;
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    plus_count  = 0;
    plus_timer  = 0;
    ESP8266_ClearRx();
    esp_state   = ESP_STATE_INIT;
    state_timer = 0;
    mqtt_online = 0;
    mqtt_timer  = 0;
    printf("\r\n=== 透传已退出 ===\r\n");
}

/* ===================== 透传阻塞循环 ===================== */
static void ESP8266_TransparentLoop(void)
{
    uint8_t minus_count = 0;
    while (transparent_mode) {
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
            uint8_t ch = (uint8_t)USART_ReceiveData(USART1);
            if (ch == '-') {
                minus_count++;
                if (minus_count >= 3) { ESP8266_TransparentExit(); return; }
            } else {
                minus_count = 0;
            }
            while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
            USART_SendData(USART2, (uint16_t)ch);
        }
        if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
            uint8_t ch = (uint8_t)USART_ReceiveData(USART2);
            while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, (uint16_t)ch);
        }
    }
}

/* ===================== 初始化 ===================== */
void ESP8266_Init(void)
{
    ESP8266_USART2_Init(WIFI_BAUDRATE);
    ESP8266_ClearRx();
    esp_state       = ESP_STATE_INIT;
    state_timer     = 0;
    abs_ticks       = 0;
    wifi_connected  = 0;
    mqtt_online     = 0;
    mqtt_timer      = 0;
    transparent_mode = 0;
    plus_count      = 0;
    plus_timer      = 0;

    printf("ESP8266: USART2 init OK (%d baud)\r\n", WIFI_BAUDRATE);
    printf("ESP8266: +++ 透传 | WiFi=%s\r\n", WIFI_SSID);
}

/* ===================== 查询接口 ===================== */
uint8_t ESP8266_IsConnected(void)  { return wifi_connected; }
uint8_t ESP8266_IsMQTTOnline(void) { return mqtt_online; }
uint8_t ESP8266_IsTransparent(void){ return transparent_mode; }
ESP8266_State ESP8266_GetState(void){ return esp_state; }

/* ===================== 主状态机 ===================== */
void ESP8266_Process(void)
{
    uint8_t  new_data;
    uint16_t pkt_len;
    uint8_t  onenet_data[128];  /* JSON 最长 ~88 字节 */
    int      slen;
    char     tmp[96];

    /* ===== 透传入口检测 ===== */
    if (!transparent_mode) {
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
            uint8_t ch = (uint8_t)USART_ReceiveData(USART1);
            if (ch == '+') {
                plus_count++;
                plus_timer = 50;
                if (plus_count >= 3) {
                    ESP8266_TransparentEnter();
                    ESP8266_TransparentLoop();
                    return;
                }
            } else {
                plus_count = 0;
                plus_timer = 0;
            }
        }
        if (plus_timer > 0) { plus_timer--; if (!plus_timer) plus_count = 0; }
    }

    /* ===== 状态机 ===== */
    new_data = (rx_len != last_rx_len);
    last_rx_len = rx_len;

    switch (esp_state) {

    /* ---- INIT ---- */
    case ESP_STATE_INIT:
        if (state_timer >= TICK_INIT_WAIT) {
            state_timer = 0;
            ESP8266_ClearRx();
            ESP8266_SendCmd("ATE0\r\n");   /* 关回显，防 buffer 污染导致超时失效 */
            esp_state = ESP_STATE_WAIT_AT;
        }
        break;

    /* ---- WAIT AT ---- */
    case ESP_STATE_WAIT_AT:
        if (ESP8266_RxHas("OK")) {
            state_timer = 0;
            ESP8266_ClearRx();
            ESP8266_SendCmd("AT+CWMODE=1\r\n");
            esp_state = ESP_STATE_WAIT_CWMODE;
        } else if (ESP8266_RxHas("ERROR") || state_timer >= TICK_AT_TIMEOUT) {
            state_timer = 0;
            ESP8266_ClearRx();
            ESP8266_SendCmd("AT\r\n");
        }
        break;

    /* ---- WAIT CWMODE ---- */
    case ESP_STATE_WAIT_CWMODE:
        if (ESP8266_RxHas("OK")) {
            state_timer = 0;
            ESP8266_ClearRx();
            ESP8266_SendCmd("AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASSWORD "\"\r\n");
            esp_state = ESP_STATE_WAIT_CWJAP;
        } else if (ESP8266_RxHas("ERROR") || state_timer >= TICK_AT_TIMEOUT) {
            state_timer = 0;
            ESP8266_ClearRx();
            ESP8266_SendCmd("AT\r\n");
            esp_state = ESP_STATE_WAIT_AT;
        }
        break;

    /* ---- WAIT CWJAP ---- */
    case ESP_STATE_WAIT_CWJAP:
        if (ESP8266_RxHas("WIFI GOT IP") || ESP8266_RxHas("WIFI CONNECTED")) {
            printf("ESP8266: WiFi Connected!\r\n");
            wifi_connected = 1;
            state_timer = 0;
            ESP8266_ClearRx();
            /* 先设置单连接模式，再发起 TCP */
            ESP8266_SendCmd("AT+CIPMUX=0\r\n");
            esp_state = ESP_STATE_TCP_START;
        } else if (ESP8266_RxHas("FAIL") || ESP8266_RxHas("ERROR")) {
            printf("ESP8266: WiFi FAIL\r\n");
            wifi_connected = 0;
            state_timer = 0;
            ESP8266_ClearRx();
            esp_state = ESP_STATE_WAIT_RECONNECT;
        } else if (state_timer >= TICK_CWJAP_TIMEOUT) {
            printf("ESP8266: WiFi timeout\r\n");
            wifi_connected = 0;
            state_timer = 0;
            ESP8266_ClearRx();
            esp_state = ESP_STATE_WAIT_RECONNECT;
        }
        break;

    /* ---- WiFi 已连接：巡检 ---- */
    case ESP_STATE_CONNECTED:
        if (state_timer >= TICK_CHECK_INTERVAL) {
            state_timer = 0;
            ESP8266_ClearRx();
            ESP8266_SendCmd("AT+CWJAP?\r\n");
            esp_state = ESP_STATE_CHECK;
        }
        break;

    case ESP_STATE_CHECK:
        if (ESP8266_RxHas("+CWJAP:") || ESP8266_RxHas("OK")) {
            state_timer = 0;
            ESP8266_ClearRx();
            esp_state = ESP_STATE_CONNECTED;
        } else {
            printf("ESP8266: WiFi lost\r\n");
            wifi_connected = 0;
            state_timer = 0;
            ESP8266_ClearRx();
            esp_state = ESP_STATE_WAIT_RECONNECT;
        }
        break;

    /* ---- WiFi 重连 ---- */
    case ESP_STATE_WAIT_RECONNECT:
        if (state_timer >= TICK_RECONNECT_WIFI) {
            state_timer = 0;
            ESP8266_ClearRx();
            ESP8266_SendCmd("AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASSWORD "\"\r\n");
            esp_state = ESP_STATE_WAIT_CWJAP;
        }
        break;

    /* ============================================= */
    /*              MQTT / OneNET 状态                */
    /* ============================================= */

    /* ---- 发起 TCP 连接（先等 CIPMUX OK） ---- */
    case ESP_STATE_TCP_START:
        if (ESP8266_RxHas("OK") || ESP8266_RxHas("no change")) {
            /* CIPMUX=0 就绪，发起 TCP */
            ESP8266_ClearRx();
            sprintf(tmp, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ONENET_IP, ONENET_PORT);
            printf("ESP8266: %s", tmp);
            ESP8266_SendCmd(tmp);
            state_timer = 0;
            esp_state = ESP_STATE_TCP_WAIT;
        } else if (state_timer >= 50) {
            /* 超时，重试 */
            state_timer = 0;
            ESP8266_ClearRx();
            ESP8266_SendCmd("AT+CIPMUX=0\r\n");
        }
        break;

    /* ---- 等待 TCP CONNECT ---- */
    case ESP_STATE_TCP_WAIT:
        if (ESP8266_RxHas("CONNECT") || ESP8266_RxHas("ALREADY CONNECTED")) {
            /* 必须同时收到 OK 才说明 TCP 真正就绪 */
            if (ESP8266_RxHas("OK")) {
                printf("ESP8266: TCP connected to OneNET\r\n");
                state_timer = 0;
                ESP8266_ClearRx();

                /* 等 ESP8266 内部稳定 */
                Delay_ms(300);
                ESP8266_ClearRx();

                /* 构造 MQTT CONNECT 报文（Password=Token） */
                /* 构造 MQTT CONNECT 报文（密码直接用 access-key） */
                pkt_len = MQTT_BuildConnect(tx_buf,
                             MQTT_CLIENT_ID, ONENET_PID, ONENET_AUTH,
                             MQTT_KEEPALIVE);
                printf("ESP8266: MQTT CONNECT %d bytes, CID=%s\r\n",
                       pkt_len, MQTT_CLIENT_ID);
                /* 诊断：打印 CONNECT 报文前 16 字节 */
                {
                    uint8_t di;
                    printf("ESP8266: CONNECT hex: ");
                    for (di = 0; di < 16 && di < pkt_len; di++) {
                        printf("%02X ", tx_buf[di]);
                    }
                    printf("\r\n");
                }

                if (ESP8266_TCPSend(tx_buf, pkt_len) == 0) {
                    printf("ESP8266: MQTT CONNECT sent\r\n");
                    state_timer = 0;
                    esp_state = ESP_STATE_TCP_WAIT_CONNACK;
                } else {
                    printf("ESP8266: CIPSEND fail\r\n");
                    state_timer = 0;
                    esp_state = ESP_STATE_TCP_DISCONNECT;
                }
            }
            /* 有 CONNECT 但无 OK → 继续等 */
        } else if (ESP8266_RxHas("ERROR") || ESP8266_RxHas("CLOSED") ||
                   ESP8266_RxHas("DNS Fail") ||
                   state_timer >= TICK_TCP_TIMEOUT) {
            printf("ESP8266: TCP connect fail, retry...\r\n");
            state_timer = 0;
            ESP8266_ClearRx();
            esp_state = ESP_STATE_TCP_DISCONNECT;
        }
        break;

    /* ---- 等待 MQTT CONNACK ---- */
    case ESP_STATE_TCP_WAIT_CONNACK:
        if (ESP8266_RxHas("+IPD,")) {
            uint16_t ipd_len;
            uint8_t *ipd_data;
            if (ESP8266_ParseIPD(rx_buf, &ipd_len, &ipd_data)) {
                uint8_t result = MQTT_ParseConnack(ipd_data, ipd_len);
                if (result == 0) {
                    printf("ESP8266: MQTT Online! (OneNET)\r\n");
                    mqtt_online = 1;
                    mqtt_timer  = 0;
                    ESP8266_ClearRx();
                    esp_state   = ESP_STATE_MQTT_ONLINE;
                } else {
                    printf("ESP8266: CONNACK err=%d\r\n", result);
                    esp_state = ESP_STATE_TCP_DISCONNECT;
                }
            }
        } else if (ESP8266_RxHas("CLOSED") || ESP8266_RxHas("ERROR") ||
                   state_timer >= TICK_CONNACK_TIMEOUT) {
            printf("ESP8266: CONNACK timeout\r\n");
            esp_state = ESP_STATE_TCP_DISCONNECT;
        }
        break;

    /* ---- MQTT 在线 ---- */
    case ESP_STATE_MQTT_ONLINE:
        mqtt_timer++;

        /* 检测 TCP 断开 */
        if (ESP8266_RxHas("CLOSED") || ESP8266_RxHas("ERROR")) {
            printf("ESP8266: TCP lost (CLOSED/ERROR)\r\n");
            mqtt_online = 0;
            mqtt_timer  = 0;
            ESP8266_ClearRx();
            esp_state = ESP_STATE_TCP_DISCONNECT;
            break;
        }

        /* ==== 上报温湿度（每 5s） ==== */
        if ((mqtt_timer % TICK_MQTT_PUB) == 0) {
            uint8_t pub_ok;
            /* OneNET Studio OneJSON 格式：属性值嵌套 value，time 可选(省略) */
            slen = sprintf((char *)onenet_data,
                   "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{\"temp\":{\"value\":%d},\"humi\":{\"value\":%d}}}",
                   (unsigned long)mqtt_timer, temp, humi);
            pkt_len = MQTT_BuildPublish(tx_buf, TOPIC_UPLOAD,
                                        onenet_data, (uint16_t)slen);

            pub_ok = ESP8266_TCPSend(tx_buf, pkt_len);
            if (pub_ok == 0) {
                printf("MQTT PUB: temp=%d humi=%d\r\n", temp, humi);
            } else {
                printf("MQTT PUB fail (%d), reconnecting\r\n", pub_ok);
                mqtt_online = 0;
                mqtt_timer  = 0;
                esp_state = ESP_STATE_TCP_DISCONNECT;
                break;
            }
        }

        /* ==== 心跳 PING（每 30s） ==== */
        if ((mqtt_timer % TICK_MQTT_PING) == 0) {
            uint8_t ping_ok;
            pkt_len = MQTT_BuildPingReq(tx_buf);
            ping_ok = ESP8266_TCPSend(tx_buf, pkt_len);
            if (ping_ok != 0) {
                printf("MQTT PING fail, reconnecting\r\n");
                mqtt_online = 0;
                mqtt_timer  = 0;
                esp_state = ESP_STATE_TCP_DISCONNECT;
                break;
            }
        }

        /* 清理旧的 +IPD 数据，避免重复检测 */
        if (ESP8266_RxHas("+IPD,")) {
            ESP8266_ClearRx();
        }

        /* 防止 mqtt_timer 溢出 */
        if (mqtt_timer >= 60000) mqtt_timer = 0;
        break;

    /* ---- 断开 TCP（等 CIPCLOSE 完成再重试） ---- */
    case ESP_STATE_TCP_DISCONNECT:
        mqtt_online = 0;
        mqtt_timer  = 0;
        if (state_timer == 0) {
            ESP8266_SendCmd("AT+CIPCLOSE\r\n");
            printf("ESP8266: CIPCLOSE sent\r\n");
        }
        /* 等待 CIPCLOSE 完成，避免 TCP 状态残留 */
        if (ESP8266_RxHas("CLOSED") || ESP8266_RxHas("OK") ||
            ESP8266_RxHas("ERROR") || state_timer >= TICK_CIPCLOSE_WAIT) {
            ESP8266_ClearRx();
            Delay_ms(100);
            if (wifi_connected) {
                state_timer = 0;
                esp_state = ESP_STATE_TCP_START;
            } else {
                state_timer = 0;
                esp_state = ESP_STATE_WAIT_RECONNECT;
            }
        }
        break;

    default:
        esp_state = ESP_STATE_INIT;
        break;
    }

    /* 计时器：仅在无新数据时推进 */
    if (!new_data) {
        state_timer++;
    }
    abs_ticks++;
    /* 绝对超时保底：15s 无进展则强制复位 */
    if (abs_ticks >= 750) {
        uint16_t i;
        abs_ticks = 0;
        printf("ESP8266: ABS timeout, state=%d, reset\r\n", esp_state);
        ESP8266_ClearRx();
        /* 尽力断开TCP */
        ESP8266_SendCmd("AT+CIPCLOSE\r\n");
        for (i = 0; i < 10000; i++);  /* ~10ms */
        ESP8266_ClearRx();
        esp_state   = ESP_STATE_INIT;
        state_timer = 0;
        mqtt_online = 0;
        mqtt_timer  = 0;
    }
}
