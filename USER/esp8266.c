/**
 * @file    esp8266.c
 * @brief   ESP8266 WiFi + OneNET MQTT 驱动
 * @note    PA2=USART2_TX  PA3=USART2_RX  波特率 115200
 *
 * 状态流转：
 *   INIT → AT → CWMODE → CWJAP → CONNECTED
 *     → TIME_SNTP_CFG → TIME_SNTP_GET      （网络授时第 1 层：SNTP）
 *     → TIME_HTTP_CONN → TIME_HTTP_GET     （第 1 层不行才走，第 2 层：HTTP 的 Date 头）
 *     → TIME_DONE
 *     → TCP_START → TCP_WAIT → SEND_CONNECT → WAIT_CONNACK → MQTT_ONLINE
 *     ↓ 断连/失败
 *   TCP_DISCONNECT → TCP_START（重试） / WAIT_RECONNECT（WiFi断了）
 *
 * 【授时策略】联网优先用网络时间，拿到就写回 DS1302；
 *   两层都拿不到 → 继续用 DS1302 走时，60s 后再试，全程不阻塞主循环。
 *
 * 【透传】USART1 输入 +++ 进入，--- 退出
 */

#include "esp8266.h"
#include "mqtt.h"
#include "delay.h"
#include "usart1.h"
#include "misc.h"
#include "clock.h"
#include "timeparse.h"
#include "relay_cloud.h"
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
static uint8_t        sub_ok          = 0;   // 订阅是否已被平台确认（收到 SUBACK）

/* ===================== 授时变量 ===================== */
static uint8_t  sntp_try              = 0;   /* SNTP 已问过几次 */
static uint8_t  sntp_sent             = 0;   /* 本次查询已发出 */
static uint8_t  sntp_supported        = 1;   /* 这版 AT 固件是否支持 SNTP */
static uint8_t  probe_on              = 0;   /* MQTT 在线时的周期性校时 */
static uint16_t probe_timer           = 0;
static uint8_t  close_sent            = 0;   /* CIPCLOSE 只发一次 */

/* ===================== 下行回执记账 =====================
 * 收到属性设置后不能当场发回执：发回执要过 TCPSend，它会清接收缓冲，
 * 而同一个缓冲里可能还躺着别的帧。所以先记账，等所有帧处理完再统一发。
 */
static char     dl_reply[160];        /* 待发 set_reply 的 JSON */
static uint16_t dl_rlen          = 0;
static uint8_t  dl_reply_ready   = 0;
static uint16_t dl_puback_id     = 0;
static uint8_t  dl_puback_ready  = 0;
static ESP8266_State last_state       = ESP_STATE_INIT;  /* 防卡死保底用 */

/* ===================== 透传变量 ===================== */
static uint8_t  transparent_mode      = 0;
static uint8_t  plus_count            = 0;
static uint16_t plus_timer            = 0;

/* ===================== DHT11 数据（外部引用） ===================== */
extern u8 temp;
extern u8 humi;

/* ===================== 内部函数声明 ===================== */
static void   ESP8266_USART2_Init(uint32_t baud);
static void   ESP8266_SendCmd(const char *cmd);
static void   ESP8266_ClearRx(void);
static uint8_t ESP8266_RxHas(const char *str);
static char  *ESP8266_RxStr(void);
static void   ESP8266_RxSlide(void);
static void   ESP8266_TimeSyncStart(void);
static void   ESP8266_TimeSyncHttp(void);
static void   ESP8266_TransparentLoop(void);
static void   ESP8266_TransparentEnter(void);
static void   ESP8266_TransparentExit(void);
static uint8_t ESP8266_TCPSend(uint8_t *data, uint16_t len);
static uint8_t ESP8266_ParseIPD(char *buf, uint16_t *plen, uint8_t **pdata);
static void    ESP8266_SendSubscribe(void);
static void    ESP8266_ProcessDownlink(void);
static void    ESP8266_FlushDownlink(void);
static void    ESP8266_HandleDownlink(uint8_t *data, uint16_t len);
static void    ESP8266_JsonId(const char *json, char *out, uint16_t max);

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

/* ===================== 缓冲加字符串尾巴 ===================== */
static uint8_t ESP8266_RxHas(const char *str)
{
    if (rx_len < RX_BUF_SIZE) {
        rx_buf[rx_len] = '\0';
    } else {
        rx_buf[RX_BUF_SIZE - 1] = '\0';
    }
    return (strstr(rx_buf, str) != NULL);
}

/* 缓冲补 '\0' 后返回，供解析函数使用 */
static char *ESP8266_RxStr(void)
{
    if (rx_len < RX_BUF_SIZE) {
        rx_buf[rx_len] = '\0';
    } else {
        rx_buf[RX_BUF_SIZE - 1] = '\0';
    }
    return rx_buf;
}

/**
 * @brief  缓冲快满时把尾部挪到前面
 * @note   HTTP 响应头动辄几百字节，512 的缓冲一满后面就再也收不进来，
 *         解析 Date 之前先滑一次窗口。挪动期间关 RXNE 中断防止踩踏。
 */
static void ESP8266_RxSlide(void)
{
    uint16_t keep = 160;
    uint16_t i;

    if (rx_len < RX_BUF_SIZE - 32) return;   /* 还没满，不用滑 */

    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    for (i = 0; i < keep; i++) {
        rx_buf[i] = rx_buf[rx_len - keep + i];
    }
    rx_len      = keep;
    last_rx_len = keep;
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
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

/* ===================== 订阅云端属性设置 ===================== */
static void ESP8266_SendSubscribe(void)
{
    uint16_t n = MQTT_BuildSubscribe(tx_buf, TOPIC_PROP_SET, MQTT_SUB_PKT_ID);

    if (ESP8266_TCPSend(tx_buf, n) == 0) {
        printf("ESP8266: SUB %s\r\n", TOPIC_PROP_SET);
    } else {
        printf("ESP8266: SUB send fail\r\n");
    }
}

/* 从下发 JSON 里抠出 "id" 的值（set_reply 要原样带回） */
static void ESP8266_JsonId(const char *json, char *out, uint16_t max)
{
    const char *p;
    uint16_t    n = 0;

    if (!out || max == 0) return;
    out[0] = '\0';

    p = strstr(json, "\"id\"");
    if (!p) return;

    p = strchr(p + 4, ':');
    if (!p) return;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\"') p++;
    /* 结束条件要带上 , } 和空白：数字形式的 id（"id":1,）原先只在遇到引号时才停，
       会把后面的逗号一起抠出来，回执的 id 跟请求对不上（平台显示"未正确应答"） */
    while (*p && *p != '"' && *p != ',' && *p != '}' &&
           *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n' &&
           n < (uint16_t)(max - 1)) {
        out[n++] = *p++;
    }
    out[n] = '\0';
}

/**
 * @brief  处理一帧下行 MQTT 数据（+IPD 里的载荷）
 * @note   topic / payload 都直接指进接收缓冲，所以先把要用的东西拷到栈上再发报文——
 *         TCPSend 内部会清接收缓冲，指针一发就废
 *         QoS1 的下发必须先回 PUBACK；命中的属性再按 OneNET 物模型规范回 set_reply
 */
static void ESP8266_HandleDownlink(uint8_t *data, uint16_t len)
{
    const char    *topic;
    const uint8_t *payload;
    uint16_t       topic_len   = 0;
    uint16_t       payload_len = 0;
    uint16_t       pkt_id      = 0;
    uint8_t        qos         = 0;
    uint8_t        hit;
    static char    json[384];   /* 栈紧张，一律 static；放大到 384：多属性 OneJSON 可能很长，
                                   截断会让靠后的属性静默失效（回给云端的是"未知属性"，方向全被带偏） */
    static char    id[24];
    uint16_t       n;

    if (len >= 4 && (data[0] & 0xF0) == MQTT_SUBACK) {
        /* SUBACK = 固定头(2) + 报文标识符(2) + 每个订阅项一个返回码，
           返回码是**最后一个字节**：0x00/0x01/0x02 = 授予的 QoS，0x80 = 失败/未授权。
           返修记录：这里原先判 data[3]，那是报文标识符的低字节 —— 平台正常回
           90 03 00 01 00 时它等于 0x01，成功的订阅被当成失败，于是每 10s 无脑重订 */
        sub_ok = (uint8_t)((len >= 5 && data[len - 1] != 0x80) ? 1 : 0);
        return;
    }

    if (!MQTT_ParsePublish(data, len, &topic, &topic_len,
                           &payload, &payload_len, &pkt_id, &qos)) {
        return;                        /* PINGRESP 之类：不是下发，不理 */
    }

    if (topic_len != (uint16_t)strlen(TOPIC_PROP_SET) ||
        strncmp(topic, TOPIC_PROP_SET, topic_len) != 0) {
        /* 临时诊断（定位完了删）：帧收到了但 topic 不是我们的 */
        printf("MQTT SET: topic mismatch [%.*s]\r\n", (int)topic_len, topic);
        return;                        /* 不是本设备的下发 topic */
    }

    n = (payload_len < (uint16_t)(sizeof(json) - 1)) ? payload_len
                                                     : (uint16_t)(sizeof(json) - 1);
    memcpy(json, payload, n);
    json[n] = '\0';

    ESP8266_JsonId(json, id, (uint16_t)sizeof(id));

    printf("MQTT SET: %s\r\n", json);   /* 临时诊断（定位完了删）：云端下发的原文 */

    hit = Relay_Cloud_HandleSet(json, n);   /* 让位给云端 + 开关继电器 */

    /* 回执只记账，不在这里发（发报文的 TCPSend 会清接收缓冲，
       会把同一缓冲里还没处理的帧一起清掉） */
    if (qos == 1) {                         /* QoS1：不回 PUBACK 服务端会一直重发 */
        dl_puback_id    = pkt_id;
        dl_puback_ready = 1;
    }

    /* 按 OneNET 物模型规范回 set_reply：命中的回 200，没认出来的回 404，
       这样云端能看到是"执行了"还是"属性名没对上"，不用盯串口 */
    dl_rlen = (uint16_t)sprintf(dl_reply, "{\"id\":\"%s\",\"code\":%d,\"msg\":\"%s\"}",
                                (id[0] ? id : "0"), (hit ? 200 : 404),
                                (hit ? "success" : "unknown property"));
    dl_reply_ready = 1;
}

/* ===================== 发下行回执（所有帧处理完之后才调用） ===================== */
static void ESP8266_FlushDownlink(void)
{
    uint16_t n;

    if (dl_puback_ready) {                 /* QoS1 必须先确认，否则服务端会一直重发 */
        dl_puback_ready = 0;
        n = MQTT_BuildPubAck(tx_buf, dl_puback_id);
        ESP8266_TCPSend(tx_buf, n);
    }
    if (dl_reply_ready) {
        uint16_t r;

        dl_reply_ready = 0;
        n = MQTT_BuildPublish(tx_buf, TOPIC_SET_REPLY,
                              (const uint8_t *)dl_reply, dl_rlen);
        r = ESP8266_TCPSend(tx_buf, n);
        /* 临时诊断（定位完了删）：回执发不出去原来是静默的 */
        printf("MQTT SET reply: %s\r\n", (r == 0) ? "sent" : "FAIL");
    }
}

/**
 * @brief  处理接收缓冲里所有"已经收全"的下行 +IPD 帧
 * @note   三个要点：
 *         ① 帧没到齐就原地等下一轮 —— 不半帧处理、也不清缓冲。
 *            115200 下一帧 112 字节要占约 9.7ms 线时，而主循环一轮约 20ms，
 *            帧跨轮到达是常态；原先"一看到 +IPD 就按截断长度处理并无条件清缓冲"，
 *            结果是命令时灵时不灵、还没法从面板上看出来
 *         ② 一个缓冲里可能挤着好几帧（PINGRESP/SUBACK 和 set 报文落到同一个 20ms 窗口），
 *            循环把收全的都处理掉
 *         ③ 只搬走已消费的字节（memmove 剩下的到缓冲头），不再整块清掉
 */
static void ESP8266_ProcessDownlink(void)
{
    uint16_t consumed = 0;

    for (;;) {
        char     *buf = ESP8266_RxStr();                      /* 补 '\0' 并返回缓冲 */
        char     *ipd = strstr(buf + consumed, "+IPD,");
        uint16_t  dstart, ipd_len;
        uint8_t  *ipd_data;

        if (!ipd) break;                                       /* 没有更多 +IPD 了 */

        if (!ESP8266_ParseIPD(ipd, &ipd_len, &ipd_data)) break; /* 头还没收全 */

        dstart = (uint16_t)(ipd_data - (uint8_t *)buf);        /* 统一指针类型，别让编译器抱怨 */
        if ((uint32_t)dstart > rx_len) break;
        if ((uint32_t)dstart + ipd_len > rx_len) break;        /* 数据没到齐：等下一轮 */

        ESP8266_HandleDownlink(ipd_data, ipd_len);
        consumed = (uint16_t)(dstart + ipd_len);
    }

    if (consumed) {
        /* 只搬走已消费的部分（关中断，别和接收中断抢缓冲） */
        USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
        rx_len = (uint16_t)(rx_len - consumed);
        if (rx_len) {
            memmove(rx_buf, rx_buf + consumed, rx_len);
        }
        rx_buf[rx_len] = '\0';
        last_rx_len = rx_len;
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    }

    ESP8266_FlushDownlink();       /* 回执最后发：它内部的 TCPSend 会清缓冲 */
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

    /* 等 ">" 提示符：这个循环是阻塞的，上限调小一点，
       ESP 正常几毫秒就回，2e6 次 @72MHz 万一不应答会拖住主循环好几秒 */
    timeout = 0;
    while (!ESP8266_RxHas(">") && timeout < 20000) {
        if (ESP8266_RxHas("ERROR") || ESP8266_RxHas("busy")) {
            printf("ESP8266: TCPSend rejected (%d)\r\n", len);
            return 1;
        }
        timeout++;
    }
    if (timeout >= 20000) {
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
    printf("  transparent mode | type --- to exit\r\n");
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
    last_state  = ESP_STATE_INIT;
    close_sent  = 0;
    probe_on    = 0;
    mqtt_online = 0;
    mqtt_timer  = 0;
    sub_ok      = 0;
    Relay_Cloud_OnMqttLost();
    printf("\r\n=== transparent mode exited ===\r\n");
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

/* ===================== 授时：第 1 层 SNTP ===================== */
static void ESP8266_TimeSyncStart(void)
{
    sntp_try   = 0;
    sntp_sent  = 0;
    close_sent = 0;
    ESP8266_ClearRx();
    printf("ESP8266: net time sync (1/2 SNTP)\r\n");
    ESP8266_SendCmd("AT+CIPSNTPCFG=1," "8" ",\"" TIME_NTP_SERVER "\"\r\n");
    esp_state   = ESP_STATE_TIME_SNTP_CFG;
    state_timer = 0;
}

/* ===================== 授时：第 2 层 HTTP Date ===================== */
static void ESP8266_TimeSyncHttp(void)
{
    char cmd[64];

    sntp_supported = 0;      /* 这版固件没有 SNTP，本次连接内不再试 */
    close_sent     = 0;
    ESP8266_ClearRx();
    printf("ESP8266: net time sync (2/2 HTTP Date)\r\n");
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", TIME_HTTP_HOST, TIME_HTTP_PORT);
    ESP8266_SendCmd(cmd);
    esp_state   = ESP_STATE_TIME_HTTP_CONN;
    state_timer = 0;
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
    sntp_try        = 0;
    sntp_sent       = 0;
    sntp_supported  = 1;
    probe_on        = 0;
    probe_timer     = 0;
    close_sent      = 0;
    last_state      = ESP_STATE_INIT;

    printf("ESP8266: USART2 init OK (%d baud)\r\n", WIFI_BAUDRATE);
    printf("ESP8266: transparent mode=+++ / --- , WiFi=%s\r\n", WIFI_SSID);
}

/* ===================== 查询接口 ===================== */
uint8_t ESP8266_IsConnected(void)  { return wifi_connected; }
uint8_t ESP8266_IsMQTTOnline(void) { return mqtt_online; }
uint8_t ESP8266_IsTransparent(void){ return transparent_mode; }
ESP8266_State ESP8266_GetState(void){ return esp_state; }

/* ===================== 主状态机 ===================== */
void ESP8266_Process(void)
{
    /* 大数组一律 static：本工程栈只有 512 字节(Stack_Size=0x200)，
       局部数组会把栈顶穿，表现是随机跑飞而不是编译报错 */
    static uint8_t onenet_data[128];  /* JSON 最长 ~88 字节 */
    static char    tmp[96];
    uint8_t  new_data;
    uint16_t pkt_len;
    int      slen;

    /* ===== WiFi 掉线监听（模块会主动推 WIFI DISCONNECT，不必等 TCP 出错）===== */
    if (ESP8266_RxHas("WIFI DISCONNECT")) {
        if (wifi_connected) {
            printf("ESP8266: WiFi lost (WIFI DISCONNECT)\r\n");
        }
        wifi_connected = 0;      /* 清标志：后面 TCP 连不上会自动退回 WAIT_RECONNECT 重连 AP */
        ESP8266_ClearRx();
    }

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
            sntp_supported = 1;
            ESP8266_ClearRx();
            /* 刚联网：先抢网络时间（拿不到会退回 DS1302），再进 MQTT */
            if (Clock_NetSyncDue()) {
                Clock_NetSyncTry();
                ESP8266_TimeSyncStart();
            } else {
                /* 先设置单连接模式，再发起 TCP */
                ESP8266_SendCmd("AT+CIPMUX=0\r\n");
                esp_state = ESP_STATE_TCP_START;
            }
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
    /*                 网络授时状态                   */
    /* ============================================= */

    /* ---- SNTP 配置 ---- */
    case ESP_STATE_TIME_SNTP_CFG:
        if (ESP8266_RxHas("OK")) {
            ESP8266_ClearRx();
            state_timer = 0;
            sntp_sent   = 0;
            printf("ESP8266: SNTP cfg OK, wait first sync\r\n");
            esp_state   = ESP_STATE_TIME_SNTP_GET;
        } else if (ESP8266_RxHas("ERROR") || state_timer >= TICK_SNTP_FIRST_WAIT) {
            printf("ESP8266: SNTP not supported\r\n");
            ESP8266_TimeSyncHttp();
        }
        break;

    /* ---- 查询 SNTP 时间 ---- */
    case ESP_STATE_TIME_SNTP_GET:
        if (!sntp_sent) {
            /* 刚配好 SNTP，给它 TICK_SNTP_FIRST_WAIT 时间做首次同步再问 */
            if (state_timer >= TICK_SNTP_FIRST_WAIT) {
                ESP8266_ClearRx();
                ESP8266_SendCmd("AT+CIPSNTPTIME?\r\n");
                sntp_sent   = 1;
                sntp_try++;
                state_timer = 0;
            }
            break;
        }

        if (ESP8266_RxHas("+CIPSNTPTIME:")) {
            DS1302_TIME nt;
            if (TimeParse_SntpTime(ESP8266_RxStr(), &nt)) {
                ESP8266_ClearRx();
                Clock_NetSyncOk(&nt);
                probe_on    = 0;
                state_timer = 0;
                esp_state   = ESP_STATE_TIME_DONE;
                break;
            }
            printf("ESP8266: SNTP time not ready\r\n");
            ESP8266_ClearRx();
        }

        if (state_timer >= TICK_SNTP_QUERY_GAP) {
            if (sntp_try >= SNTP_MAX_TRY) {
                printf("ESP8266: SNTP timeout\r\n");
                ESP8266_TimeSyncHttp();
            } else {
                ESP8266_ClearRx();
                ESP8266_SendCmd("AT+CIPSNTPTIME?\r\n");
                sntp_try++;
                state_timer = 0;
            }
        }
        break;

    /* ---- HTTP 授时：建立临时 TCP ---- */
    case ESP_STATE_TIME_HTTP_CONN:
        if (ESP8266_RxHas("CONNECT") && ESP8266_RxHas("OK")) {
            char req[192];
            int  n;

            ESP8266_ClearRx();
            n = sprintf(req,
                        "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: ESP8266\r\nConnection: close\r\n\r\n",
                        TIME_HTTP_PATH, TIME_HTTP_HOST);
            printf("ESP8266: time GET %d bytes\r\n", n);
            if (ESP8266_TCPSend((uint8_t *)req, (uint16_t)n) == 0) {
                state_timer = 0;
                esp_state   = ESP_STATE_TIME_HTTP_GET;
            } else {
                printf("ESP8266: time GET send fail\r\n");
                Clock_NetSyncFail();
                state_timer = 0;
                esp_state   = ESP_STATE_TIME_DONE;
            }
        } else if (ESP8266_RxHas("ERROR") || ESP8266_RxHas("CLOSED") ||
                   ESP8266_RxHas("DNS Fail") || state_timer >= TICK_TCP_TIMEOUT) {
            printf("ESP8266: time HTTP connect fail\r\n");
            Clock_NetSyncFail();
            ESP8266_ClearRx();
            state_timer = 0;
            esp_state   = ESP_STATE_TIME_DONE;
        }
        break;

    /* ---- HTTP 授时：等响应头里的 Date（服务器时间，GMT） ---- */
    case ESP_STATE_TIME_HTTP_GET:
    {
        char *p;

        ESP8266_RxSlide();                 /* 响应头可能几百字节，先滑窗口 */
        p = strstr(ESP8266_RxStr(), "Date: ");
        if (p) {
            DS1302_TIME nt;
            if (TimeParse_HttpDate(p, &nt)) {
                ESP8266_ClearRx();
                Clock_NetSyncOk(&nt);
                state_timer = 0;
                esp_state   = ESP_STATE_TIME_DONE;
                break;
            }
            printf("ESP8266: HTTP Date parse fail\r\n");
            ESP8266_ClearRx();
        }

        if (ESP8266_RxHas("CLOSED") || state_timer >= TICK_TIME_HTTP_WAIT) {
            printf("ESP8266: time HTTP timeout\r\n");
            Clock_NetSyncFail();
            ESP8266_ClearRx();
            state_timer = 0;
            esp_state   = ESP_STATE_TIME_DONE;
        }
        break;
    }

    /* ---- 授时收尾：关掉临时 TCP，回 MQTT 流程 ---- */
    case ESP_STATE_TIME_DONE:
        if (!close_sent) {
            ESP8266_SendCmd("AT+CIPCLOSE\r\n");
            close_sent  = 1;
            state_timer = 0;
            printf("ESP8266: time sync done, clock src = %s\r\n",
                   (Clock_GetSource() == CLOCK_SRC_NET) ? "NET" : "RTC");
        }
        if (ESP8266_RxHas("OK") || ESP8266_RxHas("ERROR") ||
            ESP8266_RxHas("CLOSED") || state_timer >= TICK_CIPCLOSE_WAIT) {
            ESP8266_ClearRx();
            ESP8266_SendCmd("AT+CIPMUX=0\r\n");
            close_sent  = 0;
            state_timer = 0;
            esp_state   = ESP_STATE_TCP_START;
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
                uint16_t off = (uint16_t)(ipd_data - (uint8_t *)rx_buf);

                /* 声明长度可能大于缓冲里实际到的字节（帧跨轮到达）：
                   不截断就直接解析，MQTT_ParseConnack 会读到 rx_buf 之外；
                   而半帧当场判失败又会白白重连一次。
                   所以没到齐就等下一轮，由 state_timer 超时和 15s 保底兜底 */
                if ((uint32_t)off + ipd_len <= (uint32_t)rx_len) {
                    uint8_t result = MQTT_ParseConnack(ipd_data, ipd_len);
                    if (result == 0) {
                        printf("ESP8266: MQTT Online! (OneNET)\r\n");
                        mqtt_online = 1;
                        mqtt_timer  = 0;
                        ESP8266_ClearRx();
                        esp_state   = ESP_STATE_MQTT_ONLINE;
                        ESP8266_SendSubscribe();   /* 订阅云端属性设置，等云端下发控制 */
                    } else {
                        printf("ESP8266: CONNACK err=%d\r\n", result);
                        esp_state = ESP_STATE_TCP_DISCONNECT;
                    }
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
        /* ==== 下行最先处理 ====
           本分支后面的重订、上报、心跳都会走 TCPSend，而它内部会清接收缓冲：
           先清后查 = 这一轮到达的云端命令原地消失（订阅 QoS0，平台不会重发） */
        ESP8266_ProcessDownlink();

        /* ==== 周期校时探测：放最前面，它的 OK/ERROR 不能被当成 MQTT 断线 ==== */
        if (probe_on) {
            probe_timer++;
            if (ESP8266_RxHas("+CIPSNTPTIME:")) {
                DS1302_TIME nt;
                if (TimeParse_SntpTime(ESP8266_RxStr(), &nt)) {
                    Clock_NetSyncOk(&nt);
                } else {
                    Clock_NetSyncFail();
                }
                ESP8266_ClearRx();
                probe_on = 0;
            } else if (ESP8266_RxHas("ERROR")) {
                sntp_supported = 0;     /* 这版 AT 固件不支持 SNTP，本轮到下次联网前不再试 */
                printf("ESP8266: SNTP unavailable\r\n");
                Clock_NetSyncFail();
                ESP8266_ClearRx();
                probe_on = 0;
            } else if (probe_timer >= TICK_SNTP_PROBE_WAIT) {
                Clock_NetSyncFail();    /* 无应答：记一次失败，退避 60s 后再试 */
                probe_on = 0;
            }
            /* 探测窗口内不发 MQTT 报文：AT 正忙时 CIPSEND 会被拒，
               紧接着就被当成掉线。代价是本轮 mqtt_timer 不推进，
               所以窗口必须短（3s）且失败必须退避 —— 不退避就是探测风暴，
               上报和心跳会被一直挤掉（见 clock.c 的 Clock_NetSyncDue） */
            break;
        }
        if (sntp_supported && Clock_NetSyncDue()) {
            printf("ESP8266: SNTP recheck\r\n");
            Clock_NetSyncTry();
            ESP8266_ClearRx();
            ESP8266_SendCmd("AT+CIPSNTPTIME?\r\n");
            probe_on    = 1;
            probe_timer = 0;
            break;
        }

        mqtt_timer++;

        /* 订阅一直没被平台确认就周期性重订：
           漏了订阅 = 云端下发永远收不到，表面上还看不出来 */
        if (!sub_ok && (mqtt_timer % TICK_MQTT_SUB_RETRY) == 0) {
            ESP8266_SendSubscribe();
        }

        /* 检测 TCP 断开：只认模块明确报的 TCP 关闭。
           这里不能带 "ERROR" —— AT 层的 ERROR（上一条命令被拒、SNTP 不支持、
           透传退出等）和真正的断链共用同一串关键字，拿它判掉线会误重连，
           表现成"偶发上报中断"。真断链另有两道兜底：上报/心跳发不出去会重连、
           15s 绝对超时保底 */
        if (ESP8266_RxHas("CLOSED")) {
            printf("ESP8266: TCP lost (CLOSED)\r\n");
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
                abs_ticks = 0;      /* 报文真发出去了 = 有进展，15s 保底重新计时 */
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

        /* 防止 mqtt_timer 溢出 */
        if (mqtt_timer >= 60000) mqtt_timer = 0;
        break;

    /* ---- 断开 TCP（等 CIPCLOSE 完成再重试） ---- */
    case ESP_STATE_TCP_DISCONNECT:
        mqtt_online = 0;
        mqtt_timer  = 0;
        sub_ok      = 0;
        Relay_Cloud_OnMqttLost();   /* MQTT 掉线 → 控制权交还本地联动（fail-safe） */
        if (!close_sent) {
            ESP8266_SendCmd("AT+CIPCLOSE\r\n");
            close_sent  = 1;
            state_timer = 0;
            printf("ESP8266: CIPCLOSE sent\r\n");
        }
        /* 等待 CIPCLOSE 完成，避免 TCP 状态残留 */
        if (ESP8266_RxHas("CLOSED") || ESP8266_RxHas("OK") ||
            ESP8266_RxHas("ERROR") || state_timer >= TICK_CIPCLOSE_WAIT) {
            ESP8266_ClearRx();
            close_sent = 0;
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

    /* 状态内超时：仅在无新数据时推进 */
    if (!new_data) {
        state_timer++;
    } else {
        state_timer = 0;
    }

    /* 15s 保底只认"真进展"：状态发生变化，或 MQTT 在线时成功发出过报文。
       不能拿"收到任意字节"当进展 —— 模块供电不足反复重启时会一直喷日志，
       计时器每轮都被清零，于是既没有状态超时也没有保底，
       状态机卡在 WAIT_AT / WAIT_CWJAP 里再也出不来 */
    if (esp_state != last_state) {
        last_state = esp_state;
        abs_ticks  = 0;
    }

    abs_ticks++;
    /* 绝对超时保底：连续 15s 既没新数据也没状态变化，才强制复位 */
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
        last_state  = ESP_STATE_INIT;
        close_sent  = 0;
        probe_on    = 0;
        mqtt_online = 0;
        mqtt_timer  = 0;
        sub_ok      = 0;
        Relay_Cloud_OnMqttLost();
    }
}
