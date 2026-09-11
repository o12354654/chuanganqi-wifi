/***************STM32F103C8T6**********************
 * 文件名  ：main.c
 * 功能    ：DHT11温湿度传感器 + 网络授时/DS1302时钟 + OLED显示 + ESP8266 WiFi上传
 * 备注    ：时间优先从网络授时拿，拿不到才用 DS1302 走时
 * 接口    ：参见各模块头文件

********************LIGEN*************************/

#include "stm32f10x.h"
#include "usart1.h"
#include "delay.h"
#include "dht11.h"
#include "ds1302.h"
#include "oled.h"
#include "clock.h"
#include "esp8266.h"
#include <string.h>

u8 temp;
u8 humi;
DS1302_TIME time;

/**
 * @brief  此刻的时间是不是"网络在管"
 * @note   = WiFi 链路在(ESP8266_IsConnected) 且 成功网络授时过(Clock_IsSynced)
 *         跟云端(OneNET MQTT)在不在线无关：SNTP 走的是阿里云 NTP，云端鉴权过期也照样算网络在管
 *         WiFi 掉线 / 从没授过时 → 显示 R（此时用的是上一次网络校准过的 RTC 在走时）
 */
static uint8_t net_time_active(void)
{
    return (uint8_t)(ESP8266_IsConnected() && Clock_IsSynced());
}

int main(void)
{
    SystemInit();
    USART1_Config();
    Delay_init(72);

    OLED_Init();
    OLED_Clear();

    printf("Start \n");

    /* 时钟：TIM2 毫秒节拍 + 读 DS1302；
       DS1302 时间无效或停振(新片/掉电)会自动写默认时间并启动走时 */
    Clock_Init();

    while (DHT11_Init())
    {
        printf("DHT11 Error \r\n");
        Delay_ms(1000);
    }

    ESP8266_Init();   /* 联网后 ESP8266 会优先做网络授时 */

    {
        static uint16_t oled_tick = 0;

        while (1)
        {
            Clock_Task();        /* 推进软时钟，非阻塞 */
            ESP8266_Process();

            oled_tick++;
            if (oled_tick >= 50) {
                oled_tick = 0;

                if (DHT11_Read_Data(&temp, &humi) != 0) {
                    printf("DHT11 read fail\r\n");
                }
                Clock_GetTime(&time);   /* 时间来源：网络 / DS1302 */

                printf("20%02d/%02d/%02d %02d:%02d:%02d  src=%s\r\n",
                       time.year, time.month, time.day,
                       time.hour, time.minute, time.second,
                       net_time_active() ? "NET" : "RTC");
                printf("temp:%d humi:%d%%\r\n", temp, humi);

                OLED_ShowNum(32, 0, 20, 2, 16);
                OLED_ShowNum(48, 0, time.year, 2, 16);
                OLED_ShowString(64, 0, "/");

                if (time.month < 10) {
                    OLED_ShowNum(72, 0, 0, 1, 16);
                    OLED_ShowNum(80, 0, time.month, 1, 16);
                } else {
                    OLED_ShowNum(72, 0, time.month, 2, 16);
                }
                OLED_ShowString(88, 0, "/");
                if (time.day < 10) {
                    OLED_ShowNum(96, 0, 0, 1, 16);
                    OLED_ShowNum(104, 0, time.day, 1, 16);
                } else {
                    OLED_ShowNum(96, 0, time.day, 2, 16);
                }

                /* 时间来源标记：N = 此刻网络在管（云端在线且授过时），R = 此刻靠本机 RTC 走 */
                OLED_ShowString(32, 2, (u8 *)(net_time_active() ? "N" : "R"));

                if (time.hour < 10) {
                    OLED_ShowNum(48, 2, 0, 1, 16);
                    OLED_ShowNum(56, 2, time.hour, 1, 16);
                } else {
                    OLED_ShowNum(48, 2, time.hour, 2, 16);
                }
                OLED_ShowString(64, 2, ":");

                if (time.minute < 10) {
                    OLED_ShowNum(72, 2, 0, 1, 16);
                    OLED_ShowNum(80, 2, time.minute, 1, 16);
                } else {
                    OLED_ShowNum(72, 2, time.minute, 2, 16);
                }
                OLED_ShowString(88, 2, ":");

                if (time.second < 10) {
                    OLED_ShowNum(96, 2, 0, 1, 16);
                    OLED_ShowNum(104, 2, time.second, 1, 16);
                } else {
                    OLED_ShowNum(96, 2, time.second, 2, 16);
                }

                OLED_ShowCHinese(32, 4, 5);
                OLED_ShowCHinese(48, 4, 6);
                OLED_ShowCHinese(64, 4, 8);

                if (temp < 10) {
                    OLED_ShowNum(80, 4, 0, 1, 16);
                    OLED_ShowNum(88, 4, temp, 1, 16);
                } else {
                    OLED_ShowNum(80, 4, temp, 2, 16);
                }
                OLED_ShowCHinese(96, 4, 6);

                OLED_ShowCHinese(32, 6, 7);
                OLED_ShowCHinese(48, 6, 6);
                OLED_ShowCHinese(64, 6, 8);

                if (humi < 10) {
                    OLED_ShowNum(80, 6, 0, 1, 16);
                    OLED_ShowNum(88, 6, humi, 1, 16);
                } else {
                    OLED_ShowNum(80, 6, humi, 2, 16);
                }
                OLED_ShowString(96, 6, "%");
            }

            Delay_ms(20);
        }
    }
}
