/***************STM32F103C8T6**********************
 * 文件名  ：main.c
 * 功能    ：DHT11温湿度传感器 + DS1302时钟 + OLED显示 + ESP8266 WiFi上传
 * 备注    ：主程序入口
 * 接口    ：参见各模块头文件

********************LIGEN*************************/

#include "stm32f10x.h"
#include "led.h"
#include "usart1.h"
#include "delay.h"
#include "dht11.h"
#include "ds1302.h"
#include "oled.h"
#include "esp8266.h"
#include <string.h>

volatile u8 temp;
volatile u8 humi;
DS1302_TIME time;

int main(void)
{
    SystemInit();
    LED_GPIO_Config();
    USART1_Config();
    Delay_init(72);

    OLED_Init();
    OLED_Clear();

    printf("Start \n");
    while (DHT11_Init())
    {
        printf("DHT11 Error \r\n");
        Delay_ms(1000);
    }

    DS1302_Init();

    ESP8266_Init();

    time.year    = 26;
    time.month   = 5;
    time.day     = 29;
    time.hour    = 1;
    time.minute  = 16;
    time.second  = 0;
    //DS1302_SetTime(&time);

    {
        static uint16_t oled_tick = 0;

        while (1)
        {
            ESP8266_Process();

            oled_tick++;
            if (oled_tick >= 50) {
                oled_tick = 0;

                DHT11_Read_Data(&temp, &humi);
                DS1302_GetTime(&time);

                printf("20%02d/%02d/%02d %02d:%02d:%02d\r\n",
                       time.year, time.month, time.day,
                       time.hour, time.minute, time.second);
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

                LED_Toggle();
            }

            Delay_ms(20);
        }
    }
}
