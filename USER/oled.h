#ifndef __OLED_H
#define __OLED_H
#include "sys.h"

#define OLED_MODE 0
#define SIZE 16
#define XLevelL     0x00
#define XLevelH     0x10
#define Max_Column  128
#define Max_Row     64
#define Brightness  0xFF
#define X_WIDTH     128
#define Y_WIDTH     64

#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define OLED_SCLK_Set() GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define OLED_SDIN_Set() GPIO_SetBits(GPIOB, GPIO_Pin_11)

#define OLED_CMD  0
#define OLED_DATA 1

void OLED_WR_Byte(u8 dat, u8 cmd);
void OLED_Display_On(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(u8 x, u8 y, u8 chr);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size);
void OLED_ShowString(u8 x, u8 y, u8 *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x, u8 y, u8 no);

#endif
