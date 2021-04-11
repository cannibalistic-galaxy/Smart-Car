#ifndef _OLED_H
#define _0LED_H

extern uint8 fushu[];
extern uint8 null[];

extern void OLED_Init();
extern void OLED_CLS();
extern void OLED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
extern void OLED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
extern void OLED_P32x64Str(uint8 x,uint8 y,uint8 ch[]);
extern void OLED_P8x16_Clr_One(uint8 x,uint8 y);
extern void OLED_P14x16Str(uint8 x,uint8 y,uint8 ch[],uint8 which); //∫∫◊÷œ‘ æ
extern void OLED_Print(uint8 x, uint8 y, uint8 ch[]);
extern void OLED_PutPixel(uint8 x,uint8 y);
extern void OLED_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif);
extern void OLEDInADNormalization(void);
extern void OLEDDisplay(uint16 *a);
extern void Dis_Num(uint8 y, uint8 x,int16 num,uint8 N);
extern void OLED_ShowData(int ad,uint8 column,uint8 line);
extern void Dis_Num_P32x64(int16 num,uint8 N);
extern void OLED_DLY_ms(uint16 ms);
//extern void OLED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
#endif

