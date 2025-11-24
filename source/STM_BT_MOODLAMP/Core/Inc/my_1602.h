#ifndef __MY_1602_H__
#define __MY_1602_H__

#include "stm32f4xx_hal.h"

// LCD ?? (0x27 ?? 0x3F ? ??? ? ??)
#define LCD_I2C_ADDR (0x27 << 1)  // 0x27 ?? 0x3F ?? ??

void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_send_data(uint8_t data);
void lcd_send_string(char *str);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);

void lcd_create_char(uint8_t location, uint8_t charmap[]);

//----------------------------------------------------
void lcd_send_Speed(uint8_t speed);
void lcd_replace_char(uint8_t row, uint8_t colMin, uint8_t colMax, uint8_t cgromArr);

#endif // __MY_1602_H__