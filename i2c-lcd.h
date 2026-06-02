#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "main.h"

void lcd_init(void);
void lcd_clear(void);
void lcd_put_cur(uint8_t row, uint8_t col);
void lcd_send_string(char *str);

#endif