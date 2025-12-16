// i2c_lcd.h
#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "driver/i2c.h" // Cần include cái này để dùng kiểu dữ liệu i2c_port_t

#define LCD_ADDR 0x27

// Xóa các define SDA_PIN, SCL_PIN cũ đi vì LCD không còn quyền quyết định chân nữa

// Sửa hàm init để nhận tham số port
void lcd_init(i2c_port_t port_num);

void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_put_cur(int row, int col);
void lcd_clear(void);

#endif