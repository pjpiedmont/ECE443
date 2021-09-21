// #include guard
#ifndef __LCDLIB_H__
#define __LCDLIB_H__

void LCD_init(void);
char readLCD(int addr);
void writeLCD(int addr, char c);
void LCD_putc(char c);
void LCD_puts(char *char_string);
void LCD_clear(void);
char busyLCD(void);
void LCD_delay(unsigned int ms);

#endif