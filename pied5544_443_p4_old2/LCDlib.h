/** @file LCDlib.h
 * 
 * @brief
 * Declares functions to communicate with the LCD using the PMP.
 *
 * @author
 * Parker Piedmont
 * 
 * @date
 * 02 Nov 2020
 */

// #include guard
#ifndef __LCDLIB_H__
#define __LCDLIB_H__

// Function prototypes
void LCD_init(void);
char readLCD(int addr);
void writeLCD(int addr, char c);
void LCD_putc(char c);
void LCD_puts(char *char_string);
void LCD_clear(void);
void LCD_clearline(int line);
void LCD_set_cursor_pos(int line, int pos);
char busyLCD(void);
void LCD_delay(unsigned int ms);

#endif

/*** end of file ***/