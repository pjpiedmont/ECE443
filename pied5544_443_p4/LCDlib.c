/** @file LCDlib.c
 * 
 * @brief
 * Contains functions to communicate with the LCD using the PMP.
 *
 * @author
 * Parker Piedmont
 * 
 * @date
 * 02 Nov 2020
 */

// PIC32 includes
#include <plib.h>

// Cerebot includes
#include "CerebotMX7cK.h"

// header for this file
#include "LCDlib.h"

/*!
 * @brief
 * Initializes the LCD by clearing it and setting the cursor in the top left
 * corner.
 * 
 * @return None
 */
void LCD_init(void)
{
    LCD_delay(20);
    writeLCD(0, 0x38);
    LCD_delay(50);
    writeLCD(0, 0x0f);
    LCD_delay(50);
    writeLCD(0, 0x01);
    LCD_delay(10);
}

/*!
 * @brief
 * Reads from one of the LCD's registers.
 * 
 * @param[in] addr  Address of the register to read
 * 
 * @return Value of the register
 */
char readLCD(int addr)
{
    PMPSetAddress(addr);         // Set LCD RS control
    mPMPMasterReadByte();        // initiate dummy read sequence
    return mPMPMasterReadByte(); // read actual data
}

/*!
 * @brief
 * Writes to one of the LCD's registers.
 * 
 * @param[in] addr  Address of the register to write
 * @param[in] c	  	Value to write
 * 
 * @return None
 */
void writeLCD(int addr, char c)
{
    while (busyLCD());     // Wait for LCD to be ready
    PMPSetAddress(addr);  // Set LCD RS control
    PMPMasterWrite(c);    // initiate write sequence
}

/*!
 * @brief
 * Prints one character to the LCD and advances the cursor. Sets cursor to the
 * start of the next line on \r or to the start of the current line on \n.
 * 
 * @param[in] c  Character to print
 * 
 * @return None
 */
void LCD_putc(char c)
{
    unsigned int addr;

    while (busyLCD());
    addr = readLCD(0) & 0x7f;
    
    switch (c)
    {
        case '\r':
            if ((addr >= 0x00) && (addr < 0x40))
                writeLCD(0, 0xc0);
            else
                writeLCD(0, 0x80);
            break;

        case '\n':
            if ((addr >= 0x00) && (addr < 0x40))
                writeLCD(0, 0x80);
            else
                writeLCD(0, 0xc0);
            break;

        default:
            if ((addr > 0x0f) && (addr < 0x40))
                writeLCD(0, 0xc0);
            if ((addr > 0x4f) && (addr < 0x80))
                writeLCD(0, 0x80);
            writeLCD(1, c);
            break;
    }
}

/*!
 * @brief
 * Prints an entire string to the LCD.
 * 
 * @param[in] char_string  String to print
 * 
 * @return None
 */
void LCD_puts(char *char_string)
{
    while (*char_string) // Look for end of string NULL character
    {
        LCD_putc(*char_string); // Write character to LCD
        char_string++; // Increment string pointer
    }
}

/*!
 * @brief
 * Clears the LCD and sets the cursor to the top left corner.
 * 
 * @return None
 */
void LCD_clear(void)
{
    writeLCD(0, 0x01);
}

/*!
 * @brief
 * Clears one line of the LCD and sets the cursor to the beginning of the line.
 * 
 * @param[in] line  Row to clear
 * 
 * @return None
 */
void LCD_clearline(int line)
{
    switch (line)
    {
        case 1:
            writeLCD(0, 0x80);
            LCD_puts("                ");  // 16 spaces
            writeLCD(0, 0x80);
            break;

        default:
            writeLCD(0, 0xc0);
            LCD_puts("                ");  // 16 spaces
            writeLCD(0, 0xc0);
            break;
    }
}

/*!
 * @brief
 * Sets the position of the cursor.
 * 
 * @param[in] line  Row
 * @param[in] pos	Column
 * 
 * @return None
 */
void LCD_set_cursor_pos(int line, int pos)
{
    switch (line)
    {
        case 1:
            writeLCD(0, 0x80 + pos);
            break;
            
        default:
            writeLCD(0, 0xc0 + pos);
            break;
    }
}

/*!
 * @brief
 * Checks whether the LCD is busy writing to a register.
 * 
 * @return Non-zero if busy, zero if not busy
 */
char busyLCD(void)
{
    char busy_flag = readLCD(0) & 0x80;
    return busy_flag;
}

/*** end of file ***/