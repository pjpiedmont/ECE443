#include <plib.h>
#include "CerebotMX7cK.h"
#include "LCDlib.h"

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

char readLCD(int addr)
{
    PMPSetAddress(addr);         // Set LCD RS control
    mPMPMasterReadByte();        // initiate dummy read sequence
    return mPMPMasterReadByte(); // read actual data
} // End of readLCD

void writeLCD(int addr, char c)
{
    while (busyLCD());     // Wait for LCD to be ready
    PMPSetAddress(addr);  // Set LCD RS control
    PMPMasterWrite(c);    // initiate write sequence
} // End of writeLCD

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

void LCD_puts(char *char_string)
{
    while (*char_string) // Look for end of string NULL character
    {
        LCD_putc(*char_string); // Write character to LCD
        char_string++; // Increment string pointer
    }
}

void LCD_clear(void)
{
    writeLCD(0, 0x01);
}

//void LCD_clearline(void)
//{
//    char line_addr;
//    int i;
//
//    if (0x00 <= readLCD(0) <= 0x0f)
//        line_addr = 0x80;
//    else
//        line_addr = 0xc0;
//
//    writeLCD(0, line_addr);
//
//    for (i = 0; i < 16; i++)
//        LCD_putc(' ');
//
//    writeLCD(0, line_addr);
//}

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

char busyLCD(void)
{
    char busy_flag = readLCD(0) & 0x80;
    return busy_flag;
}

void LCD_delay(unsigned int ms)
{
	unsigned int tWait, tStart;
	tStart = ReadCoreTimer();
	tWait = (CORE_MS_TICK_RATE * ms);
	while ((ReadCoreTimer() - tStart) < tWait);
	LATBINV = LEDA;
}