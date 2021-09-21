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

void LCD_puts(char* char_string)
{
//    writeLCD(0, 0x80);
    
    while (*char_string) // Look for end of string NULL character
    {
        LCD_putc(*char_string); // Write character to LCD
        char_string++; // Increment string pointer
    }
} // End of LCD_puts

void LCD_clear(void)
{
    char blank[] = "                                ";
    LCD_puts(blank);
    writeLCD(0, 0x80);
    writeLCD(0, 0x80); // I have no idea why, but if this isn't here the text gets messed up
    char addr = readLCD(0);
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