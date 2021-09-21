/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#include <plib.h>
#include "CerebotMX7cK.h"
#include "eeprom.h"

int I2CReadEEPROM(char SlaveAddress, int mem_addr, char* data, int data_len)
{
    char ctrl_byte_w, ctrl_byte_r, addr_hi, addr_lo;
    int i = 0, write_err = 0;

    ctrl_byte_w = ((SlaveAddress << 1) | 0);
    addr_hi = ((mem_addr >> 8) & 0xff);
    addr_lo = mem_addr & 0xff;

    ctrl_byte_r = ((SlaveAddress << 1) | 1);

    StartI2C2();
    IdleI2C2();

    write_err |= MasterWriteI2C2(ctrl_byte_w);
    write_err |= MasterWriteI2C2(addr_hi);
    write_err |= MasterWriteI2C2(addr_lo);

    RestartI2C2();

    MasterWriteI2C2(ctrl_byte_r);   // Reverse bus direction

    while (data_len--)
        data[i++] = MasterReadI2C2();
    
//    do
//    {
//        data[i++] = MasterReadI2C2();
//    }
//    while (data[i] != 0 && data[i] != '\r' && data[i] != '\n');

    NotAckI2C2();
    IdleI2C2();
    StopI2C2();
    IdleI2C2();

    return write_err;
}

int I2CWriteEEPROM(char SlaveAddress, int mem_addr, char* data, int data_len)
{
    char ctrl_byte, addr_hi, addr_lo;
    int i = 0, write_err = 0;

    ctrl_byte = ((SlaveAddress << 1) | 0);
    addr_hi = ((mem_addr >> 8) & 0xff);
    addr_lo = mem_addr & 0xff;

    StartI2C2();
    IdleI2C2();

    write_err |= MasterWriteI2C2(ctrl_byte);
    write_err |= MasterWriteI2C2(addr_hi);
    write_err |= MasterWriteI2C2(addr_lo);

    while (data_len--)
    //while (data[i] != 0)// && data[i] != '\r' && data[i] != '\n')
    {
        write_err |= MasterWriteI2C2(data[i++]);
        mem_addr++;

        if (mem_addr % 64 == 0)
        {
            StopI2C2();
            IdleI2C2();

            wait_i2c_xfer(SlaveAddress);    // Wait for page write

            // Recompute address bytes for new page
            addr_hi = ((mem_addr >> 8) & 0xff);
            addr_lo = mem_addr & 0xff;

            StartI2C2();
            IdleI2C2();

            write_err |= MasterWriteI2C2(ctrl_byte);
            write_err |= MasterWriteI2C2(addr_hi);
            write_err |= MasterWriteI2C2(addr_lo);
        }
    }

    StopI2C2();
    IdleI2C2();

    wait_i2c_xfer(SlaveAddress);

    return write_err;
}

int wait_i2c_xfer(int SlaveAddress)
{
    char ctrl_byte = ((SlaveAddress << 1) | 0);
    int write_err = 0;

    StartI2C2();
    IdleI2C2();

    while (/*write_err |=*/ MasterWriteI2C2(ctrl_byte))
    {
        RestartI2C2();
        IdleI2C2();
    }
    
    StopI2C2();
    IdleI2C2();

    return write_err;
}

char BusyI2C2(void)
{
    return (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RSEN ||
            I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
}
