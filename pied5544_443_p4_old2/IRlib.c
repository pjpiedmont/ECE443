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
#include "IRlib.h"

int I2C1_IR_Read(uint8_t slave_addr, uint8_t command, uint8_t* data, int data_len)
{
    uint8_t ctrl_byte_w;
    uint8_t ctrl_byte_r;
    char temp;
    
    int write_err = 0;

    ctrl_byte_w = ((slave_addr << 1) | 0);
    ctrl_byte_r = ((slave_addr << 1) | 1);

    StartI2C1();
    IdleI2C1();

    write_err |= MasterWriteI2C1(ctrl_byte_w);
    write_err |= MasterWriteI2C1(command);

    RestartI2C1();
    IdleI2C1();

    MasterWriteI2C1(ctrl_byte_r);   // Reverse bus direction

    while (data_len--)
    {
        temp = MasterReadI2C1();
        *data = temp;
        
        data++;
        
        if (data_len >= 1)
        {
            AckI2C1();
            IdleI2C1();
        }
    }

    NotAckI2C1();
    IdleI2C1();
    StopI2C1();
    IdleI2C1();

    return write_err;
}
