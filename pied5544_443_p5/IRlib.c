/** @file IRlib.c
 * 
 * @brief
 * Contains functions to communicate with the SMBus IR sensor.
 *
 * @author
 * Parker Piedmont
 * 
 * @date
 * 04 Oct 2021
 */

// PIC32 includes
#include <plib.h>

// Cerebot includes
#include "CerebotMX7cK.h"

// C libraries
#include <stdint.h>

// header for this file
#include "IRlib.h"

/*!
 * @brief
 * Reads from the IR sensor using SMBus.
 * 
 * @param[in] slave_addr  Slave address of the IR sensor
 * @param[in] command	  Instructions sent to sensor
 * @param[out] data		  Data read from sensor
 * @param[in] data_len    Number of bytes to read
 * 
 * @return Whether an error occurred
 */
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

/*** end of file ***/