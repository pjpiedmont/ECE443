/** @file IRlib.h
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

#ifndef __IRLIB_H__
#define __IRLIB_H__

int I2C1_IR_Read(uint8_t slave_addr, uint8_t command, uint8_t* data, int data_len);

#endif  // __IRLIB_H__

/*** end of file ***/