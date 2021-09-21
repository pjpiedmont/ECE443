/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _EXAMPLE_FILE_NAME_H    /* Guard against multiple inclusion */
#define _EXAMPLE_FILE_NAME_H

int I2CReadEEPROM(char SlaveAddress, int mem_addr, char* data, int data_len);
int I2CWriteEEPROM(char SlaveAddress, int mem_addr, char* data, int data_len);
int wait_i2c_xfer(int SlaveAddress);
char BusyI2C2(void);

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
