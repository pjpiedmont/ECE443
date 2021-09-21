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

#define Fsck        400000
#define BRG_VAL     ((FPB / 2 / Fsck) - 2)
#define DATA_LEN    64

int I2CReadEEPROM(char SlaveAddress, int mem_addr, char* data, int data_len);
int I2CWriteEEPROM(char SlaveAddress, int mem_addr, char* data, int data_len);
int wait_i2c_xfer(int SlaveAddress);
char BusyI2C2(void);

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
