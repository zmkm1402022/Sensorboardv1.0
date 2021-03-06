/*********************************************************************
File    : i2c.h
Purpose : 
**********************************************************************/
#ifndef __I2C_H__
#define __I2C_H__
#include "header.h"
/****************************** Includes *****************************/
/****************************** Defines *******************************/
#define SENSORS_I2C               I2C2

#define I2C_SPEED                 200000
#define I2C_OWN_ADDRESS           0x68

#define I2C_Config() I2cMaster_Init();

void I2cMaster_Init(void);
void Set_I2C_Retry(unsigned short ml_sec);
uint32_t Get_I2C_Retry(void);

int Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, 
                                          unsigned short RegisterLen, unsigned char *RegisterValue);
int Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, 
                                           unsigned short RegisterLen, const unsigned char *RegisterValue);
 
#endif // __I2C_H__


