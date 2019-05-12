#include "MPU9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "header.h"
#include "ml_math_func.h"
void DMPSetup(void)
{
	u8 errcode;
	errcode = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	if (!errcode)
		printf("error:fail to setup the sensor for DMP function\r\n");
  errcode = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); /* Push both gyro and accel data into the FIFO. */
	if (!errcode)
		printf("error:fail to pudh gyro and accel data into FIFO\r\n");
  errcode = mpu_set_sample_rate(DEFAULT_MPU_HZ);	
}




