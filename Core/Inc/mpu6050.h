#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define DEVICE_ADRESS	0x68

#define FS_GYRO_250		0
#define FS_GYRO_500	  	8
#define FS_GYRO_1000 	16
#define FS_GYRO_2000	24

#define FS_ACC_2G		0
#define FS_ACC_4G		8
#define FS_ACC_8G		16
#define FS_ACC_16G		24

#define REG_CONFIG_GYRO	27
#define REG_CONGIG_ACC	28
#define REG_PWR_MGMT	107
#define REG_ACC_DATA	59
#define REG_GYRO_DATA	67

void mpu6050_init();
void mpu6050_read();
void mpu6050_getAngle();

#endif /* INC_MPU6050_H_ */
