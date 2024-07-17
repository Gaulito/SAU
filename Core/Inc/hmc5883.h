#ifndef INC_HMC5883_H_
#define INC_HMC5883_H_

#define DEVICE_ADRESS1			0x0D //0x0D Видимо, китайский датчик имеет анормальный адрес, не как в Datasheet.

#define HMC5883L_RA_CONFIG_A    0 //
#define HMC5883L_RA_CONFIG_B    1 //
#define HMC5883L_RA_MODE        2 //
#define HMC5883L_RA_DATAX_H    	3
#define HMC5883L_RA_DATAX_L     4
#define HMC5883L_RA_DATAZ_H     5
#define HMC5883L_RA_DATAZ_L     6
#define HMC5883L_RA_DATAY_H     7
#define HMC5883L_RA_DATAY_L     8
#define HMC5883L_RA_STATUS      9
#define HMC5883L_RA_ID_A        10
#define HMC5883L_RA_ID_B        11
#define HMC5883L_RA_ID_C        12

#define HERNYA_9 9
#define HERNYA_10 10
#define HERNYA_11 11

#define HERNYA_CHITAT 0
#define HERNYA_Z9 89
#define HERNYA_Z10 65
#define HERNYA_Z11 1

#define HMC5883L_CONFIG_A       112

void hmc5883_init();
void hmc5883_read();

double hmc5883_getYaw (double Pitch, double Roll);

#endif /* INC_HMC5883_H_ */
