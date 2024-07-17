#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#define BMP180_ADDRESS 			0xEE

void bmp180_init (void);

float bmp180_gettemp (void);

float bmp180_getpress (int oss);

float bmp180_getalt (int oss);

#endif /* INC_BMP180_H_ */
