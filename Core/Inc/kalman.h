/*
 * kalman.h
 *
 *  Created on: Jun 4, 2024
 *      Author: vovav
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

void getKalmanA();
void getKalmanCSE();
void getKalmanOMEGA();

void getKalmanVGPS();

double simpleKalmanYAW(double newVal9);

#endif /* INC_KALMAN_H_ */
