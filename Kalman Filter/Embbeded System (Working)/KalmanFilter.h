/*
 * KalmanFilter.h
 *
 *  Created on: 21/11/2019
 *      Author: loya_
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

//Earth Radious
#define Rea  6378137
#define e  0.08181919
#define PI 3.1416
#define LONGBASE -(103.417415)*0.0175
#define LATBASE (20.606715)*0.0175
#define ALTBASE (1584.84915)


//void MatrixMult(int Row1, int Col1,int Row2, int Col2, float Matrix1[M][N], float Matrix2[M][1]);

void LLA2NED(double Longitude, double Latitude, double Altitude, float* North, float* East, float* Down);

void MatrixMult(double *Matrix, double * Matrix2, int m, int q, int p);
void multiplyMatrices(double firstMatrix[][3], double secondMatrix[][3], double mult[][3], int rowFirst, int columnFirst, int rowSecond, int columnSecond);

void KalmanFilter(double deltat, double Yaw, double Acc, double N, double E, double D);
#endif /* KALMANFILTER_H_ */
