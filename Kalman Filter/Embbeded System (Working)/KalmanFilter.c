/*
 * KalmanFilter.c
 *
 *  Created on: 21/11/2019
 *      Author: loya_
 */

#include "KalmanFilter.h"
#include <stdio.h>
#include <math.h>
#include "fsl_debug_console.h"

double Pk[3][3] = {{.2913,0.1966},{0.1966,0.2506}};
double Xp[3][3] = {{-0.2320,0,0},{0.3693,0,0},{0,0,0}};


void LLA2NED(double Longitude, double Latitude, double Altitude, float* North, float* East, float* Down)
{
	double Ren[3][3] =   {{-sin(LATBASE)*cos(LONGBASE), -sin(LATBASE)*sin(LONGBASE), cos(LATBASE)},{-sin(LONGBASE), cos(LONGBASE), 0},{-cos(LATBASE)*cos(LONGBASE),-cos(LATBASE)*sin(LONGBASE), -sin(LATBASE)}};
	double NED[3][3];
	double LongRad = Longitude*0.0175;
	double LatRad = Latitude*0.0175;
	double AltRad= Altitude;
	double Pe[3][3];
	double PeRef[3];
    /** Convert LLA to ECEF */
	double NE = ((Rea)/pow((1-((pow(e,2))*pow(sin(LatRad),2))),3/2));
    Pe[0][0] =(NE+AltRad)*cos(LatRad)*cos(LongRad);
    Pe[1][0] = (NE+AltRad)*cos(LatRad)*sin(LongRad);
    Pe[2][0] = (NE*(1-(pow(e,2)))+AltRad)*sin(LatRad);

    PeRef[0] =(NE+ALTBASE)*cos(LATBASE)*cos(LONGBASE);
    PeRef[1] = (NE+ALTBASE)*cos(LATBASE)*sin(LONGBASE);
    PeRef[2] = (NE*(1-(pow(e,2)))+ALTBASE)*sin(LATBASE);

    /** Convert ECEF to NED */
    Pe[0][0] = Pe[0][0] - PeRef[0];
    Pe[1][0] = Pe[1][0] - PeRef[1];
    Pe[2][0] = Pe[2][0] - PeRef[2];
    //MatrixMult(Ren,Pe,3,3,1);
    multiplyMatrices(Ren, Pe, NED, 3, 3, 3, 1);
    *North = NED[0][0];
    *East = NED[1][0];
    *Down = NED[2][0];
    //MatrixMult(3, 3, 3, 1, Ren, Pe);
}

void MatrixMult(double *Matrix, double * Matrix2, int m, int q, int p)
{
	int c, d, k;
	double multiply[m][p];
	double sum = 0;

	PRINTF("%f ",*(Matrix));

    for (c = 0; c < m; c++) {
      for (d = 0; d < q; d++) {
        for (k = 0; k < p; k++) {
          sum = sum + *(Matrix + ((3*c)+k))*(*(Matrix2 + ((3*k)+d)));
        }
        multiply[c][d] = sum;
        sum = 0;
      }
    }
}

void multiplyMatrices(double firstMatrix[][3], double secondMatrix[][3], double mult[][3], int rowFirst, int columnFirst, int rowSecond, int columnSecond)
{
	int i, j, k;
	// Initializing elements of matrix mult to 0.
	for(i = 0; i < rowFirst; ++i)
	{
		for(j = 0; j < columnSecond; ++j)
		{
			mult[i][j] = 0;
		}
	}
	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i < rowFirst; ++i)
	{
		for(j = 0; j < columnSecond; ++j)
		{
			for(k=0; k<columnFirst; ++k)
			{
				mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}
}

void KalmanFilter(double deltat, double Yaw, double Acc, double N, double E, double D)
{
	double Xe[3][3] ={0};
		double BxData[3][3]={{0,0,0},{0,0,0},{0,0,0}};
		double Data[3][3]={{0,0,0},{0,0,0},{0,0,0}};
		double A[3][3] = {{1, .2},{0,1}};
		double At[3][3] = {{1, 0},{0.2,1}};
		double B[2][3] = {{pow(.2,2)/2},{.2}};
		//double Acc =0.4837;
		//double Yaw =4.4298;

		//Predict
		double Pe[3][3]={{0,0,0},{0,0,0},{0,0,0}};
		double Q[3][3] = {{.01,0},{0,.01}};
		double AxPk[3][3]={{0,0,0},{0,0,0},{0,0,0}};

		//KALMAN GAIN
		double Kk[3][3]={{0,0,0},{0,0,0},{0,0,0}};
		double KkDiv[3][3]={{0,0,0},{0,0,0},{0,0,0}};
		double CxPe[3][3]={{0,0,0},{0,0,0},{0,0,0}};
		double CxPexCt[3][3]={{0,0,0},{0,0,0},{0,0,0}};
		double C[3][3]={{1,0,0},{0,0,0},{0,0,0}};
		double Ct[3][3]={{1,0,0},{0,0,0},{0,0,0}};

		// Pk
		double KkxC[3][3] ={0};
		double I[2][2] = {{1,0},{0,1}};
		double YawRad = Yaw*3.1416/180;

		Data[0][0]=Acc*cos(YawRad);

		multiplyMatrices(A, Xp, Xe, 2, 2, 2, 1);

		multiplyMatrices(B, Data, BxData, 2, 1, 1, 1);

		Xe[0][0] = Xe[0][0] +BxData[0][0];
		Xe[1][0] = Xe[1][0] +BxData[1][0];

		multiplyMatrices(A, Pk, AxPk, 2, 2, 2, 2);

		multiplyMatrices(AxPk, At, Pe, 2, 2, 2, 2);
		Pe[0][0] = Pe[0][0] + Q[0][0];
		Pe[0][1] = Pe[0][1] + Q[0][1];
		Pe[1][0] = Pe[1][0] + Q[1][0];
		Pe[1][1] = Pe[1][1] + Q[1][1];

		//Kalman gain operation
		multiplyMatrices(C, Pe, CxPe, 1, 2, 2, 2);
		multiplyMatrices(CxPe, Ct, CxPexCt, 1, 2, 2, 1);
		CxPexCt[0][0] = CxPexCt[0][0] + 1;
		multiplyMatrices(Pe, Ct, Kk, 2, 2, 2, 1);
		Kk[0][0] = Kk[0][0]/CxPexCt[0][0];
		Kk[1][0] = Kk[1][0]/CxPexCt[0][0];

		//Calculate Xp
		KkDiv[0][0] = Kk[0][0]*(N-Xe[0][0]);
		KkDiv[1][0] = Kk[1][0]*(N-Xe[0][0]);
		Xp[0][0] = Xe[0][0]+KkDiv[0][0];
		Xp[1][0] = Xe[1][0]+KkDiv[1][0];

		//printf("")

		//Calculate Pk
		multiplyMatrices(Kk, C, KkxC, 2, 1, 1, 2);
		KkxC[0][0] = I[0][0] - KkxC[0][0];
		KkxC[0][1] = I[0][1] - KkxC[0][1];
		KkxC[1][0] = I[1][0] - KkxC[1][0];
		KkxC[1][1] = I[1][1] - KkxC[1][1];

		multiplyMatrices(KkxC, Pe, Pk, 2, 2, 2, 2);

}
