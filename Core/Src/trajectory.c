#include <trajectory.h>
#include <main.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#define NofPoints 47

extern VECTOR RV0;
//extern double x0;
//extern double z0;

double R = 30;
double Rmax = 300;
double phidrop = -1;
double phiNL = -1;
double Hpolet = 100;
double Hud = 50;
double Hdrop = 35;
double Ldrop = 150;
double Lsap = 50;

double alphagr = M_PI/9;
double Hgr = 1.5;
double Lgru = 10;
double Lgrd = 20;
double input[NofPoints][3] = {{1, 2, 1},{1, 2, 2},{1, 2, 3}, {1, 2, 4}, {1, 2, 5}, {1, 2, 6}, {1, 2, 6}};
double trajectory0[NofPoints][4];


int trajUP_Length, traj10_Length, traj1_Length;

double phi;
double yo;
double xo;
double yo0;
double xo0;
double phi1;
double xb;
double yb;
double xk;
double yk;
double Smax;
double phi0;
double xkp;
double ykp;
double a;
double xmax;
double ymax;
double Smax;
double phi0;
double xkp;
double ykp;
bool Rside;
int calcvar1;
double x1;
double x2;
double y_1;
double y2;
double ret;

double findMinimum(int k) {
    // Assume the first element is the minimum
	double min = input[0][k];
    // Loop through the array to find the minimum
    for (int i = 1; i < 4; i++) {
        if (input[i][k] < min) {
            // Update minimum if a smaller element is found
            min = input[i][k];
        }
    }
    return min;
}

double xo0func(double phi)
{
	double ret=(1/2*a) * tan((phi/2) - phi0) + xmax;
	return(ret);
}

double yo0func(double phi)
{
	double ret = (1/4*a) * pow(tan((phi/2) - phi0),2)+ ymax;
	return(ret);
}

double xofunc(double phi, double R)
{
	if (abs(phi) < 2 * phi0)
	{
		double ret = xo0 * cos(phi0) - yo0 * sin(phi0);
		return(ret);
	}
	else
	{
		double ret = xkp + R * cos(phi0 + (M_PI/2));
		return(ret);
	}
}

double yofunc(double phi, double R)
{
	if (abs(phi) < 2 * phi0)
	{
		double ret = xo0 * sin(phi0) + yo0 * cos(phi0);
		return(ret);
	}
	else
	{
		double ret = ykp + R * sin(phi0 + (M_PI/2));
		return(ret);
	}
}

double xbfunc(double phi, double R)
{
	if (phi == 0)
	{
		double ret = 0;
		return(ret);
	}
	else
	{
		double ret = 2*cos((phi/2)-atan(yo/xo))*sqrt(pow(yo,2)+pow(xo,2))*cos(phi/2);
		return(ret);
	}
}

double ybfunc(double phi, double R)
{
	if (phi == 0)
	{
		double ret = 0;
		return(ret);
	}
	else
	{
		double ret = 2*cos((phi/2)-atan(yo/xo))*sqrt(pow(yo,2)+pow(xo,2))*sin(phi/2);
		return(ret);
	}
}

double xkfunc(double phi, double R, double phi1, double Rside)
{
	if (Rside == true)
	{
		calcvar1= -1;
	}
	if (Rside == false)
	{
		calcvar1= 1;
	}
	double ret = xb*cos(phi1) - calcvar1*yb*sin(phi1);
	return(ret);
}

double ykfunc(double phi, double R, double phi1, double Rside)
{
	if (Rside == true)
	{
		calcvar1= -1;
	}
	if (Rside == false)
	{
		calcvar1= 1;
	}
	double ret = xb*sin(phi1) + calcvar1*yb*cos(phi1);
	return(ret);
}

void trajectory()
{
	double a = (1/2)*R;
	double xmax = R * (sqrt(pow(pow((Rmax/R),2), 1/3)-1));
	double ymax = -a * (pow(xmax,2));
	double Smax = (2*a*xmax*sqrt(1+4*pow(a,2)*pow(xmax,2))+log(abs(sqrt(1+4*pow(a,2)*pow(xmax,2))+2*a*xmax)))/(4*a);
	double phi0 = atan(2*a*xmax);
	double xkp = xmax * cos(phi0) - ymax * sin(phi0);
	double ykp = xmax * sin(phi0) + ymax * cos(phi0);
	RV0.x = findMinimum(0) * M_PI/180;
	RV0.z = findMinimum(1) * M_PI/180;

for (int i = 0; i < NofPoints; i++)
{
	trajectory0[i][0] = 6378000 * (input[i][0] * M_PI/180 - RV0.x);
	trajectory0[i][1] = Hpolet;
	trajectory0[i][2] = 6378000 * cos(input[i][0] * M_PI/180) * (input[i][1] * M_PI/180 - RV0.z);
	trajectory0[i][3] = input[i][2];
}

//*------------------------------------------------------------------------------*//

//*------------------------------------------------------------------------------*//
double xc0=x2 - x1;
double yc0=y2 - y_1;
double xc1=xc0*cos(phi1) + yc0*sin(phi1);
double yc1=abs(-xc0*sin(phi1) + yc0*cos(phi1));
double grE[70][2];
double maxXgrE = 0;
double maxJgrE = 0;
for (double j=0.05; j<=2*phi0; j=j+0.05)
{
	for (int i = 0; i <= 69; i++)
	{
		xo0=xo0func(j);
		yo0=yo0func(j);
		xo=xofunc(j, R);
		yo=yofunc(j, R);
		xb=xbfunc(j, R);
		yb=ybfunc(j, R);
		grE[i][0]=xb;
		grE[i][1]=yb;
		if (xb > maxXgrE)
		{
			maxXgrE = xb;
		}
		else
		{
			//max
		}
	}
}
//*------------------------------------------------------------------------------*//
}





void Vslet_Trajectory(double alpha0, double phi0, double x0, double y0, double z0)
{

}

POINT_TR Get_point_traj(int Npoint, char UpT)
{

}
