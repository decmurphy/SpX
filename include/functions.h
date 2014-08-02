
#ifndef __FUNCTIONS_H__
#define __FUNCTIONS_H__

#include<math.h>

#include"initialise.h"

double mod(double a, double b)
{
	return a < b ? a : mod(a-b, b);
}

inline double g(double h)
{
	return G*Me/(h*h);
}

/*
	These next two functions for atmospheric density and pressure were obtained from a load of data points
	I found on some NASA website, to which I fitted functions using some other awesome plot-fitting website.
*/

inline double rho(double h)
{
	return 1.21147*exp(h*-1.12727e-4);
}

inline double P(double h)
{
	return -517.18*log(0.012833*log(6.0985e28*h + 2.0981e28));
}

/*
	Interpolating Isp at given altitude using sea-level/vacuum values, and the current atmospheric pressure.
*/

inline double Isp(double h)
{
	return 	(h<80000) ? M1D.Isp_sl + (1.0/P(0))*(P(0)-P(h*1e-3))*(M1D.Isp_vac - M1D.Isp_sl)
			    : M1D.Isp_vac;
}

inline double GetThrust(double H, int i)
{
	return (i==0 || !_MECO1) ? Isp(H-Re)*236*g0	// 236 kg/s = M1D rate of fuel consumption
				     : M1Dv.Th_vac;
}

inline void flip(int i)
{
	gam[i] = alpha[i] + M_PI;	// retrograde
}

#endif

