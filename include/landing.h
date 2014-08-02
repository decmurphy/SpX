
#ifndef __LANDING_H__
#define __LANDING_H__

#include<math.h>

#include"initialise.h"
#include"functions.h"
#include"motion.h"

/*
	This function tests a throttle value to find out, at a certain throttle, what
	height will the vehicle reach zero vertical velocity.
*/

double throttle_test(double hy, double ux, double uy, double mf, double throttle)
{

	double VEL, ft, fd, mass;
	double fx, fy, ax, ay;

	VEL = sqrt(ux*ux + uy*uy);

	ft = throttle*GetThrust(hy, 0);

	do
	{
		mf -= throttle*236*dt;
		mass = mf + F9[0].Mr;						

		fd = (0.5)*F9[0].Cd*F9[0].A*rho(hy-Re)*VEL*VEL;

		flip(0);

		fx = ft*cos(gam[0]) + fd*cos(alpha[0]+M_PI) + mass*g(hy)*cos(beta[0]+M_PI);
		ax = fx/mass;
		ux += ax*dt;
	
		fy = ft*sin(gam[0]) + fd*sin(alpha[0]+M_PI) + mass*g(hy)*sin(beta[0]+M_PI);
		hy += uy*dt;
		ay = fy/mass;
		uy += ay*dt;

		VEL = sqrt(ux*ux + uy*uy);
	}
	while(uy<0);

	return hy-Re;

}

/*
	This guy calls the above function to get as close to a hoverslam as possible. Need to keep calculating this
	as we fall cause it changes with rounding errors etc. I re-calculate it every 5 seconds, gets me a pretty good result
*/

inline double get_landing_throttle(double H, double ux, double uy, double mf)
{

	double a = 0.7, b = 1.0;
	double end_H;

	if( throttle_test(H, ux, uy, mf, b) > 0 )			// Will a full-power burn keep you alive?
	{
		if(throttle_test(H, ux, uy, mf, a) < 0 )			// Yes. Will a minimum-power burn kill you?
		{
			do
			{
				end_H = throttle_test(H, ux, uy, mf, (a+b)/2.0);
				if(fabs(end_H) < 0.1)
					return (a+b)/2.0;				// Yes. Burn at this throttle from now to do hoverslam.
				a = end_H < 0 ? (a+b)/2.0 : a;
				b = end_H > 0 ? (a+b)/2.0 : b;
			}
			while(1);
		}
		else return 0.0;						// No. Don't start burn yet. 
	}
	else return 1.0;						// No. Too late. Crash unavoidable. Should have started earlier

}

inline void update_landing_throttle()
{
	p[0] = get_landing_throttle(S[0], vR[0][0], vR[0][1], F9[0].Mf);
}

#endif

