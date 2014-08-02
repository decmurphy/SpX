
#ifndef __MOTION_H__
#define __MOTION_H__

#include<math.h>

#include"initialise.h"
#include"landing.h"

inline void pitch_kick(int *x)
{
	gam[0] = M_PI/2 - 0.025;
	*x = 1;
}

inline void angles(int i)
{
/*
	beta is the angle through which gravity pulls the vehicle. alpha is the angle of attack relative to earth.
	The 'if' statements are just for trigonometry. If you draw a picture you'll see why they're necessary.
*/

	beta[i] = acos(s[i][0]/S[i]);
	if(s[i][1]<0)
		beta[i] = 2*M_PI - beta[i];

	alpha[i] = acos(vR[i][0]/VR[i]);
	if(vR[i][1]<0)
		alpha[i] = 2*M_PI - alpha[i];

}

inline void grav_turn(int i)
{
	gam[i] = alpha[i];
	if(_MECO1 && cos(beta[i]-alpha[i])<0 && !_SECO1)
		gam[i] = asin(-M[i]*g(S[i])*sin(beta[i]+M_PI)/Ft[i]);
/*
	The line above ensures that if the gravity turn makes the 2nd stage point down towards earth,
	(which should happen when the velocity is also tangential to earth)
	it will correct the angle and point such that it's upward thrust cancels out
	the downward force of gravity - theoretically keeping it at constant altitude
	until it reaches orbital velocity. Should never be used if gravity turn executed
	properly though, as it's super inefficient.
*/
/*
	The next few lines are OG2 course corrections
	Trial and error. Lots of corrections for ultra steep trajectory.
*/
	if(VR[1] > 2200)
		p[1] = 0.7;				// Throttle down to 70%
	if(VR[1] > 2900)
		gam[1] = beta[1] - M_PI/2;		// go horizontal rel. to earth
	if(VR[1] > 3400)
		gam[1] = beta[1] - M_PI/2 - 0.1;	// Start pointing down....
	if(VR[1] > 3900)
		gam[1] = beta[1] - M_PI/2 - 0.2;
	if(VR[1] > 4400)
		gam[1] = beta[1] - M_PI/2 - 0.3;
	if(VR[1] > 6200)
		gam[1] = beta[1] - M_PI/2 - 0.4;
}

inline void first_step()
{
	F[0][1] = 9*M1D.Th_sl - M[0]*g0;
	a[0][1] = F[0][1]/M[0];

	vA[0][0] = vE;				// Absolute velocity in x-direction = velocity of earth at surface
	vA[0][1] += a[0][1]*dt/2;

	S[0] = sqrt(s[0][0]*s[0][0] + s[0][1]*s[0][1]);
	A[0] = sqrt(a[0][0]*a[0][0] + a[0][1]*a[0][1]);
}

/*
	Leapfrog integrator for moving ma boi F9. Conservative which is nice. Euler can go poo a pineapple.
*/

void leapfrog_step(int i) // i = stage
{

	if(_MEI1) {
		dm = engines[i]*p[i]*236*dt;
		F9[i].Mf -= dm;
		M[i] -= dm;
	}

	if(_release) {
		q = 0.5*rho(S[i]-Re)*VR[i]*VR[i]*1e-3;			// Aerodynamic stress
		Fd = (0.5)*F9[i].Cd*F9[i].A*rho(S[i]-Re)*VR[i]*VR[i];	// Drag
		Ft[i] = engines[i]*p[i]*GetThrust(S[i], i);		// Thrust

		/* x-direction	*/
		F[i][0] = Ft[i]*cos(gam[i]) + Fd*cos(alpha[i]+M_PI) + M[i]*g(S[i])*cos(beta[i]+M_PI);
		s[i][0] += vA[i][0]*dt;
		a[i][0] = F[i][0]/M[i];
		vA[i][0] += a[i][0]*dt;
		vR[i][0] = vA[i][0] - vE*sin(beta[i]);
	
		/* y-direction	*/
		F[i][1] = Ft[i]*sin(gam[i]) + Fd*sin(alpha[i]+M_PI) + M[i]*g(S[i])*sin(beta[i]+M_PI);
		s[i][1] += vA[i][1]*dt;
		a[i][1] = F[i][1]/M[i];
		vA[i][1] += a[i][1]*dt;
		vR[i][1] = vA[i][1] - vE*cos(M_PI + beta[i]);

		S[i] = sqrt(s[i][0]*s[i][0] + s[i][1]*s[i][1]);
		VA[i] = sqrt(vA[i][0]*vA[i][0] + vA[i][1]*vA[i][1]);
		VR[i] = sqrt(vR[i][0]*vR[i][0] + vR[i][1]*vR[i][1]);
		A[i] = sqrt(a[i][0]*a[i][0] + a[i][1]*a[i][1]);
	}

	if(_release)	angles(i);
	if(t > 55) {
		if(i==1) 		grav_turn(i);
		if(i==0 && !_MECO1)	grav_turn(i);
	}
	if(_BBURN || _LBURN)
		flip(0);
	if(_LBURN && mod(t, 5) < dt)
		update_landing_throttle();
}

#endif

