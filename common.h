
#ifndef __COMMON_H__
#define __COMMON_H__

#define G 6.67384e-11		// Gravitational Const
#define Me 5.97219e24		// Mass of Earth
#define Re 6378137		// Radius of Earth
#define g0 9.7976		// Gravity acceleration on surface

typedef struct {

	double Cd, A;			// Coeff. of drag (I guessed this) and cross-sectional area
	double Mr[2], Mf[2], Mp;	// Mass_rocket, Mass_fuel, Mass_payload

} Rocket;

typedef struct {

	double Isp_sl, Isp_vac;		// Sea level/Vacuum Isp
	double Th_sl, Th_vac;		// Sea level/Vacuum Thrust

} Engine;

extern Rocket F9;
extern Engine M1D;
extern Engine M1Dv;

int engines;
double S, V, VA, VR, A, M;		// Distance, Velocity, Absolute V, Relative V, Acceleration, Mass
double p, q, Ft, Fd, *Fuel;		// throttle, areo pressure, thrust, drag, fuel pointer (to 1st or 2nd stage)
double dm, t = 0.0;			// rate of fuel consumption, time
double dt = 0.001;			// time step

				// inclination (rads) = 28.49*M_PI/180;
double vE = 407.6614278; 	// Earth velocity at Cape Canveral = 2*M_PI*Re*cos(incl)/(24*60*60);

double F[2] = {0, 0};		// x,y Forces on vehicle
double s[2] = {0, Re};		// x,y distance
double a[2] = {0, -g0};		// x,y acceleration
double v[2] = {0, 0};		// x,y absolute velocity
double vA[2] = {0, 0};		// x,y absolute velocity
double vR[2] = {0, 0};		// x,y relative velocity (rel. to Earth)

double alpha = 0, beta = M_PI/2, gam = M_PI/2;	// alpha = angle of velocity, beta = gravity, gamma = thrust

/////////////////////////////////////////////////

inline void output_telemetry(char* event, FILE *f2, int coriolis)
{

	double temp_s0 = s[0], temp_S = S-Re, temp_V = V;
	char *dist = "m", *vel = "m/s";

	if(coriolis) {
		temp_s0 -= vE*t;
		temp_V = VR;
	}

	if(temp_s0 > 1e3 || temp_S > 1e3) {
		temp_s0 *= 1e-3;
		temp_S *= 1e-3;
		dist = "km";
	}

	if(f2) fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, temp_V, A/g0, M, event);

	if(temp_V > 1e3) {
		temp_V *= 1e-3;
		vel = "km/s";
	}

	printf("T+%06.2f\t%16.16s\t%.2f%s x %.2f%s @ %.2f%s\n", t, event, temp_s0, dist, temp_S, dist, temp_V, vel);
}

inline void ignition(int stage, int *x, int num_engs)
{
	p = 1;					// p is throttle
	Fuel = stage==1 ? &(F9.Mf[0])
			  : &(F9.Mf[1]);

	*x = 1;
	engines = num_engs;

}

inline first_step_coriolis()
{
	M = F9.Mr[0] + F9.Mr[1] + F9.Mf[0] + F9.Mf[1] + F9.Mp;

	vA[0] = vE;				// Absolute velocity in x-direction = velocity of earth at surface

	F[1] = 9*M1D.Th_sl - M*g0;
	a[1] = F[1]/M;
	vA[1] += a[1]*dt/2;

	S = sqrt(s[0]*s[0] + s[1]*s[1]);
	A = sqrt(a[0]*a[0] + a[1]*a[1]);
}

inline first_step_no_coriolis()
{
	M = F9.Mr[0] + F9.Mr[1] + F9.Mf[0] + F9.Mf[1] + F9.Mp;

	F[1] = 9*M1D.Th_sl - M*g0;
	a[1] = F[1]/M;
	v[1] += a[1]*dt/2;

	S = sqrt(s[0]*s[0] + s[1]*s[1]);
	A = sqrt(a[0]*a[0] + a[1]*a[1]);
}

inline void pitch_kick(int *x)
{
	gam = M_PI/2 - 0.025;
	*x = 1;
}

inline void MSECO(int *x)
{
	p = 0;
	engines = 0;
	*x = 1;
}

inline void stage_sep(int stage_follow)
{
	M -= stage_follow==1 ?  (F9.Mr[1] + F9.Mf[1] + F9.Mp)
				: (F9.Mr[0] + F9.Mf[0]);
}

inline void flip(int x)
{
	gam = 	(x==1) ? beta + M_PI/2 :
		(x==2) ? alpha + M_PI :
		0;
}

/////////////////////////////////////////////////

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

inline double GetThrust(double H, int stage, int sync)
{
	return (stage==1 || !sync) ? Isp(H-Re)*236*g0	// 236 kg/s = M1D rate of fuel consumption
				     : M1Dv.Th_vac;
}

/*
	Leapfrog integrator used for moving ma boi F9. It's conservative which is nice. Fuck you Euler.
*/

void leapfrog_step_coriolis(int stage, int sync)
{

	Ft = engines*p*GetThrust(S, stage, sync);	// Thrust
	dm = engines*p*236*dt;
	*Fuel -= dm;
	M -= dm;

	q = 0.5*rho(S-Re)*VR*VR*1e-3;			// Aerodynamic stress
	Fd = (0.5)*F9.Cd*F9.A*rho(S-Re)*VR*VR;		// Drag

	/* x-direction	*/
	F[0] = Ft*cos(gam) + Fd*cos(alpha+M_PI) + M*g(S)*cos(beta+M_PI);
	s[0] += vA[0]*dt;
	a[0] = F[0]/M;
	vA[0] += a[0]*dt;
	vR[0] = vA[0] - vE*sin(beta);
	
	/* y-direction	*/
	F[1] = Ft*sin(gam) + Fd*sin(alpha+M_PI) + M*g(S)*sin(beta+M_PI);
	s[1] += vA[1]*dt;
	a[1] = F[1]/M;
	vA[1] += a[1]*dt;
	vR[1] = vA[1] - vE*cos(M_PI + beta);

	S = sqrt(s[0]*s[0] + s[1]*s[1]);
	VA = sqrt(vA[0]*vA[0] + vA[1]*vA[1]);
	VR = sqrt(vR[0]*vR[0] + vR[1]*vR[1]);
	A = sqrt(a[0]*a[0] + a[1]*a[1]);

	t += dt;
}
void leapfrog_step_no_coriolis(int stage, int sync)
{

	Ft = engines*p*GetThrust(S, stage, sync);
	dm = engines*p*236*dt;
	*Fuel -= dm;
	M -= dm;

	q = 0.5*rho(S-Re)*V*V*1e-3;
	Fd = (0.5)*F9.Cd*F9.A*rho(S-Re)*V*V;

	/* x-direction	*/
	F[0] = Ft*cos(gam) + Fd*cos(alpha+M_PI) + M*g(S)*cos(beta+M_PI);
	s[0] += v[0]*dt;
	a[0] = F[0]/M;
	v[0] += a[0]*dt;
	
	/* y-direction	*/
	F[1] = Ft*sin(gam) + Fd*sin(alpha+M_PI) + M*g(S)*sin(beta+M_PI);
	s[1] += v[1]*dt;
	a[1] = F[1]/M;
	v[1] += a[1]*dt;

	S = sqrt(s[0]*s[0] + s[1]*s[1]);
	V = sqrt(v[0]*v[0] + v[1]*v[1]);
	A = sqrt(a[0]*a[0] + a[1]*a[1]);

	t += dt;
}

/*
	This function tests a throttle value to find out, at a certain throttle, what
	height will the vehicle reache zero vertical velocity.
*/

double throttle_test(double hy, double ux, double uy, double mf, double throttle)
{

	double VEL, ft, fd, mass;
	double fx, fy, ax, ay;

	VEL = sqrt(ux*ux + uy*uy);

	ft = throttle*GetThrust(hy, 1, 1);

	do
	{
		mf -= throttle*236*dt;
		mass = mf + F9.Mr[0];						

		fd = (0.5)*F9.Cd*F9.A*rho(hy-Re)*VEL*VEL;

		flip(2);

		fx = ft*cos(gam) + fd*cos(alpha+M_PI) + mass*g(hy)*cos(beta+M_PI);
		ax = fx/mass;
		ux += ax*dt;
	
		fy = ft*sin(gam) + fd*sin(alpha+M_PI) + mass*g(hy)*sin(beta+M_PI);
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
	as we fall cause it changes with rounding errors etc. I think I re-calculate it every 5 seconds
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
	p = get_landing_throttle(S, v[0], v[1], F9.Mf[0]);
}

inline void angles_coriolis()
{
/*
	beta is the angle through which gravity pulls the vehicle. alpha is the angle of attack relative to earth.
	The 'if' statements are just for trigonometry. If you draw a picture you'll see why they're necessary.
*/

	beta = acos(s[0]/S);
	if(s[1]<0)
		beta = 2*M_PI - beta;

	alpha = acos(vR[0]/VR);
	if(vR[1]<0)
		alpha = 2*M_PI - alpha;

}

inline void angles_no_coriolis()
{

	beta = acos(s[0]/S);
	if(s[1]<0)
		beta = 2*M_PI - beta;

	alpha = acos(v[0]/V);
	if(v[1]<0)
		alpha = 2*M_PI - alpha;

}

inline void grav_turn_coriolis(int _M1, int _S1)
{
	gam = alpha;
	if(_M1 && cos(beta-alpha)<0 && !_S1)
		gam = asin(-M*g(S)*sin(beta+M_PI)/Ft);
/*
	The line above ensures that if the gravity turn makes the 2nd stage point down towards earth,
	(which should happen when the velocity is also tangential to earth)
	it will correct the angle and point such that it's upward thrust cancels out
	the downward force of gravity - theoretically keeping it at constant altitude
	until it reaches orbital velocity. Should never be used if gravity turn executed
	properly though, as it's super inefficient.
*/

	if(VR > 2200)
		p = 0.7;			// Throttle down to 70%
	if(VR > 3200)
		gam = beta - M_PI/2;		// go horizontal rel. to earth
	if(VR > 3500)
		gam = beta - M_PI/2 - 0.1;	// Start pointing down....
	if(VR > 3900)
		gam = beta - M_PI/2 - 0.2;
	if(VR > 4400)
		gam = beta - M_PI/2 - 0.3;
	if(VR > 5100)
		gam = beta - M_PI/2 - 0.4;

/*
	Trial and error. Lots of course corrections for OG2's ultra steep trajectory.
*/
}

inline void grav_turn_no_coriolis(int _M1, int _S1)
{
	gam = alpha;
	if(_M1 && cos(beta-alpha)<0 && !_S1)
		gam = asin(-M*g(S)*sin(beta+M_PI)/Ft);

	if(V > 2200)
		p = 0.7;
	if(V > 3200)
		gam = beta - M_PI/2;
	if(V > 3500)
		gam = beta - M_PI/2 - 0.1;
	if(V > 3900)
		gam = beta - M_PI/2 - 0.2;
	if(V > 4400)
		gam = beta - M_PI/2 - 0.3;
	if(V > 5100)
		gam = beta - M_PI/2 - 0.4;

}

double mod(double a, double b)
{
	return a < b ? a : mod(a-b, b);
}

inline void reset_variables(int *x, int *y, int*z)
{

	t = 0.0;
	dt = 0.001;

	s[0] = 0;
	s[1] = Re;
	v[0] = 0;
	v[1] = 0;

	alpha = 0;
	beta = M_PI/2;
	gam = M_PI/2;

	F9.Mf[0] = 390000;
	F9.Mf[1] = 72700;

	*x = 0;
	*y = 0;
	*z = 0;
}

#endif

