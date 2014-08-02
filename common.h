
#ifndef __COMMON_H__
#define __COMMON_H__

#include<string.h>
#include<unistd.h>

#define G 6.67384e-11		// Gravitational Const
#define Me 5.97219e24		// Mass of Earth
#define Re 6378137		// Radius of Earth
#define g0 9.7976		// Gravity acceleration on surface

typedef struct {

	double Cd, A;			// Coeff. of drag (I guessed this) and cross-sectional area
	double Mr, Mf, Mp;		// Mass_rocket, Mass_fuel, Mass_payload

} Rocket;

typedef struct {

	double Isp_sl, Isp_vac;		// Sea level/Vacuum Isp
	double Th_sl, Th_vac;		// Sea level/Vacuum Thrust

} Engine;

typedef struct {

	int stage;
	double t;
	char name[128];

} Event;

extern Rocket F9[2];
extern Engine M1D;
extern Engine M1Dv;

extern int 	_release, _pitch,
		_MEI1, _MEI2, _MEI3,
		_MECO1, _MECO2, _MECO3,
		_LBURN, _BBURN,
		_SEI1, _SECO1;

int engines[2];
double S[2], VA[2], VR[2], A[2], M[2];	// Distance, Velocity, Absolute V, Relative V, Acceleration, Mass
double p[2], q, Ft[2], Fd;		// throttle, aero pressure, thrust, drag
double dm, t = -10.0;			// rate of fuel consumption, time
double dt = 0.001;			// time step

double vE = 0.0;		// inclination (rads) = 28.49*M_PI/180;

double F[2][2] = {{0}};		// x,y Forces on vehicle
double s[2][2] = {{0}};		// x,y distance
double a[2][2] = {{0}};		// x,y acceleration
double vA[2][2] = {{0}};	// x,y absolute velocity
double vR[2][2] = {{0}};	// x,y relative velocity (rel. to Earth)

double alpha[2] = {0}, beta[2] = {M_PI/2}, gam[2] = {M_PI/2};	// alpha = angle of velocity, beta = gravity, gamma = thrust

/////////////////////////////////////////////////

inline void init(Event **event, int *N, int argc, char **argv)
{
	s[0][1] = Re;
	S[0] = Re;
	M[0] = F9[0].Mr + F9[1].Mr + F9[0].Mf + F9[1].Mf + F9[1].Mp;
	M[1] = F9[0].Mr + F9[1].Mr + F9[0].Mf + F9[1].Mf + F9[1].Mp;

	int opt, line = -1;
	char str[128], inputfile[128];

	while( (opt=getopt(argc,argv,"f:s:")) != -1) {
		switch(opt) {
			case 'f': strncpy(inputfile, optarg, sizeof(inputfile)); break;
			case 's': vE = atoi(optarg)==0 ? 0 : 407.6614278; break; 	// Earth velocity at Cape Canveral
		}
	}

	FILE *in = fopen(inputfile, "r");
	while( fgets(str, sizeof(str), in) != NULL) {

		if(line == -1) {
			sscanf(str, "%d", N);
			*event = (Event*)calloc(*N, sizeof(Event));
		}
		else {
			sscanf(str, "%d\t%lf\t%s\n", &(*event)[line].stage, &(*event)[line].t, (*event)[line].name);
		}
		line++;
	}

	fclose(in);
}	

inline void output_telemetry(char* event, FILE *f2, int i)	// i = stage
{

	double temp_s0 = s[i][0] - vE*t, temp_S = S[i]-Re, temp_V = VR[i];
	char *dist = "m", *vel = "m/s";

	if(temp_s0 > 1e3 || temp_S > 1e3) {
		temp_s0 *= 1e-3;
		temp_S *= 1e-3;
		dist = "km";
	}

	if(f2) fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[i][0]*1e-3, (s[i][1]-Re)*1e-3, (S[i]-Re)*1e-3, temp_V, A[i]/g0, M[i], event);

	if(temp_V > 1e3) {
		temp_V *= 1e-3;
		vel = "km/s";
	}

	printf("T%+07.2f\t%16.16s\t%.2f%s x %.2f%s @ %.2f%s\n", t, event, temp_s0, dist, temp_S, dist, temp_V, vel);
}

inline void output_file(int i, FILE *f)
{
	fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[i][0]*1e-3, (s[i][1]-Re)*1e-3, (S[i]-Re)*1e-3, VR[i], A[i]/g0, M[i], p[i]);
}

inline void ignition(int i, int *x, int num_engs)
{
	p[i] = 1.0;					// p is throttle
	engines[i] = num_engs;
	*x = 1;
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

inline void pitch_kick(int *x)
{
	gam[0] = M_PI/2 - 0.025;
	*x = 1;
}

inline void MSECO(int i, int *x)
{
	p[i] = 0;
	engines[i] = 0;
	*x = 1;
}

inline void sync_stages()
{
	s[1][0] = s[0][0];
	s[1][1] = s[0][1];
	vA[1][0] = vA[0][0];
	vA[1][1] = vA[0][1];

	S[1] = S[0];
	VA[1] = VA[0];

	alpha[1] = alpha[0];
	beta[1] = beta[0];
	gam[1] = gam[0];
}

inline void stage_sep()
{
	int i;
	for(i=0;i<2;i++)
		M[i] = F9[i].Mr + F9[i].Mf + F9[i].Mp;
}

inline void flip(int i)
{
	gam[i] = alpha[i] + M_PI;	// retrograde
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

inline double GetThrust(double H, int i)
{
	return (i==0 || !_MECO1) ? Isp(H-Re)*236*g0	// 236 kg/s = M1D rate of fuel consumption
				     : M1Dv.Th_vac;
}

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

double mod(double a, double b)
{
	return a < b ? a : mod(a-b, b);
}

/*
	Leapfrog integrator used for moving ma boi F9. It's conservative which is nice. Fuck you Euler.
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

void execute(char *name, FILE *f)
{
	if(!strcmp(name, "MEI-1")) {
		ignition(0, &_MEI1, 9);
		output_telemetry(name, NULL, 0);
	}
	else if(!strcmp(name, "Liftoff")) {
		_release = 1;
		first_step();
		output_telemetry(name, NULL, 0);
	}
	else if(!strcmp(name, "Pitch_Kick")) {
		pitch_kick(&_pitch);
		output_telemetry(name, NULL, 0);
	}
	else if(!strcmp(name, "MECO-1")) {
		MSECO(0, &_MECO1);
		sync_stages();
		output_telemetry(name, f, 1);
	}
	else if(!strcmp(name, "Stage_Sep")) {
		stage_sep();
		output_telemetry(name, NULL, 0);
	}
	else if(!strcmp(name, "SEI-1")) {
		ignition(1, &_SEI1, 1);
		output_telemetry(name, NULL, 1);
	}
	else if(!strcmp(name, "MEI-2")) {
		ignition(0, &_MEI2, 3);
		_BBURN = 1;
		output_telemetry(name, f, 0);
	}
	else if(!strcmp(name, "MECO-2")) {
		MSECO(0, &_MECO2);
		_BBURN = 0;
		output_telemetry(name, f, 0);
	}
	else if(!strcmp(name, "SECO-1")) {
		MSECO(1, &_SECO1);
		output_telemetry(name, f, 1);
	}
	else if(!strcmp(name, "MEI-3")) {
		ignition(0, &_MEI3, 1);
		_LBURN = 1;
		output_telemetry(name, f, 0);
	}
}

#endif

