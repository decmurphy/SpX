
#ifndef __INIT_H__
#define __INIT_H__

#include<string.h>
#include<unistd.h>
#include<math.h>

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

	strncpy(inputfile, "profile.txt", sizeof(inputfile));	// Default flight profile

	while( (opt=getopt(argc,argv,"f:c:")) != -1) {
		switch(opt) {
			case 'f': strncpy(inputfile, optarg, sizeof(inputfile)); break;
			case 'c': vE = atoi(optarg)==0 ? 0 : 407.6614278; break; 		// Earth velocity at Cape Canveral
			case '?': printf("Usage: %s -c [coriolis?] -f [flight-profile]\n", argv[0]); exit(1);
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

#endif

