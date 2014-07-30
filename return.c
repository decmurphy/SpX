#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include"common.h"

Rocket F9	= {0.3, 10.52, {20000, 4900}, {390000, 72700}, 1200};
Engine M1D	= {282, 311, 650000, 720000};
Engine M1Dv	= {0, 345, 0, 801000};

int 	_pitch = 0,
	_ME1 = 0, _ME2 = 0, _ME3 = 0,
	_MECO1 = 0, _MECO2 = 0, _MECO3 = 0,
	_LBURN = 0, _BBURN = 0,
	_SE1 = 0, _SECO1 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {

	int crash = 0;

/*************************************************************************************************/

	Event *event;
	int i, N, line = -1;
	char str[128];

	FILE *in = fopen("profile.txt", "r");
	while( fgets(str, sizeof(str), in) != NULL) {

		if(line == -1) {
			sscanf(str, "%d", &N);
			event = (Event*)calloc(N, sizeof(Event));
		}
		else {
			sscanf(str, "%lf\t%s\n", &event[line].t, event[line].name);
		}
		line++;
	}

/*************************************************************************************************/
/*	Launch/Pitch Kick/Gravity turn				*/
/*	First Stage: Flip/Entry burn/Landing burn		*/
/*************************************************************************************************/

	FILE *f, *f1, *f2;
	f = fopen("Stage1.dat", "w");
	f1 = fopen("Points.dat", "w");
	f2 = fopen("Stage2.dat", "w");

	do{

		for(i=0;i<N;i++)
			if(fabs(t-event[i].t) < dt/2)				// If an event in profile.txt occurs at
				execute(event[i].name, 1, f1, 0);		// this time, execute the event

		if((F9.Mf[0] < 5 || (_LBURN && S-Re < 0.1)) && !_MECO3) {	// If Alt = 0.1m or Fuel runs out
			output_telemetry("MECO3", f1, 0);
			MSECO(&_MECO3);
			_LBURN = 0;
		}
		else if(S<Re) {							// If Alt = 0.0m
			output_telemetry("Touchdown", NULL, 0);
			break;
		}

	/**********************************************************************/

		leapfrog_step_no_coriolis(1, _MECO1);
		angles_no_coriolis();
		if(t > 55 && !_MECO1)
			grav_turn_no_coriolis(_MECO1, _SECO1);

		if(_BBURN || _LBURN)
			flip(2);						// flip(2) keeps rocket pointed retrograde during burns
		if(_LBURN && mod(t, 5) < dt)
			update_landing_throttle();				// update landing throttle every 5s


		fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, q);

	}
	while(1);
	printf("\n");

/*************************************************************************************************/
/*	Second Stage: Orbital burn/Drift			*/
/*************************************************************************************************/

	reset_variables(&_pitch, &_ME1, &_MECO1);

	do{

		for(i=0;i<N;i++)
			if(fabs(t-event[i].t) < dt/2)
				execute(event[i].name, 2, f1, 0);

		if((F9.Mf[1]<10 || V > sqrt(G*Me/S)) && !_SECO1) {		// If Fuel runs out, or reach orbital velocity
			output_telemetry("SECO", f1, 0);
			MSECO(&_SECO1);
			break;
		}

	/**********************************************************************/

		leapfrog_step_no_coriolis(2, _MECO1);
		angles_no_coriolis();
		if(t > 55)
			grav_turn_no_coriolis(_MECO1, _SECO1);

		fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M);

	}
	while(1);
	printf("\n");

	fclose(f);
	fclose(f1);
	fclose(f2);
	fclose(in);

	return 0;
}
