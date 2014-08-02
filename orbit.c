#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include"common.h"

Rocket F9[2]	= {{0.3, 10.52, 20000, 390000, 0}, {0.3, 10.52, 4900, 75700, 1200}};
Engine M1D	= {282, 311, 650000, 720000};
Engine M1Dv	= {0, 345, 0, 801000};

int 	_release = 0, _pitch = 0,
	_ME1 = 0, _ME2 = 0, _ME3 = 0,
	_MECO1 = 0, _MECO2 = 0, _MECO3 = 0,
	_LBURN = 0, _BBURN = 0,
	_SE1 = 0, _SECO1 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {

	double oldS = 0, newS = 0, apo = 0, per = 0;
	int orbit = 0;

	int i, N;
	Event *event;

	init(&event, &N, argc, argv);

/*************************************************************************************************/
/*	Launch/Pitch Kick/ Gravity Turn				*/
/*	First Stage: Takeoff					*/
/*************************************************************************************************/

	FILE *f, *f1, *f2;
	f = fopen("Stage1.dat", "w");
	f1 = fopen("Points.dat", "w");
	f2 = fopen("Stage2.dat", "w");

	do{

		/*	Execute events		*/
		for(i=0;i<N;i++) {
			if(event[i].stage == 0 && fabs(t-event[i].t) < dt/2 && !_MECO1)	// If an event in profile.txt occurs at
				execute(event[i].name, f1, 1);				// this time, execute the event
			if(event[i].stage == 1 && fabs(t-event[i].t) < dt/2)
				execute(event[i].name, f1, 1);
		}

		/*	SECO1			*/
		if(!_SECO1 && VA[1] > sqrt(G*Me/S[1])) {
			output_telemetry("SECO", f1, 1, 1);
			printf("\t\t\t\t\t@ %g degrees\n", (-3*M_PI/2 + alpha[1] - beta[1])*180/M_PI);
			MSECO(1, &_SECO1);
			apo = S[1];
			per = S[1];
			dt = 0.1;
		}

		/*	Orbit			*/
		else if(_SECO1 && oldS<0 && newS>0) {
			orbit = 1;
		}

		if(!_MECO1) {
			leapfrog_step_coriolis(0, _MECO1);
			output_file(0, f, 1);
		}

		if(_MECO1 && !orbit) {
			leapfrog_step_coriolis(1, _MECO1);
			output_file(1, f2, 1);

			oldS = newS;
			newS = s[1][0];
			if(_SECO1) {
				apo = S[1] > apo ? S[1] : apo;
				per = S[1] < per ? S[1] : per;
			}
		}

		t += dt;

	}
	while(!orbit);

	printf("\nT%+07.2f\t%16.16s\t%.2f%s x %.2f%s\n", t, "Orbit", (per-Re)*1e-3, "km", (apo-Re)*1e-3, "km");

	free(event);

	fclose(f);
	fclose(f1);
	fclose(f2);

	return 0;
}
