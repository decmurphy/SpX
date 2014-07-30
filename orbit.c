#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include"common.h"

Rocket F9	= {0.3, 10.52, {20000, 4900}, {390000, 75700}, 1200};
Engine M1D	= {282, 311, 650000, 720000};
Engine M1Dv	= {0, 345, 0, 801000};

int 	_pitch = 0,
	_ME1 = 0, _ME2 = 0, _ME3 = 0,
	_MECO1 = 0, _MECO2 = 0, _MECO3 = 0,
	_LBURN = 0, _BBURN = 0,
	_SE1 = 0, _SECO1 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {

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

/************************************************************/

	int old, new, orbit=0, crash=0;
	double peri, apo;

	FILE *f, *f1, *f2;
	f = fopen("Stage1.dat", "w");
	f1 = fopen("Points.dat", "w");
	f2 = fopen("Stage2.dat", "w");

	do{

/*************************************************************************************************/
/*	First Stage: Takeoff					*/
/*************************************************************************************************/

		for(i=0;i<N;i++) {
			if(fabs(t-event[i].t) < dt/2) {				// If an event in profile.txt occurs at
				execute(event[i].name, 2, f1, 1);		// this time, execute the event
			}
		}

		if((F9.Mf[1]<10 || VA > sqrt(G*Me/S)) && _SE1 && !_SECO1) {
			output_telemetry("SECO", f1, 1);
			MSECO(&_SECO1);
			apo = S-Re;
			peri = S-Re;
			dt = 0.1;
		}

		if(_SECO1) {
			if(S-Re > apo)	apo = S-Re;
			if(S-Re < peri) peri = S-Re;
			if(S-Re < 1e5) {
				crash = 1;
				break;
			}
		}

		leapfrog_step_coriolis(2, _MECO1);
		angles_coriolis();
		if(t > 55)
			grav_turn_coriolis(_MECO1, _SECO1);

		if(!_MECO1)
			fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M, q);
		else
			fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M);

/*************************************************************************************************/

		if(s[0]>0)	new = 1;
		else		new = 2;

		if(old==2 && new==1)
			orbit++;
		old = new;

/*************************************************************************************************/

	}
	while(orbit<2);

	if(crash) printf("\nT+%.0f\t\t\tCrash\t\t%.2fkm x %.2fkm\n", t, peri*1e-3, apo*1e-3);
	else printf("\nT+%.0f\t\t\tOrbit\t\t%.2fkm x %.2fkm\n", t, peri*1e-3, apo*1e-3);

	free(event);

	fclose(f);
	fclose(f1);
	fclose(f2);
	fclose(in);

	return 0;
}
