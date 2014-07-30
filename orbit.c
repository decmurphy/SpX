#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include"common.h"

Rocket F9	= {0.3, 10.52, {20000, 4900}, {390000, 75700}, 1200};
Engine M1D	= {282, 311, 650000, 720000};
Engine M1Dv	= {0, 345, 0, 801000};

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {

	int old, new, orbit=0, crash=0;
	double peri, apo;
	double MECO_t = -100;

	int 	_pitch = 0,
		_ME1 = 0, _MECO1 = 0,
		_SE1 = 0, _SECO1 = 0;

	FILE *f, *f1, *f2, *f3;
	f = fopen("Stage1.dat", "w");
	f1 = fopen("Stage1_Points.dat", "w");
	f2 = fopen("Stage2.dat", "w");
	f3 = fopen("Stage2_Points.dat", "w");

	do{

/*************************************************************************************************/
/*	First Stage: Takeoff					*/
/*************************************************************************************************/

		if(t==0) {
			ignition(1, &_ME1, 9);
			first_step_coriolis();
			output_telemetry("Liftoff", NULL, 1);
		}
		else if(fabs(t-7)<dt/2 && !_pitch) {
			output_telemetry("Pitch Kick", NULL, 1);
			pitch_kick(&_pitch);
		}
		else if(F9.Mf[0]<45000 && !_MECO1) {
			output_telemetry("MECO", f1, 1);
			MSECO(&_MECO1);
			MECO_t = t;
		}

/*************************************************************************************************/
/*	Second Stage: Orbital burn/Drift			*/
/*************************************************************************************************/

		else if(fabs(t - MECO_t - 2) < dt/2) {
			output_telemetry("Stage Sep", NULL, 1);
			stage_sep(2);
		}
		else if(fabs(t - MECO_t - 4) < dt/2) {
			output_telemetry("M1Dv Ignition", NULL, 1);
			ignition(2, &_SE1, 1);
		}
		else if((F9.Mf[1]<10 || VA > sqrt(G*Me/S)) && _SE1 && !_SECO1) {
			output_telemetry("SECO", f3, 1);
			MSECO(&_SECO1);
			apo = S-Re;
			peri = S-Re;
			dt = 0.1;
		}
			if(_SECO1 && S-Re < 1e5) {
			crash = 1;
			break;
		}
 
		if(_SECO1) {
			if(S-Re > apo) apo = S-Re;
			if(S-Re < peri) peri = S-Re;
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

	fclose(f);
	fclose(f1);
	fclose(f2);
	fclose(f3);

	return 0;
}
