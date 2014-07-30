#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<mpi.h>
#include"common.h"

Rocket F9	= {0.3, 10.52, {20000, 4900}, {390000, 75700}, 1200};
Engine M1D	= {282, 311, 650000, 720000};
Engine M1Dv	= {0, 345, 0, 801000};

double dt = 0.001;

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {

	MPI_Init(&argc, &argv);

	int rank, size;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &size);

	int old, new, orbit=0, crash=0;
	double peri, apo;
	double MECO_t = -100;

	int 	_pitch = 0,
		_ME1 = 0, _MECO1 = 0,
		_SE1 = 0, _SECO1 = 0;

	FILE *f, *f1;
	if(!rank) {
		f = fopen("Stage1.dat", "w");
		f1 = fopen("Stage1_Points.dat", "w");
	}
	else {
		f = fopen("Stage2.dat", "w");
		f1 = fopen("Stage2_Points.dat", "w");
	}

/*************************************************************************************************/
/*	Launch/Pitch Kick/Gravity turn		*/
/*************************************************************************************************/

	do{
		if(t==0) {
			ignition(1, &_ME1, 9);
			first_step_coriolis();
			if(!rank) printf("T+%.2f\t\tLiftoff\n", t);
		}
		else if(fabs(t-7)<dt/2 && !_pitch) {
			if(!rank) printf("T+%.2f\t\tPitch Kick\n", t);
			pitch_kick(&_pitch);
		}
		else if(F9.Mf[0]<45000 && !_MECO1) {
			if(!rank) printf("T+%.2f\tMECO 1\t\t\t\t%.fkm x %.fkm @ Velocity %.1fkm/s\n",
				t, (s[0]-vE*t)*1e-3, (S-Re)*1e-3, VR*1e-3);
			fprintf(f1, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M, "MECO");
			MSECO(&_MECO1);
			MECO_t = t;
		}

/*************************************************************************************************/
/*	First Stage: Takeoff					*/
/*************************************************************************************************/

		if(!rank) {

			leapfrog_step_coriolis(1, _MECO1);
			angles_coriolis();
			if(t > 55)
				grav_turn_coriolis(_MECO1, _SECO1);
			if(_MECO1) {
				MPI_Barrier(MPI_COMM_WORLD);
				break;
			}

			fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M, q);

		}

/*************************************************************************************************/
/*	Second Stage: Orbital burn/Drift			*/
/*	Second Stage: Attitude Correction when V>5km/sec	*/
/*************************************************************************************************/

		else {
			if(fabs(t - MECO_t - 2) < dt/2) {
				MPI_Barrier(MPI_COMM_WORLD);
				printf("T+%.2f\tFirst Stage Separation\n", t);
				stage_sep(rank+1);
			}
			else if(fabs(t - MECO_t - 4) < dt/2) {
				printf("T+%.2f\tSecond Engine Start\n", t);
				ignition(2, &_SE1, 1);
			}
			else if((F9.Mf[1]<10 || VA > sqrt(G*Me/S) || mod(360+(alpha-beta+M_PI/2)*180/M_PI, 360) < 0.01) && _SE1 && !_SECO1) {
				printf("T+%.2f\tSECO 1\t\t\t\t%.2fkm x %.2fkm @ %.2fkm/s, %.2f deg --- (Fuel %.2fkg)\n", 
					t, (s[0]-vE*t)*1e-3, (S-Re)*1e-3, VA*1e-3, mod(360+(alpha-beta+M_PI/2)*180/M_PI, 360), F9.Mf[1]);
				fprintf(f1, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M, "SECO");
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

			fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M);

/*************************************************************************************************/

			if(s[0]>0)	new = 1;
			else		new = 2;

			if(old==2 && new==1)
				orbit++;
			old = new;
		}

/*************************************************************************************************/

	}
	while(orbit<2);

	MPI_Barrier(MPI_COMM_WORLD);

	if(rank && crash) printf("\nT+%.2f\tCrash\t\t\t\t%.2fkm x %.2fkm\n", t, peri*1e-3, apo*1e-3);
	else if(rank) printf("\nT+%.2f\tOrbit\t\t\t\t%.2fkm x %.2fkm\n", t, peri*1e-3, apo*1e-3);

	fclose(f);
	fclose(f1);

	MPI_Finalize();
	return 0;
}
