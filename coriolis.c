#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<mpi.h>
#include"coriolis.h"

inline void ME_ignition();
inline void pitch_kick();
inline void MECO();
inline void stage_sep();
inline void SE_ignition();
inline void SECO();
void leapfrog_step();
void angles();
void grav_turn();
double mod(double a, double b);

inline double g(double h);

Rocket F9	= {0.3, 10.52, {20000, 4900}, {390000, 75700}, 1200};
Engine M1D	= {282, 311, 650000, 720000};
Engine M1Dv	= {0, 345, 0, 801000};

int 	_pitch = 0,
	_ME1 = 0, _ME2 = 0, _ME3 = 0, _ME4 = 0,
	_MECO1 = 0, _MECO2 = 0, _MECO3 = 0, _MECO4 = 0,
	_LBURN = 0, _BBURN = 0,
	_SE1 = 0, _SECO = 0;

double dt = 0.001, p = 1.0;
int crash = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {

	MPI_Init(&argc, &argv);

	int rank, size;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &size);

	int old, new, orbit=0;
	double peri, apo;
	double MECO_t = -100, RETRO_t = -100;

	FILE *f, *f1;
	if(!rank) {
		f = fopen("Stage1_launch.dat", "w");
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
			ME_ignition(1);
			first_step();
			if(!rank) printf("T+%.2f\t\tLiftoff\n", t);
		}
		else if(fabs(t-7)<dt/2 && !_pitch) {
			if(!rank) printf("T+%.2f\t\tPitch Kick\n", t);
			pitch_kick();
		}
		else if(F9.Mf[0]<54000 && !_MECO1) {
			if(!rank) printf("T+%.2f\tMECO 1\t\t\t\t%.fkm x %.fkm @ Velocity %.1fkm/s\n",
				t, (s[0]-vE*t)*1e-3, (S-Re)*1e-3, VR*1e-3);
			fprintf(f1, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M, "MECO");
			MECO(1);
			MECO_t = t;
		}
		else if(fabs(t - MECO_t - 2) < dt/2) {
			if(!rank) printf("T+%.2f\tFirst Stage Separation\n", t);
			stage_sep(rank+1);
		}

/*************************************************************************************************/
/*	First Stage: Takeoff					*/
/*************************************************************************************************/

		if(!rank) {

			leapfrog_step(1);
			angles();
			if(t > 55 && !_MECO1)
				grav_turn();
			if(_MECO1)
				break;

			fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M, q);

		}

/*************************************************************************************************/
/*	Second Stage: Orbital burn/Drift			*/
/*	Second Stage: Attitude Correction when V>5km/sec	*/
/*************************************************************************************************/

		else {
			if(fabs(t - MECO_t - 4) < dt/2) {
				printf("T+%.2f\tSecond Engine Start\n", t);
				SE_ignition();
			}
			else if((F9.Mf[1]<10 || VA > sqrt(G*Me/S) || mod(360+(alpha-beta+M_PI/2)*180/M_PI, 360) < 0.01) && _SE1 && !_SECO) {
				printf("T+%.2f\tSECO 1\t\t\t\t%.2fkm x %.2fkm @ %.2fkm/s, %.2f deg --- (Fuel %.2fkg)\n", 
					t, (s[0]-vE*t)*1e-3, (S-Re)*1e-3, VA*1e-3, mod(360+(alpha-beta+M_PI/2)*180/M_PI, 360), F9.Mf[1]);
				fprintf(f1, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M, "SECO");
				SECO();
				apo = S-Re;
				peri = S-Re;
				dt = 0.1;
			}

			if(_SECO && S-Re < 1e5) {
				crash = 1;
				break;
			}

			if(_SECO) {
				if(S-Re > apo) apo = S-Re;
				if(S-Re < peri) peri = S-Re;
			}

			leapfrog_step(2);
			angles();
			if(t > 55) grav_turn();

			double x = gam-beta+M_PI/2;
			if(x > 2*M_PI)
				x -= 2*M_PI;

			fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, VR, A, M, x);

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
