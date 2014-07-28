#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<mpi.h>
#include"default.h"

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

Rocket F9	= {0.3, 10.52, {20000, 4900}, {390000, 72700}, 1200};
Engine M1D	= {282, 311, 650000, 720000};
Engine M1Dv	= {0, 345, 0, 801000};

int 	_pitch = 0,
	_ME1 = 0, _ME2 = 0, _ME3 = 0, _ME4 = 0,
	_MECO1 = 0, _MECO2 = 0, _MECO3 = 0, _MECO4 = 0,
	_LBURN = 0, _BBURN = 0,
	_SE1 = 0, _SECO = 0,
	_p = 0;

double dt = 0.001, p = 1.0;
int crash = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {

	MPI_Init(&argc, &argv);

	int rank, size;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &size);

	double MECO_t = -100, RETRO_t = -100;

	FILE *f, *f1, *f2;
	if(!rank) {
		f = fopen("Stage1_launch.dat", "w");
		f1 = fopen("Stage1_retro.dat", "w");
		f2 = fopen("Stage1_Points.dat", "w");
	}
	else {
		f = fopen("Stage2.dat", "w");
		f2 = fopen("Stage2_Points.dat", "w");
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
		else if(F9.Mf[0]<45000 && !_MECO1) {
			if(!rank) {
				printf("T+%.2f\tMECO 1\t\t\t\t%.fkm x %.fkm @ Velocity %.1fkm/s\n", t, (s[0])*1e-3, (S-Re)*1e-3, V*1e-3);
				fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, "MECO");
			}
			MECO(1);
			MECO_t = t;
		}
		else if(fabs(t - MECO_t - 2) < dt/2) {
			if(!rank) printf("T+%.2f\tFirst Stage Separation\n", t);
			stage_sep(rank+1);
		}

/*************************************************************************************************/
/*	First Stage: Flip/Entry burn/Landing burn		*/
/*************************************************************************************************/

		if(!rank) {
			if(t>515 && !_ME4 && F9.Mf[0]>0) {
				printf("\nT+%.2f\tBraking Burn Start.\t\t%.2fkm x %.2fkm @ %.2fm/s\n", t, (s[0])*1e-3, (S-Re)*1e-3, V);
				fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, "EB");
				flip(2);
				ME_ignition(4);
				_BBURN = 1;
			}
			else if((t>552 && !_MECO4) || F9.Mf[0]<0) {
				printf("T+%.2f\tBraking Burn End.\t\t%.2fkm x %.2fkm @ %.2fm/s\n", t, (s[0])*1e-3, (S-Re)*1e-3, V);
				fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, "EBX");
				MECO(4);
			}				
			else if(t>628 && _MECO4 && !_ME3 && F9.Mf[0]>0) {
				printf("T+%.2f\tLanding Burn Start.\t\t%.2fkm x %.2fkm @ %.2fm/s\n", t, (s[0])*1e-3, (S-Re)*1e-3, V);
				fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, "LB");
				flip(2);
				ME_ignition(3);
				_LBURN = 1;
			}
			else if((F9.Mf[0] < 5 || (_LBURN && S-Re < 0.1)) && !_MECO3) {
				printf("T+%.2f\tLanding Burn End.\t\t%.2fkm x %04.2fkm @ %04.2fm/s --- (Fuel %.2fkg)\n",
					t, (s[0])*1e-3, (S-Re)*1e-3, V, F9.Mf[0]);
				fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, "LBX");
				MECO(3);
			}
			else if(S<Re) {
				printf("\n\t///////////////////////////////////////////////////////////////////////////////////\n");
				printf("\t//\tTouchdown:\n\t//\t\t\t\t\t%.2fkm x %.2fkm @ %.2fm/s --- (Fuel: %.2fkg)\n",
					s[0]*1e-3, (Re-S)*1e-3, V, F9.Mf[0]);
				printf("\t///////////////////////////////////////////////////////////////////////////////////\n");
				break;
			}

			leapfrog_step(1);
			angles();
			if(t > 55 && !_MECO1) 	grav_turn();
			if(_BBURN || _LBURN)	flip(2);

			if(!_MECO1)
				fprintf(f , "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, q);
			else
				fprintf(f1, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, q);

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
			else if((F9.Mf[1]<10 || V > 7800) && !_SECO) {
				printf("T+%.2f\tSECO 1\t\t\t\t%.2fkm x %.2fkm @ %.2fkm/s, %.2f deg --- (Fuel %.2fkg)\n", 
					t, (s[0])*1e-3, (S-Re)*1e-3, V*1e-3, (alpha-beta+M_PI/2)*180/M_PI, F9.Mf[1]);
				fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, "SECO");
				SECO();
				break;
			}

			leapfrog_step(2);
			angles();
			if(t > 55) grav_turn();

			fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M);

/*************************************************************************************************/

		}

	}
	while(1);

	MPI_Barrier(MPI_COMM_WORLD);

	if(!rank) fclose(f1);
	fclose(f);
	fclose(f2);

	MPI_Finalize();
	return 0;
}
