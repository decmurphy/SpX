#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<mpi.h>
#include"common.h"

Rocket F9	= {0.3, 10.52, {20000, 4900}, {390000, 72700}, 1200};
Engine M1D	= {282, 311, 650000, 720000};
Engine M1Dv	= {0, 345, 0, 801000};

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {

	MPI_Init(&argc, &argv);

	int rank, size;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &size);

	double MECO_t = -100;
	int crash = 0, engines;
	int 	_pitch = 0,
		_ME1 = 0, _ME2 = 0, _ME3 = 0, _ME4 = 0,
		_MECO1 = 0, _MECO2 = 0, _MECO3 = 0, _MECO4 = 0,
		_LBURN = 0, _BBURN = 0,
		_SE1 = 0, _SECO1 = 0;

	FILE *f, *f2;
	if(!rank) {
		f = fopen("Stage1.dat", "w");
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
		if(t==0) {							//	T+0s
			ignition(1, &_ME1, 9);
			first_step_no_coriolis();
			if(!rank) output_telemetry("Liftoff", NULL, 0);
		}
		else if(fabs(t-7)<dt/2 && !_pitch) {				//	T+7s
			if(!rank) output_telemetry("Pitch Kick", NULL, 0);
			pitch_kick(&_pitch);
		}
		else if(F9.Mf[0]<45000 && !_MECO1) {				//	When 45000kg of fuel left
			if(!rank) output_telemetry("MECO", f2, 0);
			MSECO(&_MECO1);
			MECO_t = t;
		}
		else if(fabs(t - MECO_t - 2) < dt/2) {				//	2s after MECO
			if(!rank) output_telemetry("Stage Sep", NULL, 0);
			stage_sep(rank+1);
			MPI_Barrier(MPI_COMM_WORLD);
		}

/*************************************************************************************************/
/*	First Stage: Flip/Entry burn/Landing burn		*/
/*************************************************************************************************/

		if(!rank) {
			if(t>515 && !_ME4 && F9.Mf[0]>0) {			//	T+515s, if Fuel exists
				output_telemetry("Brake_Burn...", f2, 0);
				flip(2);
				ignition(1, &_ME4, 3);
				_BBURN = 1;
			}
			else if((t>552 && !_MECO4) || F9.Mf[0]<0) {		//	T+552s or if Fuel runs out
				output_telemetry("Brake_Burn_X", f2, 0);
				MSECO(&_MECO4);
			}				
			else if(t>628 && _MECO4 && !_ME3 && F9.Mf[0]>0) {	//	T+628s, if Fuel exists
				output_telemetry("Landing_Burn...", f2, 0);
				flip(2);
				ignition(1, &_ME3, 1);
				_LBURN = 1;
			}
			else if((F9.Mf[0] < 5 || (_LBURN && S-Re < 0.1)) && !_MECO3) {	//	If Alt = 0.1m or Fuel runs out
				output_telemetry("Landing_Burn_X", f2, 0);
				MSECO(&_MECO3);
			}
			else if(S<Re) {						// If Alt = 0.0m
				output_telemetry("Touchdown", NULL, 0);
				break;
			}

			leapfrog_step_no_coriolis(1, _MECO1);
			angles_no_coriolis();
			if(t > 55 && !_MECO1)
				grav_turn_no_coriolis(_MECO1, _SECO1);
			if(_BBURN || _LBURN)
				flip(2);					// flip(2) keeps rocket pointed retrograde during burns
			if(_LBURN && mod(t, 5) < dt)
				update_landing_throttle();			// update landing throttle every 5s

			fprintf(f , "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M, q);

		}

/*************************************************************************************************/
/*	Second Stage: Orbital burn/Drift			*/
/*	Second Stage: Attitude Correction when V>5km/sec	*/
/*************************************************************************************************/

		else {
			if(fabs(t - MECO_t - 4) < dt/2) {				//	4s after MECO
				output_telemetry("M1Dv Ignition", NULL, 0);
				ignition(2, &_SE1, 1);
			}
			else if((F9.Mf[1]<10 || V > sqrt(G*Me/S)) && !_SECO1) {		// If Fuel runs out, or reach orbital velocity
				output_telemetry("SECO", f2, 0);
				MSECO(&_SECO1);
				break;
			}

			leapfrog_step_no_coriolis(2, _MECO1);
			angles_no_coriolis();
			if(t > 55)
				grav_turn_no_coriolis(_MECO1, _SECO1);

			fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[0]*1e-3, (s[1]-Re)*1e-3, (S-Re)*1e-3, V, A/g0, M);

/*************************************************************************************************/

		}
	}
	while(1);

	MPI_Barrier(MPI_COMM_WORLD);

	fclose(f);
	fclose(f2);

	MPI_Finalize();
	return 0;
}
