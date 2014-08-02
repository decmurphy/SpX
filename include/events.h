
#ifndef __EVENTS_H__
#define __EVENTS_H__

#include"initialise.h"
#include"motion.h"
#include"io.h"

inline void ignition(int i, int *x, int num_engs)
{
	p[i] = 1.0;					// p is throttle
	engines[i] = num_engs;
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

