
#ifndef __IO_H__
#define __IO_H__

#include"initialise.h"

inline void output_telemetry(char* event, FILE *f2, int i)	// i = stage
{

	double temp_s0 = s[i][0] - vE*t, temp_S = S[i]-Re, temp_V = VR[i];
	char *dist = "m", *vel = "m/s";

	if(temp_s0 > 1e3 || temp_S > 1e3) {
		temp_s0 *= 1e-3;
		temp_S *= 1e-3;
		dist = "km";
	}

	if(f2) fprintf(f2, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%s\n", t, s[i][0]*1e-3, (s[i][1]-Re)*1e-3, (S[i]-Re)*1e-3, temp_V, A[i]/g0, M[i], event);

	if(temp_V > 1e3) {
		temp_V *= 1e-3;
		vel = "km/s";
	}

	printf("T%+07.2f\t%16.16s\t%.2f%s x %.2f%s @ %.2f%s\n", t, event, temp_s0, dist, temp_S, dist, temp_V, vel);
}

inline void output_file(int i, FILE *f)
{
	fprintf(f, "%g\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", t, s[i][0]*1e-3, (s[i][1]-Re)*1e-3, (S[i]-Re)*1e-3, VR[i], A[i]/g0, M[i], p[i]);
}

#endif

