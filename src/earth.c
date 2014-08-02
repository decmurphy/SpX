#include<stdio.h>
#include<math.h>

int main() {

	FILE *f;
	f = fopen("earth.dat", "w");
	
	double i = 0.0;
	double x, y, X, Y;
	int R = 6378137;

	while(i<2*M_PI) {

		x = R*cos(i);
		y = R*sin(i);

		X = (R+100000)*cos(i);
		Y = (R+100000)*sin(i);

		fprintf(f, "%f\t%f\t%f\t%f\n", x*1e-3, (y-R)*1e-3, X*1e-3, (Y-R)*1e-3);
		i += 0.001;

	}

	fclose(f);

	return 0;
}
