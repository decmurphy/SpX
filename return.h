
#ifndef __2DLAUNCH_H__
#define __2DLAUNCH_H__

#define G 6.67384e-11
#define Me 5.97219e24
#define Re 6378137

#define g0 9.7976

typedef struct {

	double Cd, A;
	double Mr[2], Mf[2], Mp;

} Rocket;

typedef struct {

	double Isp_sl, Isp_vac;
	double Th_sl, Th_vac;

} Engine;

extern Rocket F9;
extern Engine M1D;
extern Engine M1Dv;

int engines;
double S, V, A, M;
double p, q, Ft, Fd, *Fuel;
double dm, t = 0.0;
extern double dt;

double F[2] = {0, 0};
double s[2] = {0, Re};
double a[2] = {0, -g0};
double v[2] = {0, 0};

double alpha = 0, beta = M_PI/2, gam = M_PI/2;

/////////////////////////////////////////////////

inline void ignition(int stage, int *x, int num_eng)
{
	p = 1;
	engines = num_eng;

	*x = 1;
	Fuel = 	(stage==1) ? &(F9.Mf[0]) :
		(stage==2) ? &(F9.Mf[1]) :
		0;

}

inline first_step()
{
	M = F9.Mr[0] + F9.Mr[1] + F9.Mf[0] + F9.Mf[1] + F9.Mp;

	F[1] = 9*M1D.Th_sl - M*g0;
	a[1] = F[1]/M;
	v[1] += a[1]*dt/2;

	S = sqrt(s[0]*s[0] + s[1]*s[1]);
	A = sqrt(a[0]*a[0] + a[1]*a[1]);
}

inline void pitch_kick(int *x)
{
	gam = M_PI/2 - 0.025;
	*x = 1;
}

inline void MSECO(int *x)
{
	p = 0;
	engines = 0;
	*x = 1;
}

inline void stage_sep(int x)
{
	M -= 	(x==1) ? (F9.Mr[1] + F9.Mf[1] + F9.Mp) :
		(x==2) ? (F9.Mr[0] + F9.Mf[0]) :
		0;
}

inline void flip(int x)
{
	gam = 	(x==1) ? beta + M_PI/2 :
		(x==2) ? alpha + M_PI :
		0;
}

/////////////////////////////////////////////////

double mod(double a, double b)
{
	return a < b ? a : mod(a-b, b);
}

inline double g(double h)
{
	return G*Me/(h*h);
}

inline double rho(double h)
{
	return 1.21147*exp(h*-1.12727e-4);
}

inline double P(double h)
{
	return -517.18*log(0.012833*log(6.0985e28*h + 2.0981e28));
}

inline double Isp(double h)
{
	return 	(h<80000) ? M1D.Isp_sl + (1.0/P(0))*(P(0)-P(h*1e-3))*(M1D.Isp_vac - M1D.Isp_sl) :
		M1D.Isp_vac;
}

inline double GetThrust(double H, int stage, int sync)
{
	if(stage==1 || !sync)
		return Isp(H-Re)*236*g0;
	else if(stage==2)
		return M1Dv.Th_vac;
}

void leapfrog_step(int stage, int sync)
{

	Ft = engines*p*GetThrust(S, stage, sync);
	dm = engines*p*236*dt;
	*Fuel -= dm;
	M -= dm;

	q = 0.5*rho(S-Re)*V*V*1e-3;
	Fd = (0.5)*F9.Cd*F9.A*rho(S-Re)*V*V;

	/* x-direction	*/
	F[0] = Ft*cos(gam) + Fd*cos(alpha+M_PI) + M*g(S)*cos(beta+M_PI);
	s[0] += v[0]*dt;
	a[0] = F[0]/M;
	v[0] += a[0]*dt;
	
	/* y-direction	*/
	F[1] = Ft*sin(gam) + Fd*sin(alpha+M_PI) + M*g(S)*sin(beta+M_PI);
	s[1] += v[1]*dt;
	a[1] = F[1]/M;
	v[1] += a[1]*dt;

	S = sqrt(s[0]*s[0] + s[1]*s[1]);
	V = sqrt(v[0]*v[0] + v[1]*v[1]);
	A = sqrt(a[0]*a[0] + a[1]*a[1]);

	t += dt;
}

double throttle_test(double hy, double ux, double uy, double mf, double throttle)
{

	double VEL, ft, fd, mass;
	double fx, fy, ax, ay;

	VEL = sqrt(ux*ux + uy*uy);

	ft = throttle*GetThrust(hy, 1, 1);

	do
	{
		mf -= throttle*236*dt;
		mass = mf + F9.Mr[0];						

		fd = (0.5)*F9.Cd*F9.A*rho(hy-Re)*VEL*VEL;

		flip(2);

		fx = ft*cos(gam) + fd*cos(alpha+M_PI) + mass*g(hy)*cos(beta+M_PI);
		ax = fx/mass;
		ux += ax*dt;
	
		fy = ft*sin(gam) + fd*sin(alpha+M_PI) + mass*g(hy)*sin(beta+M_PI);
		hy += uy*dt;
		ay = fy/mass;
		uy += ay*dt;

		VEL = sqrt(ux*ux + uy*uy);
	}
	while(uy<0);

	return hy-Re;

}

inline double get_landing_throttle(double H, double ux, double uy, double mf)
{

	double a = 0.7, b = 1.0;
	double end_H;

	if( throttle_test(H, ux, uy, mf, b) > 0 )			// Will a full-power burn keep you alive?
	{
		if(throttle_test(H, ux, uy, mf, a) < 0 )			// Yes. Will a minimum-power burn kill you?
		{
			do
			{
				end_H = throttle_test(H, ux, uy, mf, (a+b)/2.0);
				if(fabs(end_H) < 0.1)
					return (a+b)/2.0;				// Yes. Burn at this throttle from now to do hoverslam.
				a = end_H < 0 ? (a+b)/2.0 : a;
				b = end_H > 0 ? (a+b)/2.0 : b;
			}
			while(1);
		}
		else return 0.0;						// No. Don't start yet. 
	}
	else return 1.0;						// No. Too late. Crash unavoidable. Should have started earlier

}

inline void update_landing_throttle()
{
	p = get_landing_throttle(S, v[0], v[1], F9.Mf[0]);
}

inline void angles()
{
	beta = acos(s[0]/S);
	if(s[1]<0)
		beta = 2*M_PI - beta;

	alpha = acos(v[0]/V);
	if(v[1]<0)
		alpha = 2*M_PI - alpha;
}

inline void grav_turn(int _M1, int _S1)
{
	gam = alpha;
	if(_M1 && V*cos(beta-alpha)<0 && !_S1)
		gam = asin(-M*g(S)*sin(beta+M_PI)/Ft);

	if(V > 2900)
		gam = beta - M_PI/2;
	if(V > 3500)
		gam = beta - M_PI/2 - 0.1;
	if(V > 3900)
		gam = beta - M_PI/2 - 0.2;
	if(V > 4400)
		gam = beta - M_PI/2 - 0.3;
	if(V > 5300)
		gam = beta - M_PI/2 - 0.4;
	if(V > 2200)
		p = 0.7;
}

#endif

