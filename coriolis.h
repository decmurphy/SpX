
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
extern int _pitch, 
	_ME1, _ME2, _ME3, _ME4,
	_MECO1, _MECO2, _MECO3, _MECO4,
	_SE1, _SECO;

double S, VA, VR, A, M;
double q, Ft, Fd, *Fuel;
double dm, t = 0.0;
extern double dt, p;
				// incl = 28.49*M_PI/180;
double vE = 407.6614278; 	// 2*M_PI*Re*cos(incl)/(24*60*60);

double F[2] = {0, 0};
double s[2] = {0, Re};
double a[2] = {0, -g0};
double vA[2] = {0, 0};
double vR[2] = {0, 0};

double alpha = 0, beta = M_PI/2, gam = M_PI/2;

/////////////////////////////////////////////////

inline void ME_ignition(int x)
{
	p = 1;
	Fuel = &(F9.Mf[0]);
	if(x==1) _ME1 = 1;
	if(x==2) _ME2 = 1;
	if(x==3) _ME3 = 1;
	if(x==4) _ME4 = 1;
}

inline first_step()
{
	M = F9.Mr[0] + F9.Mr[1] + F9.Mf[0] + F9.Mf[1] + F9.Mp;

	vA[0] = vE;

	F[1] = 9*M1D.Th_sl - M*g0;
	a[1] = F[1]/M;
	vR[1] += a[1]*dt/2;

	S = sqrt(s[0]*s[0] + s[1]*s[1]);
	A = sqrt(a[0]*a[0] + a[1]*a[1]);
}

inline void pitch_kick()
{
	gam = M_PI/2 - 0.025;
	_pitch = 1;
}

inline void MECO(int x)
{
	p = 0;
	if(x==1) 	_MECO1 = 1;
	else if(x==2) 	_MECO2 = 1;
	else if(x==3) 	_MECO3 = 1;
	else if(x==4) 	_MECO4 = 1;
}

inline void stage_sep(int stage_follow)
{

	if(stage_follow == 1)
		M -= (F9.Mr[1] + F9.Mf[1] + F9.Mp);
	else if(stage_follow == 2)
		M -= (F9.Mr[0] + F9.Mf[0]);
}

inline void flip(int x) {

	if(x==1)	gam = beta + M_PI/2;
	else if(x==2)	gam = alpha + M_PI;

}

inline void SE_ignition()
{
	p = 1;
	Fuel = &(F9.Mf[1]);
	_SE1 = 1;
}

inline void SECO()
{
	p = 0;
	_SECO = 1;
}

/////////////////////////////////////////////////

inline double g(double h)
{
	return G*Me/(h*h);
}

inline double rho(double h)
{
	return 1.21147*exp(h*-1.12727e-4);
}

inline double Isp(double h)
{
	if(h<7600)
		return M1D.Isp_sl + h*3.8e-3;
	else
		return M1D.Isp_vac;
}

inline double GetThrust(int stage)
{
	if(stage==1 || !_MECO1)
		return Isp(S-Re)*236*g0;
	else if(stage==2)
		return M1Dv.Th_vac;
}

void leapfrog_step(int stage)
{
	double num_engines;

	if(stage==2 && _SE1) {
		if(!_SECO)		 num_engines = 1;
		else			 num_engines = 0;
	}
	else {
		if(_ME1 && !_MECO1)	 num_engines = 9;	// Launch
		else if(_ME2 && !_MECO2) num_engines = 3;	// Retro Burn
		else if(_ME4 && !_MECO4) num_engines = 1;	// Braking Burn
		else if(_ME3 && !_MECO3) num_engines = 1;	// Landing Burn
		else			 num_engines = 0;
	}

	Ft = num_engines*p*GetThrust(stage);
	dm = num_engines*p*236*dt;
	*Fuel -= dm;
	M -= dm;

	q = 0.5*rho(S-Re)*VR*VR*1e-3;
	Fd = (0.5)*F9.Cd*F9.A*rho(S-Re)*VR*VR;

	/* x-direction	*/
	F[0] = Ft*cos(gam) + Fd*cos(alpha+M_PI) + M*g(S)*cos(beta+M_PI);
	s[0] += vA[0]*dt;
	a[0] = F[0]/M;
	vA[0] += a[0]*dt;
	vR[0] = vA[0] - vE*sin(beta);
	
	/* y-direction	*/
	F[1] = Ft*sin(gam) + Fd*sin(alpha+M_PI) + M*g(S)*sin(beta+M_PI);
	s[1] += vA[1]*dt;
	a[1] = F[1]/M;
	vA[1] += a[1]*dt;
	vR[1] = vA[1] - vE*cos(M_PI + beta);

	S = sqrt(s[0]*s[0] + s[1]*s[1]);
	VA = sqrt(vA[0]*vA[0] + vA[1]*vA[1]);
	VR = sqrt(vR[0]*vR[0] + vR[1]*vR[1]);
	A = sqrt(a[0]*a[0] + a[1]*a[1]);

	t += dt;
}

inline void angles()
{
	beta = acos(s[0]/S);
	if(s[1]<0)
		beta = 2*M_PI - beta;

	alpha = acos(vR[0]/VR);
	if(vR[1]<0)
		alpha = 2*M_PI - alpha;

}

inline void grav_turn()
{
	gam = alpha;
	if(_MECO1 && cos(beta-alpha)<0 && !_SECO)
		gam = asin(-M*g(S)*sin(beta+M_PI)/Ft);

	if(VR > 2900)
		gam = beta - M_PI/2;
	if(VR > 3500)
		gam = beta - M_PI/2 - 0.1;
	if(VR > 3900)
		gam = beta - M_PI/2 - 0.2;
	if(VR > 4400)
		gam = beta - M_PI/2 - 0.3;
	if(VR > 5300)
		gam = beta - M_PI/2 - 0.4;
	if(VR > 2200)
		p = 0.7;
}

double mod(double a, double b)
{
	return a < b ? a : mod(a-b, b);
}

#endif

