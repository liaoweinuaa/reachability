#include "Constant.h"
#include "FuncAirDyn.h"
#include <math.h>
#include "FuncStateTransition.h"
using namespace std;




DotState Func_DotState(State s0, double flap, double elevator,double d)
{
	double CD = Func_CD(s0.alpha, flap, elevator);
	double CL = Func_CL(s0.alpha, flap, elevator);
	double L = 0.5 * ro * V * V * Swing * CL;
	double D = 0.5 * (ro * V * V * Swing * CD);
	double T = (D + m * g * sin(s0.theta - s0.alpha)) / cos(s0.alpha);
	double G = m * g;
	double Fx = T * cos(s0.theta) - L * sin(s0.theta - s0.alpha) - D * cos(s0.theta - s0.alpha);
	double Fy = T * sin(s0.theta) + L * cos(s0.theta - s0.alpha) - D * sin(s0.theta - s0.alpha);
	double Ftotal = sqrt(Fx * Fx + Fy * Fy);

	double dotalpha = s0.q + (1.0 / (m * V)) * (-T * sin(s0.alpha) - L + m * g * cos(s0.theta - s0.alpha));
	double Cm = Func_Cm(s0.alpha, flap, elevator, s0.q, dotalpha);
	double dotq = (1.0 / Jyy) * (0.5 * ro * V * V * Swing * cBar * Cm+d);
	double dottheta = s0.q;
	return DotState{ dotalpha,dottheta,dotq,Ftotal/G };
}


State Func_StateTransition(State s0, double flap, double elevator,double d,double dt)
{
	DotState ds1 = Func_DotState(s0, flap, elevator,d);
	State s1{ s0.alpha + ds1.dotalpha * 0.5 * dt,s0.theta + ds1.dottheta * 0.5 * dt,s0.q + ds1.dotq * 0.5 * dt };
	DotState ds2 = Func_DotState(s1, flap, elevator,d);
	State s2{ s0.alpha + ds2.dotalpha * 0.5 * dt,s0.theta + ds2.dottheta * 0.5 * dt,s0.q + ds2.dotq * 0.5 * dt };
	DotState ds3 = Func_DotState(s2, flap, elevator,d);
	State s3{ s0.alpha + ds3.dotalpha *  dt,s0.theta + ds3.dottheta *  dt,s0.q + ds3.dotq * dt };
	DotState ds4 = Func_DotState(s3, flap, elevator,d);

	DotState ds{
		ds1.dotalpha + 2 * ds2.dotalpha + 2 * ds3.dotalpha + ds4.dotalpha,
		ds1.dottheta + 2 * ds2.dottheta + 2 * ds3.dottheta + ds4.dottheta,
		ds1.dotq + 2 * ds2.dotq + 2 * ds3.dotq + ds4.dotq,
	};

	double newalpha = s0.alpha + ds.dotalpha * dt / 6.0;
	double newtheta = s0.theta + ds.dottheta * dt / 6.0;
	double newq = s0.q + ds.dotq * dt / 6.0;
	double newFtoral =gammat*dt+ gammaF*(ds1.Ftotal + 2 * ds2.Ftotal + 2 * ds3.Ftotal + ds4.Ftotal)*dt / 6.0;

	if (newalpha>=alphamax)
	{
		newalpha = alphamax - 0.0001;
	}

	if (newalpha <= alphamin)
	{
		newalpha = alphamin + 0.0001;
	}

	if (newtheta >= thetamax)
	{
		newtheta = thetamax - 0.0001;
	}

	if (newtheta <= thetamin)
	{
		newtheta = thetamin + 0.0001;
	}

	if (newq >= qmax)
	{
		newq = qmax - 0.0001;
	}

	if (newq <= qmin)
	{
		newq = qmin + 0.0001;
	}

	return State{newalpha,newtheta,newq,newFtoral};
}


State Func_StateTransitionRK2(State s0, double flap, double elevator,double d, double dt)
{
	DotState ds1 = Func_DotState(s0, flap, elevator,d);
	State s1{ s0.alpha + ds1.dotalpha  * dt,s0.theta + ds1.dottheta  * dt,s0.q + ds1.dotq  * dt };
	DotState ds2 = Func_DotState(s1, flap, elevator,d);


	DotState ds{
		ds1.dotalpha + ds2.dotalpha ,
		ds1.dottheta + ds2.dottheta ,
		ds1.dotq + ds2.dotq
	};

	double newalpha = s0.alpha + ds.dotalpha * dt / 2.0;
	double newtheta = s0.theta + ds.dottheta * dt / 2.0;
	double newq = s0.q + ds.dotq * dt / 2.0;
	double newFtoral = gammat * dt +gammaF * (ds1.Ftotal + 2 * ds2.Ftotal) * dt / 2.0;

	if (newalpha >= alphamax)
	{
		newalpha = alphamax - 0.0001;
	}

	if (newalpha <= alphamin)
	{
		newalpha = alphamin + 0.0001;
	}

	if (newtheta >= thetamax)
	{
		newtheta = thetamax - 0.0001;
	}

	if (newtheta <= thetamin)
	{
		newtheta = thetamin + 0.0001;
	}

	if (newq >= qmax)
	{
		newq = qmax - 0.0001;
	}

	if (newq <= qmin)
	{
		newq = qmin + 0.0001;
	}

	return State{ newalpha,newtheta,newq,newFtoral };
}