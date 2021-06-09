#include "Constant.h"
#include "FuncAirDyn.h"
#include "FuncStateTransition.h"
#include <math.h>
#include "FuncCanBeStablized.h"
#include <cmath>

using namespace std;


bool Func_DotStateForStb(State s0, double flap, double elevator)
{
	double CD = Func_CD(s0.alpha, flap, elevator);
	double CL = Func_CL(s0.alpha, flap, elevator);
	double T = (0.5 * (ro * V * V * Swing * CD) + m * g * sin(s0.theta - s0.alpha)) / cos(s0.alpha);
	double dotalpha = s0.q + (1.0 / (m * V)) * (-T * sin(s0.alpha) - 0.5 * ro * V * V * Swing * CL + m * g * cos(s0.theta - s0.alpha));
	double Cm = Func_Cm(s0.alpha, flap, elevator, s0.q, dotalpha);
	double dotq = (1.0 / Jyy) * (0.5 * ro * V * V * Swing * cBar * Cm);
	double dottheta = s0.q;
	if (Tmin <= T && Tmax >= T && abs(dotalpha) <= 0.03 && abs(dotq) <= 0.03 && abs(dottheta) <= 0.03)
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool Func_CanBeStablized(State s0)
{
	for (int i = 0; i < Nelevator; i++)
	{
		for (int j = 0; j < Nflap; j++)
		{
			double elevator = elevatormin + deltaelevator * i;
			double flap = flapmin + deltaflap * j;
			if (Func_DotStateForStb(s0, flap, elevator))
			{
				return true;
			}
		}
	}
	return false;
}

StateIndex Func_StateIndex(State s0)
{
	double ialpha_double = (s0.alpha - alphamin) / gridsizealpha;
	int ialpha_int = int(ialpha_double);
	int ialpha;
	if (ialpha_double - ialpha_int < 0.5)
	{
		ialpha = ialpha_int;
	}
	else
	{
		ialpha = ialpha_int + 1;
	}

	double itheta_double = (s0.theta - thetamin) / gridsizetheta;
	int itheta_int = int(itheta_double);
	int itheta;
	if (itheta_double - itheta_int < 0.5)
	{
		itheta = itheta_int;
	}
	else
	{
		itheta = itheta_int + 1;
	}

	double iq_double = (s0.q - qmin) / gridsizeq;
	int iq_int = int(iq_double);
	int iq;
	if (iq_double - iq_int < 0.5)
	{
		iq = iq_int;
	}
	else
	{
		iq = iq_int + 1;
	}
	return StateIndex{ ialpha,itheta,iq };
}




