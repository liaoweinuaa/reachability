#include "Constant.h"
#include "FuncCDalpha.h"
#include "FuncCLalpha.h"
#include "FuncCm0.h"
#include "FuncAirDyn.h"

double Func_CL(double alpha, double flap, double elevator)
{
	return CL0+CLalpha*alpha + CLe * elevator + CLf * flap;
}


double Func_CD(double alpha, double flap, double elevator)
{
	//double CDalpha = Func_CDalpha(alpha);
	return CDalpha*alpha+CDalpha2*alpha*alpha  + CD0 + CDe * elevator+CDf*flap;
}


double Func_Cm(double alpha, double flap, double elevator,double q,double dotalpha)
{
	//return  Cmalpha*alpha  + Cmdotalpha * dotalpha + Cmq * q + Cme * elevator;
	//return Cm0+ Cmalpha * alpha + Cmq * q * cBar / (2 * V) + Cme * elevator;
	return Cm0 + Cmalpha * alpha  + Cme * elevator + Cmq * q * cBar / (2 * V);
}



