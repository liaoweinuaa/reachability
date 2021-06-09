#pragma once
const double CD0 = 0.00108;//0升阻力系数
const double CDalpha = 0.01;
const double CDalpha2 = 0.6;
const double CDe = 0.05;//由升降舵偏角带来的阻力系数
const double CDf = 0.105;
const double CLe = 0.2;
//const double CLdotalpha = 17.2232;
//const double CLq = -17.2232;
const double CLf = 2;
const double CLalpha = 2.4;
const double CL0 = 0.1;

const double Cm0 = 0.04;
const double Cmalpha = -0.2;
const double Cme = -1.2;
const double Cmq = -1;
const double Cmdotalpha = -4;


const double elevatormax = 0.3;
const double elevatormin = -0.3;

const int Nelevator = 31;

const double deltaelevator = (elevatormax - elevatormin) / (Nelevator - 1);


const double flapmax = 0.69;
const double flapmin = 0;

const double dmax = 10000000;
const double dmin = -10000000;


const int Nd = 31;

const double deltad = (dmax - dmin) / (Nd - 1);

const int Nflap = 31;

const double deltaflap = (flapmax - flapmin) / (Nflap - 1);


const double V = 200;

const double Swing = 524;

const double m = 235717 * 1.0;
//const double Jxx = 1278369;
const double Jyy = 44856570 * 1.5;
const double cBar = 6.32;
const double PI = 3.141592653;

const double dt_ = 0.01;
const double g = 9.8;
const double ro = 1.293;

const double Tmax = 1029000;
const double Tmin = 205000;


const double alphamax = 0.4;
const double alphamin = -0.4;

const double thetamax = 0.75;
const double thetamin = -0.75;

const double qmax = 0.75;
const double qmin = -0.75;


const int Nalpha = 201;
const int Ntheta = 201;
const int Nq = 201;

const int Ntotalgrid = Nalpha * Ntheta * Nq;

const double gridsizealpha = (alphamax - alphamin) / (Nalpha - 1);
const double gridsizetheta = (thetamax - thetamin) / (Ntheta - 1);
const double gridsizeq = (qmax - qmin) / (Nq - 1);

const double gammat = 1;
const double gammaF = 0;

struct State
{
	double alpha;
	double theta;
	double q;
	double Ftotal;
};

struct DotState
{
	double dotalpha;
	double dottheta;
	double dotq;
	double Ftotal;
};

struct StateIndex
{
	int ialpha;
	int itheta;
	int iq;
};


struct ControlInput
{
	double flap;
	double elevator;
};