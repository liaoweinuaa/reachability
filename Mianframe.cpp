#include <fstream>
#include <string>
#include "Constant.h"
#include "FuncCanBeStablized.h"
#include "FuncStateTransition.h"
#include "MianFrame.h"
#include <iostream>
#include <mutex>
#include <thread>
using namespace std;

extern double Arr_StateValue[Nalpha][Ntheta][Nq];
extern double Arr_StateValueNew[Nalpha][Ntheta][Nq];
extern State Arr_State[Nalpha][Ntheta][Nq];


void Func_Initial()
{

	memset(Arr_StateValue, 0, sizeof(double) * Nalpha * Ntheta * Nq);
	memset(Arr_StateValueNew, 0, sizeof(double) * Nalpha * Ntheta * Nq);

	for (int i = 0; i < Nalpha; i++)
	{
		for (int j = 0; j < Ntheta; j++)
		{
			for (int k = 0; k < Nq; k++)
			{
				Arr_State[i][j][k].alpha = alphamin + gridsizealpha * i;
				Arr_State[i][j][k].theta = thetamin + gridsizetheta * j;
				Arr_State[i][j][k].q = qmin + gridsizeq * k;
			}
		}
	}


	//ifstream ifs1("statevalue10.dat", ios::binary | ios::in);
	//ifs1.read((char*)Arr_StateValue, sizeof(double) * Nalpha * Ntheta * Nq);
	//ifs1.close();
}

bool Func_IsTargetSet(State s0)
{
	if (s0.alpha<=0.05 && s0.alpha>=-0.05 && s0.q<=0.1 && s0.q>=-0.1 && s0.theta<=0.1 && s0.theta>=-0.1)
	{
		return true;
	}
	else
	{
		return false;
	}
}


//bool Func_IsTargetSet(State s0)
//{
//	double alpha = s0.alpha, theta = s0.theta, q = s0.q;
//
//	if (q<-0.1 || q>0.1)
//	{
//		return false;
//	}
//
//	if (alpha>=-0.16 && alpha<=0)
//	{
//		if (theta >= (alpha - (-0.16)) * 1.8875 - 0.457 && theta <= (alpha - (-0.16)) * 2.8 - 0.094 && theta>=-0.4)
//		{
//			return true;
//		}
//	}
//
//	if (alpha>0 && alpha<=0.083)
//	{
//		if (theta >= alpha  * (-0.3) - 0.155 && theta <= alpha  * (-0.537) +0.355 )
//		{
//			return true;
//		}
//	}
//
//	if (alpha > 0.083 && alpha <= 0.16)
//	{
//		if (theta >= (alpha-0.083) * (1.545) - 0.18 && theta <= alpha * (-0.537) + 0.355)
//		{
//			return true;
//		}
//	}
//
//	return false;
//}


State Func_StateTransitionHat(State s0, double flap, double elevator,double d,double dt)
{
	State snew;
	if (Func_IsTargetSet(s0))
	{
		snew = s0;
		snew.Ftotal = 0;
		return snew;
	}
	else
	{
		return Func_StateTransitionRK2(s0, flap, elevator,d,dt);
	}
}

double Func_InterPolation(State s0)
{
	if (Func_IsTargetSet(s0))
	{
		return 0;
	}
	double ialpha_double = ((s0.alpha - alphamin) / gridsizealpha);
	double itheta_double = ((s0.theta - thetamin) / gridsizetheta);
	double iq_double = ((s0.q - qmin) / gridsizeq);

	int ialpha = int(ialpha_double);
	int itheta = int(itheta_double);
	int iq = int(iq_double);

	double v000 = Arr_StateValue[ialpha][itheta][iq];
	double v001 = Arr_StateValue[ialpha][itheta][iq+1];
	double v010 = Arr_StateValue[ialpha][itheta+1][iq];
	double v011 = Arr_StateValue[ialpha][itheta+1][iq+1];
	double v100 = Arr_StateValue[ialpha+1][itheta][iq];
	double v101 = Arr_StateValue[ialpha+1][itheta][iq+1];
	double v110 = Arr_StateValue[ialpha+1][itheta+1][iq];
	double v111 = Arr_StateValue[ialpha+1][itheta+1][iq+1];

	//if (v000==0 && v001 == 0 && v010==0 && v011 == 0 && v100 == 0 && v101 == 0 && v110 == 0 && v111 == 0)
	//{
	//	return 0;
	//}

	double dalpha0 = ialpha_double - ialpha;
	double dalpha1 = 1 - dalpha0;
	double dtheta0 = itheta_double - itheta;
	double dtheta1 = 1 - dtheta0;
	double dq0 = iq_double - iq;
	double dq1 = 1 - dq0;

	double V000 = dalpha0 * dtheta0 * dq0;
	double V001 = dalpha0 * dtheta0 * dq1;
	double V010 = dalpha0 * dtheta1 * dq0;
	double V011 = dalpha0 * dtheta1 * dq1;
	double V100 = dalpha1 * dtheta0 * dq0;
	double V101 = dalpha1 * dtheta0 * dq1;
	double V110 = dalpha1 * dtheta1 * dq0;
	double V111 = dalpha1 * dtheta1 * dq1;

	double VTotal = gridsizealpha * gridsizetheta * gridsizeq;
	return (v000 * V111 + v001 * V110 + v010 * V101 + v011 * V100 + v100 * V011 + v101 * V010 + v110 * V001 + v111 * V000);
}

//double Func_ValueUnderOptInput(State s0,double dt)
//{
//	double minvalue = 1000000;
//	for (int i = 0; i < Nflap; i++)
//	{
//		//cout << i << endl;
//		double flap = flapmin + i * deltaflap;
//		for (int j = 0; j < Nelevator; j++)
//		{
//			double elevator = elevatormin + j * deltaelevator;
//			State snew = Func_StateTransitionHat(s0, flap, elevator,dt);
//			double snewvalue = Func_StateValue(snew);
//			double value = snew.Ftotal + snewvalue;
//			if (value<minvalue)
//			{
//				minvalue = value;
//			}
//		}
//	}
//	return minvalue;
//}

double Func_ValueUnderOptInput(State s0, double dt)
{
	double minvalue = 1000000;
	//cout << i << endl;
	double flap = 0;
	for (int j = 0; j < Nelevator; j++)
	{
		double elevator = elevatormin+j*deltaelevator;


		State snew = Func_StateTransitionHat(s0, flap, elevator,0, dt);
		double snewvalue = Func_InterPolation(snew);
		double value = snew.Ftotal + snewvalue;


		if (value<minvalue)
		{
			minvalue = value;
		}
	}

	return minvalue;
}

void Func_Recursion(double dt)
{
	for (int i = 0; i < Nalpha; i++)
	{
		for (int j = 0; j < Ntheta; j++)
		{
			cout << j << endl;
			for (int k = 0; k < Nq; k++)
			{
				Arr_StateValueNew[i][j][k] = Func_ValueUnderOptInput(Arr_State[i][j][k], dt);
			}
		}
	}
	memcpy(Arr_StateValue, Arr_StateValueNew, sizeof(double) * Nalpha * Ntheta * Nq);
}

mutex mt;
void Func_RecursionST(double dt,int *current_index)
{
	int loc_index;
	while (true)
	{
		mt.lock();
		loc_index = (*current_index);
		(*current_index)++;
		mt.unlock();
		if (loc_index%800000==0)
		{
			cout << loc_index << endl;
		}
		if (loc_index >= Ntotalgrid)
		{
			break;
		}

		int ialpha = loc_index / (Ntheta * Nq);
		int itheta = (loc_index - ialpha * (Ntheta * Nq)) / Nq;
		int iq = loc_index - ialpha * (Ntheta * Nq) - itheta * Nq;

		Arr_StateValueNew[ialpha][itheta][iq] = Func_ValueUnderOptInput(Arr_State[ialpha][itheta][iq], dt);


	}
}

void Func_RecursionMT(double dt)
{
	int current_index = 0;
	thread t0(Func_RecursionST, dt, &current_index);
	thread t1(Func_RecursionST, dt, &current_index);
	thread t2(Func_RecursionST, dt, &current_index);
	thread t3(Func_RecursionST, dt, &current_index);
	thread t4(Func_RecursionST, dt, &current_index);
	thread t5(Func_RecursionST, dt, &current_index);
	thread t6(Func_RecursionST, dt, &current_index);
	thread t7(Func_RecursionST, dt, &current_index);
	thread t8(Func_RecursionST, dt, &current_index);
	thread t9(Func_RecursionST, dt, &current_index);
	/*thread t10(Func_RecursionST, dt, &current_index);
	thread t11(Func_RecursionST, dt, &current_index);
	thread t12(Func_RecursionST, dt, &current_index);
	thread t13(Func_RecursionST, dt, &current_index);
	thread t14(Func_RecursionST, dt, &current_index);
	thread t15(Func_RecursionST, dt, &current_index);
	thread t16(Func_RecursionST, dt, &current_index);
	thread t17(Func_RecursionST, dt, &current_index);
	thread t18(Func_RecursionST, dt, &current_index);
	thread t19(Func_RecursionST, dt, &current_index);
	thread t20(Func_RecursionST, dt, &current_index);
	thread t21(Func_RecursionST, dt, &current_index);
	thread t22(Func_RecursionST, dt, &current_index);
	thread t23(Func_RecursionST, dt, &current_index);
	thread t24(Func_RecursionST, dt, &current_index);
	thread t25(Func_RecursionST, dt, &current_index);
	thread t26(Func_RecursionST, dt, &current_index);
	thread t27(Func_RecursionST, dt, &current_index);
	thread t28(Func_RecursionST, dt, &current_index);
	thread t29(Func_RecursionST, dt, &current_index);
	thread t30(Func_RecursionST, dt, &current_index);
	thread t31(Func_RecursionST, dt, &current_index);
	thread t32(Func_RecursionST, dt, &current_index);
	thread t33(Func_RecursionST, dt, &current_index);
	thread t34(Func_RecursionST, dt, &current_index);
	thread t35(Func_RecursionST, dt, &current_index);
	thread t36(Func_RecursionST, dt, &current_index);
	thread t37(Func_RecursionST, dt, &current_index);
	thread t38(Func_RecursionST, dt, &current_index);
	thread t39(Func_RecursionST, dt, &current_index);*/

	t0.join();
	t1.join();
	t2.join();
	t3.join();
	t4.join();
	t5.join();
	t6.join();
	t7.join();
	t8.join();
	t9.join();
	/*t10.join();
	t11.join();
	t12.join();
	t13.join();
	t14.join();
	t15.join();
	t16.join();
	t17.join();
	t18.join();
	t19.join();
	t20.join();
	t21.join();
	t22.join();
	t23.join();
	t24.join();
	t25.join();
	t26.join();
	t27.join();
	t28.join();
	t29.join();
	t30.join();
	t31.join();
	t32.join();
	t33.join();
	t34.join();
	t35.join();
	t36.join();
	t37.join();
	t38.join();
	t39.join();*/
	memcpy(Arr_StateValue, Arr_StateValueNew, sizeof(double) * Nalpha * Ntheta * Nq);
}


void Func_SavaData(string filename0,string filename1)
{
	ofstream ofs(filename0, ios::binary | ios::out);
	ofs.write((const char*)Arr_StateValue, sizeof(double) * Nalpha * Ntheta * Nq);
	ofs.close();

	ofstream ofs1;
	ofs1.open(filename1);
	for (int i = 0; i < Nalpha; i+=2)
	{
		for (int j = 0; j < Ntheta; j+=2)
		{
			for (int k = 0; k < Nq; k+=2)
			{
				ofs1 << Arr_StateValue[i][j][k] << endl;
			}
		}
	}
	ofs1.close();
}



double Func_StateValue(State s0)
{
	int mask[3] = { 0,0,0 };
	int nrt_ix, nrt_iy, nrt_iz;
	double dbix = (s0.alpha - alphamin) / gridsizealpha;
	double dbiy = (s0.theta - thetamin) / gridsizetheta;
	double dbiz = (s0.q - qmin) / gridsizeq;

	int ix0 = int(dbix);
	int iy0 = int(dbiy);
	int iz0 = int(dbiz);

	int ix1 = ix0 + 1;
	int iy1 = iy0 + 1;
	int iz1 = iz0 + 1;

	if (dbix - ix0 > 0.5)
	{
		nrt_ix = ix1;
		mask[0] = 1;
	}
	else
	{
		nrt_ix = ix0;
	}

	if (dbiy - iy0 > 0.5)
	{
		nrt_iy = iy1;
		mask[1] = 1;
	}
	else
	{
		nrt_iy = iy0;
	}

	if (dbiz - iz0 > 0.5)
	{
		nrt_iz = iz1;
		mask[2] = 1;
	}
	else
	{
		nrt_iz = iz0;
	}

	double v1, v2, v3, v4;
	double a, b, c, d;
	double dx, dy, dz;

	if (abs(dbix - nrt_ix) + abs(dbiy - nrt_iy) + abs(dbiz - nrt_iz) > 1)
	{
		v1 = Arr_StateValue[ix1][iy0][iz0];
		v2 = Arr_StateValue[ix0][iy1][iz0];
		v3 = Arr_StateValue[ix0][iy0][iz1];
		v4 = Arr_StateValue[ix1][iy1][iz1];

		d = 0.5 * (v1 + v2 + v3 - v4);
		a = v1 - d;
		b = v2 - d;
		c = v3 - d;

		dx = dbix - ix0;
		dy = dbiy - iy0;
		dz = dbiz - iz0;
		double s1 = a * dx + b * dy + c * dz + d;

		v1 = Arr_StateValue[ix0][iy0][iz0];
		v2 = Arr_StateValue[ix1][iy1][iz0];
		v3 = Arr_StateValue[ix0][iy1][iz1];
		v4 = Arr_StateValue[ix1][iy0][iz1];

		d = v1;
		a = 0.5 * (v2 - v3 + v4 - v1);
		b = v2 - a - d;
		c = v3 - b - d;

		dx = dbix - ix0;
		dy = dbiy - iy0;
		dz = dbiz - iz0;
		double s2 = a * dx + b * dy + c * dz + d;

		return 0.5 * (s1 + s2);
	}

	int key = mask[0] + mask[1] * 2 + mask[2] * 4;
	switch (key)
	{
	case 0:
		v1 = Arr_StateValue[ix0][iy0][iz0];
		v2 = Arr_StateValue[ix1][iy0][iz0];
		v3 = Arr_StateValue[ix0][iy1][iz0];
		v4 = Arr_StateValue[ix0][iy0][iz1];

		d = v1;
		a = v2 - v1;
		b = v3 - v1;
		c = v4 - v1;

		dx = dbix - ix0; dy = dbiy - iy0; dz = dbiz - iz0;
		return a * dx + b * dy + c * dz + d;

	case 1:
		v1 = Arr_StateValue[ix1][iy0][iz0];
		v2 = Arr_StateValue[ix0][iy0][iz0];
		v3 = Arr_StateValue[ix1][iy1][iz0];
		v4 = Arr_StateValue[ix1][iy0][iz1];
		d = v1;
		a = v1 - v2;
		b = v3 - v1;
		c = v4 - v1;
		dx = dbix - ix1; dy = dbiy - iy0; dz = dbiz - iz0;
		return a * dx + b * dy + c * dz + d;

	case 2:
		v1 = Arr_StateValue[ix0][iy1][iz0];
		v2 = Arr_StateValue[ix1][iy1][iz0];
		v3 = Arr_StateValue[ix0][iy0][iz0];
		v4 = Arr_StateValue[ix0][iy1][iz1];
		d = v1;
		a = v2 - v1;
		b = v1 - v3;
		c = v4 - v1;
		dx = dbix - ix0; dy = dbiy - iy1; dz = dbiz - iz0;
		return a * dx + b * dy + c * dz + d;

	case 3:
		v1 = Arr_StateValue[ix1][iy1][iz0];
		v2 = Arr_StateValue[ix0][iy1][iz0];
		v3 = Arr_StateValue[ix1][iy0][iz0];
		v4 = Arr_StateValue[ix1][iy1][iz1];
		d = v1;
		a = v1 - v2;
		b = v1 - v3;
		c = v4 - v1;
		dx = dbix - ix1; dy = dbiy - iy1; dz = dbiz - iz0;
		return a * dx + b * dy + c * dz + d;

	case 4:
		v1 = Arr_StateValue[ix0][iy0][iz1];
		v2 = Arr_StateValue[ix1][iy0][iz1];
		v3 = Arr_StateValue[ix0][iy1][iz1];
		v4 = Arr_StateValue[ix0][iy0][iz0];

		d = v1;
		a = v2 - v1;
		b = v3 - v1;
		c = v1 - v4;

		dx = dbix - ix0; dy = dbiy - iy0; dz = dbiz - iz1;
		return a * dx + b * dy + c * dz + d;

	case 5:
		v1 = Arr_StateValue[ix1][iy0][iz1];
		v2 = Arr_StateValue[ix0][iy0][iz1];
		v3 = Arr_StateValue[ix1][iy1][iz1];
		v4 = Arr_StateValue[ix1][iy0][iz0];
		d = v1;
		a = v1 - v2;
		b = v3 - v1;
		c = v1 - v4;
		dx = dbix - ix1; dy = dbiy - iy0; dz = dbiz - iz1;
		return a * dx + b * dy + c * dz + d;

	case 6:
		v1 = Arr_StateValue[ix0][iy1][iz1];
		v2 = Arr_StateValue[ix1][iy1][iz1];
		v3 = Arr_StateValue[ix0][iy0][iz1];
		v4 = Arr_StateValue[ix0][iy1][iz0];
		d = v1;
		a = v2 - v1;
		b = v1 - v3;
		c = v1 - v4;
		dx = dbix - ix0; dy = dbiy - iy1; dz = dbiz - iz1;
		return a * dx + b * dy + c * dz + d;
	case 7:
		v1 = Arr_StateValue[ix1][iy1][iz1];
		v2 = Arr_StateValue[ix0][iy1][iz1];
		v3 = Arr_StateValue[ix1][iy0][iz1];
		v4 = Arr_StateValue[ix1][iy1][iz0];
		d = v1;
		a = v1 - v2;
		b = v1 - v3;
		c = v1 - v4;
		dx = dbix - ix1; dy = dbiy - iy1; dz = dbiz - iz1;
		return a * dx + b * dy + c * dz + d;


	default:
		cout << "interpolation error!!!!" << endl;
		return 0;
		break;
	}
}

//ControlInput Func_OptimalControlInputs(State s0,double dt)
//{
//	double minvalue = 1000000;
//	double optf;
//	double opte;
//	for (int i = 0; i < Nflap*10; i++)
//	{
//		//cout << i << endl;
//		double flap = flapmin + i * deltaflap*0.1;
//		for (int j = 0; j < Nelevator*10; j++)
//		{
//			double elevator = elevatormin + j * deltaelevator*0.1;
//			State snew = Func_StateTransitionHat(s0, flap, elevator, dt);
//			double snewvalue = Func_StateValue(snew);
//			double value = snew.Ftotal + snewvalue;
//			if (value < minvalue)
//			{
//				minvalue = value;
//				optf = flap;
//				opte = elevator;
//			}
//		}
//	}
//	return ControlInput{ optf,opte };
//}


//void Func_Simulation(State s0, double dt)
//{
//	ofstream ofs;
//	ofs.open("·ÂÕæÇúÏß.csv");
//	double cumulative_cost = 0;
//	State current_state = s0;
//	for (int i = 0; i < 1000; i++)
//	{
//		ControlInput optcontrol = Func_OptimalControlInputs(s0, dt);
//		ofs << current_state.alpha << "," << current_state.theta << "," << current_state.q << "," << optcontrol.flap << "," << optcontrol.elevator << "," << cumulative_cost << endl;
//		current_state = Func_StateTransition(current_state, optcontrol.flap, optcontrol.elevator, dt);
//		if (Func_IsTargetSet(current_state))
//		{
//			break;
//		}
//	}
//	ofs.close();
//}