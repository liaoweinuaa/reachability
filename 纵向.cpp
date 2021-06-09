// 纵向.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "FuncCDalpha.h"
#include "FuncCanBeStablized.h"
#include <fstream>
#include "Constant.h"
#include "FuncCLalpha.h"
#include <string>
#include "FuncStateTransition.h"
#include "MianFrame.h"

using namespace std;



double Arr_StateValue[Nalpha][Ntheta][Nq];
double Arr_StateValueNew[Nalpha][Ntheta][Nq];
State Arr_State[Nalpha][Ntheta][Nq];




string filenamelist0[10] = { "statevalue1.dat" ,"statevalue2.dat" ,"statevalue3.dat" ,"statevalue4.dat" ,"statevalue5.dat" ,"statevalue6.dat" ,
"statevalue7.dat" ,"statevalue8.dat" ,"statevalue9.dat" ,"statevalue10.dat" };
string filenamelist1[10] = { "statevalue1.csv" ,"statevalue2.csv" ,"statevalue3.csv" ,"statevalue4.csv" ,"statevalue5.csv" ,"statevalue6.csv" ,
"statevalue7.csv" ,"statevalue8.csv" ,"statevalue9.csv" ,"statevalue10.csv" };

int main()
{
	Func_Initial();
	int filenameindex = 0;
	for (int i = 0; i < 110; i++)
	{
		cout << "当前第" << i << "次递归" << endl;
		Func_RecursionMT(dt_);
		if ((i + 1) % 11 == 0)
		{
			Func_SavaData(filenamelist0[filenameindex], filenamelist1[filenameindex]);
			filenameindex++;
		}
	}
}

//int main()
//{
//	State s{ 0,0,-0.015 };
//	double elevator = 0.175;
//	double flap = 0;
//	//Func_DotState(s, flap, elevator);
//	Func_StateTransitionRK2(s, flap, elevator,0.2);
//}