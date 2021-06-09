#pragma once
#include "Constant.h"

DotState Func_DotState(State s0, double flap, double elevator,double d);


State Func_StateTransition(State s0, double flap, double elevator,double d,double dt);

State Func_StateTransitionRK2(State s0, double flap, double elevator, double d, double dt);