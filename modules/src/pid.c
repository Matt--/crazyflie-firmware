#define LIBRARY_TESTING false

#include <stdio.h>
#include <stdbool.h>

#define STRINGMAX 10 // used in snprint functions
#define real float // can be changed to suit application
#include "stm32f10x_conf.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "motors.h"
#include "task.h"
#include "system.h"
#include "stabilizer.h"
#include "commander.h"
#include "sensfusion6.h"
#include "param.h"
#include "imu.h"
#include "log.h"

typedef struct {
    real deriv; 
    real desired; 
    real dt; 
    real error; 
    real iLimit; 
    real iLimitLow; 
    real integ; 
    real kd; 
    real ki; 
    real kp; 
    real outD; 
    real outI; 
    real outP; 
    real prevError; 
    } PidObject;

void pidInit ( PidObject*, real, real, real, real, real );
real pidUpdate ( PidObject*, real, bool );
bool pidIsActive ( PidObject* );
void pidSetIntegralLimit ( PidObject*, real );
void pidSetIntegralLimitLow ( PidObject*, real );
void pidReset ( PidObject* );
void pidSetError ( PidObject*, real );
void pidSetDesired ( PidObject*, real );
real pidGetDesired ( PidObject* );
void pidSetKp ( PidObject*, real );
void pidSetKi ( PidObject*, real );
void pidSetKd ( PidObject*, real );
void pidSetDt ( PidObject*, real );

void pidInit ( PidObject* a0, real a1, real a2, real a3, real a4, real a5 ){
  real a6 = 0.0;
  a0->error = a6;
  real a7 = 0.0;
  a0->prevError = a7;
  real a8 = 0.0;
  a0->integ = a8;
  real a9 = 0.0;
  a0->deriv = a9;
  a0->desired = a1;
  a0->kp = a2;
  a0->ki = a3;
  a0->kd = a4;
  real a14 = 5000.0;
  a0->iLimit = a14;
  real a15 = -5000.0;
  a0->iLimitLow = a15;
  a0->dt = a5;
  return;
}

real pidUpdate ( PidObject* a0, real a1, bool a2 ){
  bool a5 = true;
  if ( a2 == a5 ) { goto label0; };
  goto label1;
  label0: ;
  PidObject a7 = *a0;
  real a8 = a7.desired;
  real a10 = a8 - a1;
  a0->error = a10;
  label1: ;
  PidObject a12 = *a0;
  real a13 = a12.integ;
  PidObject a15 = *a0;
  real a16 = a15.error;
  PidObject a18 = *a0;
  real a19 = a18.dt;
  real a20 = a16 * a19;
  real a21 = a13 + a20;
  a0->integ = a21;
  PidObject a23 = *a0;
  real a24 = a23.integ;
  PidObject a26 = *a0;
  real a27 = a26.iLimit;
  if ( a24 <= a27 ) { goto label2; };
  PidObject a29 = *a0;
  real a30 = a29.iLimit;
  a0->integ = a30;
  goto label3;
  label2: ;
  PidObject a32 = *a0;
  real a33 = a32.integ;
  PidObject a35 = *a0;
  real a36 = a35.iLimitLow;
  if ( a33 >= a36 ) { goto label3; };
  PidObject a38 = *a0;
  real a39 = a38.iLimitLow;
  a0->integ = a39;
  label3: ;
  PidObject a41 = *a0;
  real a42 = a41.error;
  PidObject a44 = *a0;
  real a45 = a44.prevError;
  real a46 = a42 - a45;
  PidObject a48 = *a0;
  real a49 = a48.dt;
  real a50 = a46 / a49;
  a0->deriv = a50;
  PidObject a52 = *a0;
  real a53 = a52.kp;
  PidObject a55 = *a0;
  real a56 = a55.error;
  real a57 = a53 * a56;
  a0->outP = a57;
  PidObject a59 = *a0;
  real a60 = a59.ki;
  PidObject a62 = *a0;
  real a63 = a62.integ;
  real a64 = a60 * a63;
  a0->outI = a64;
  PidObject a66 = *a0;
  real a67 = a66.kd;
  PidObject a69 = *a0;
  real a70 = a69.deriv;
  real a71 = a67 * a70;
  a0->outD = a71;
  PidObject a73 = *a0;
  real a74 = a73.outP;
  PidObject a76 = *a0;
  real a77 = a76.outI;
  real a78 = a74 + a77;
  PidObject a80 = *a0;
  real a81 = a80.outD;
  real a82 = a78 + a81;
  PidObject a84 = *a0;
  real a85 = a84.error;
  a0->prevError = a85;
  return a82;
}

bool pidIsActive ( PidObject* a0 ){
  bool a2 = true;
  PidObject a4 = *a0;
  real a5 = a4.kp;
  real a6 = 0.0001;
  if ( a5 >= a6 ) { goto label4; };
  PidObject a8 = *a0;
  real a9 = a8.ki;
  real a10 = 0.0001;
  if ( a9 >= a10 ) { goto label4; };
  PidObject a12 = *a0;
  real a13 = a12.kd;
  real a14 = 0.0001;
  if ( a13 >= a14 ) { goto label4; };
  bool a15 = false;
  a2 = a15;
  label4: ;
  return a2;
}

void pidSetIntegralLimit ( PidObject* a0, real a1 ){
  a0->iLimit = a1;
  return;
}

void pidSetIntegralLimitLow ( PidObject* a0, real a1 ){
  a0->iLimitLow = a1;
  return;
}

void pidReset ( PidObject* a0 ){
  real a1 = 0.0;
  a0->error = a1;
  real a2 = 0.0;
  a0->prevError = a2;
  real a3 = 0.0;
  a0->integ = a3;
  real a4 = 0.0;
  a0->deriv = a4;
  return;
}

void pidSetError ( PidObject* a0, real a1 ){
  a0->error = a1;
  return;
}

void pidSetDesired ( PidObject* a0, real a1 ){
  a0->desired = a1;
  return;
}

real pidGetDesired ( PidObject* a0 ){
  PidObject a2 = *a0;
  real a3 = a2.desired;
  return a3;
}

void pidSetKp ( PidObject* a0, real a1 ){
  a0->kp = a1;
  return;
}

void pidSetKi ( PidObject* a0, real a1 ){
  a0->ki = a1;
  return;
}

void pidSetKd ( PidObject* a0, real a1 ){
  a0->kd = a1;
  return;
}

void pidSetDt ( PidObject* a0, real a1 ){
  a0->dt = a1;
  return;
}

