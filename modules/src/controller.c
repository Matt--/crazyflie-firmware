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

void controllerInit ( PidObject** );
bool controllerTest ( void );
void dereferenceEq ( int*, int );
void dereferenceEqReal ( real*, real );
void controllerCorrectAttitudePID ( real, real, real, real, real, real, real*, real*, real*, PidObject*, PidObject*, PidObject* );
void controllerCorrectRatePID ( real*, real, real, real, int*, int*, int*, PidObject*, PidObject*, PidObject* );
void controllerResetAllPID ( PidObject** );

bool isControllerInit (  ); 
bool getControllerIsInit (  ); 
void pidInit ( PidObject*, real, real, real, real, real ); 
real pidUpdate ( PidObject*, real, bool ); 
void pidSetIntegralLimit ( PidObject*, real ); 
void pidSetDesired ( PidObject*, real ); 
void pidReset ( PidObject* ); 
void pidSetError ( PidObject*, real );

void controllerInit ( PidObject** a0 ){
  bool a1 = isControllerInit (  );
  bool a2 = true;
  if ( a1 == a2 ) { goto label0; };
  goto label1;
  label0: ;
  return;
  label1: ;
  int a4 = 0;
  PidObject *a5 = a0[ a4 ];
  real a6 = 0.0;
  real a7 = 70.0;
  real a8 = 0.0;
  real a9 = 0.0;
  real a10 = 1.0;
  real a11 = 500.0;
  real a12 = a10 / a11;
  pidInit ( a5, a6, a7, a8, a9, a12 );
  int a14 = 1;
  PidObject *a15 = a0[ a14 ];
  real a16 = 0.0;
  real a17 = 70.0;
  real a18 = 0.0;
  real a19 = 0.0;
  real a20 = 1.0;
  real a21 = 500.0;
  real a22 = a20 / a21;
  pidInit ( a15, a16, a17, a18, a19, a22 );
  int a24 = 2;
  PidObject *a25 = a0[ a24 ];
  real a26 = 0.0;
  real a27 = 50.0;
  real a28 = 25.0;
  real a29 = 0.0;
  real a30 = 1.0;
  real a31 = 500.0;
  real a32 = a30 / a31;
  pidInit ( a25, a26, a27, a28, a29, a32 );
  int a34 = 0;
  PidObject *a35 = a0[ a34 ];
  real a36 = 100.0;
  pidSetIntegralLimit ( a35, a36 );
  int a38 = 1;
  PidObject *a39 = a0[ a38 ];
  real a40 = 100.0;
  pidSetIntegralLimit ( a39, a40 );
  int a42 = 2;
  PidObject *a43 = a0[ a42 ];
  real a44 = 500;
  pidSetIntegralLimit ( a43, a44 );
  int a46 = 3;
  PidObject *a47 = a0[ a46 ];
  real a48 = 0.0;
  real a49 = 3.5;
  real a50 = 2.0;
  real a51 = 0.0;
  real a52 = 1.0;
  real a53 = 500.0;
  real a54 = a52 / a53;
  pidInit ( a47, a48, a49, a50, a51, a54 );
  int a56 = 4;
  PidObject *a57 = a0[ a56 ];
  real a58 = 0.0;
  real a59 = 3.5;
  real a60 = 2.0;
  real a61 = 0.0;
  real a62 = 1.0;
  real a63 = 500.0;
  real a64 = a62 / a63;
  pidInit ( a57, a58, a59, a60, a61, a64 );
  int a66 = 5;
  PidObject *a67 = a0[ a66 ];
  real a68 = 0.0;
  real a69 = 0.0;
  real a70 = 0.0;
  real a71 = 0.0;
  real a72 = 1.0;
  real a73 = 500.0;
  real a74 = a72 / a73;
  pidInit ( a67, a68, a69, a70, a71, a74 );
  int a76 = 3;
  PidObject *a77 = a0[ a76 ];
  real a78 = 20.0;
  pidSetIntegralLimit ( a77, a78 );
  int a80 = 4;
  PidObject *a81 = a0[ a80 ];
  real a82 = 20.0;
  pidSetIntegralLimit ( a81, a82 );
  int a84 = 5;
  PidObject *a85 = a0[ a84 ];
  real a86 = 360.0;
  pidSetIntegralLimit ( a85, a86 );
  return;
}

bool controllerTest (void){
  bool a0 = getControllerIsInit (  );
  return a0;
}

void dereferenceEq ( int* a0, int a1 ){
  return;
}

void dereferenceEqReal ( real* a0, real a1 ){
  return;
}

void controllerCorrectAttitudePID ( real a0, real a1, real a2, real a3, real a4, real a5, real* a6, real* a7, real* a8, PidObject* a9, PidObject* a10, PidObject* a11 ){
  pidSetDesired ( a9, a3 );
  bool a18 = true;
  real a15 = pidUpdate ( a9, a0, a18 );
  *a6 = a15;
  pidSetDesired ( a10, a4 );
  bool a25 = true;
  real a22 = pidUpdate ( a10, a1, a25 );
  *a7 = a22;
  real a29 = a5 - a2;
  real a31 = 180.0;
  if ( a29 <= a31 ) { goto label2; };
  real a33 = 360.0;
  real a34 = a29 - a33;
  a29 = a34;
  goto label3;
  label2: ;
  real a36 = -180.0;
  if ( a29 >= a36 ) { goto label3; };
  real a38 = 360.0;
  real a39 = a29 + a38;
  a29 = a39;
  label3: ;
  pidSetError ( a11, a29 );
  bool a46 = false;
  real a43 = pidUpdate ( a11, a2, a46 );
  *a8 = a43;
  return;
}

void controllerCorrectRatePID ( real* a0, real a1, real a2, real a3, int* a4, int* a5, int* a6, PidObject* a7, PidObject* a8, PidObject* a9 ){
  pidSetDesired ( a7, a1 );
  int a17 = 0;
  real a18 = a0[ a17 ];
  bool a19 = true;
  real a14 = pidUpdate ( a7, a18, a19 );
  int a13 = (int) floor( a14 );
  int a21 = 32768;
  int a22 = -a21;
  if ( a13 >= a22 ) { goto label4; };
  int a24 = 32768;
  int a25 = -a24;
  *a4 = a25;
  goto label5;
  label4: ;
  int a27 = 32768;
  if ( a13 <= a27 ) { goto label6; };
  int a29 = 32768;
  *a4 = a29;
  goto label5;
  label6: ;
  *a4 = a13;
  label5: ;
  pidSetDesired ( a8, a2 );
  int a38 = 1;
  real a39 = a0[ a38 ];
  real a40 = -a39;
  bool a41 = true;
  real a35 = pidUpdate ( a8, a40, a41 );
  int a34 = (int) floor( a35 );
  a13 = a34;
  int a43 = 32768;
  int a44 = -a43;
  if ( a34 >= a44 ) { goto label7; };
  int a46 = 32768;
  int a47 = -a46;
  *a5 = a47;
  goto label8;
  label7: ;
  int a49 = 32768;
  if ( a34 <= a49 ) { goto label9; };
  int a51 = 32768;
  *a5 = a51;
  goto label8;
  label9: ;
  *a5 = a13;
  label8: ;
  pidSetDesired ( a9, a3 );
  int a60 = 2;
  real a61 = a0[ a60 ];
  bool a62 = true;
  real a57 = pidUpdate ( a9, a61, a62 );
  int a56 = (int) floor( a57 );
  a13 = a56;
  int a64 = 32768;
  int a65 = -a64;
  if ( a56 >= a65 ) { goto label10; };
  int a67 = 32768;
  int a68 = -a67;
  *a6 = a68;
  goto label11;
  label10: ;
  int a70 = 32768;
  if ( a56 <= a70 ) { goto label12; };
  int a72 = 32768;
  *a6 = a72;
  goto label11;
  label12: ;
  *a6 = a13;
  label11: ;
  return;
}

void controllerResetAllPID ( PidObject** a0 ){
  int a2 = 0;
  PidObject *a3 = a0[ a2 ];
  pidReset ( a3 );
  int a5 = 1;
  PidObject *a6 = a0[ a5 ];
  pidReset ( a6 );
  int a8 = 2;
  PidObject *a9 = a0[ a8 ];
  pidReset ( a9 );
  int a11 = 3;
  PidObject *a12 = a0[ a11 ];
  pidReset ( a12 );
  int a14 = 4;
  PidObject *a15 = a0[ a14 ];
  pidReset ( a15 );
  int a17 = 5;
  PidObject *a18 = a0[ a17 ];
  pidReset ( a18 );
  return;
}

