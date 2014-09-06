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
void dereferenceEq ( int, int );
void dereferenceEqReal ( real, real );
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

void dereferenceEq ( int a0, int a1 ){
  return;
}

void dereferenceEqReal ( real a0, real a1 ){
  return;
}

void controllerCorrectAttitudePID ( real a0, real a1, real a2, real a3, real a4, real a5, real* a6, real* a7, real* a8, PidObject* a9, PidObject* a10, PidObject* a11 ){
  pidSetDesired ( a9, a3 );
  real a15 = *a6;
  bool a19 = true;
  real a16 = pidUpdate ( a9, a0, a19 );
               *a6 = a16;
  pidSetDesired ( a10, a4 );
  real a23 = *a7;
  bool a27 = true;
  real a24 = pidUpdate ( a10, a1, a27 );
              *a7 = a24;
  real a31 = a5 - a2;
  real a33 = 180.0;
  if ( a31 <= a33 ) { goto label2; };
  real a35 = 360.0;
  real a36 = a31 - a35;
  a31 = a36;
  goto label3;
  label2: ;
  real a38 = -180.0;
  if ( a31 >= a38 ) { goto label3; };
  real a40 = 360.0;
  real a41 = a31 + a40;
  a31 = a41;
  label3: ;
  pidSetError ( a11, a31 );
  real* a45 = a8;
  bool a49 = false;
  real a46 = pidUpdate ( a11, a2, a49 );
                 *a8 = a46;
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
  int a24 = *a4;
  int a25 = 32768;
  int a26 = -a25;
                     *a4 = a26;
  goto label5;
  label4: ;
  int a28 = 32768;
  if ( a13 <= a28 ) { goto label6; };
  int a30 = *a4;
  int a31 = 32768;
                     *a4 = a31;
  goto label5;
  label6: ;
  int a33 = *a4;
                    *a4 = a13;
  label5: ;
  pidSetDesired ( a8, a2 );
  int a41 = 1;
  real a42 = a0[ a41 ];
  real a43 = -a42;
  bool a44 = true;
  real a38 = pidUpdate ( a8, a43, a44 );
  int a37 = (int) floor( a38 );
  a13 = a37;
  int a46 = 32768;
  int a47 = -a46;
  if ( a37 >= a47 ) { goto label7; };
  int a49 = *a5;
  int a50 = 32768;
  int a51 = -a50;
                    *a5 = a51;
  goto label8;
  label7: ;
  int a53 = 32768;
  if ( a37 <= a53 ) { goto label9; };
  int a55 = *a5;
  int a56 = 32768;
                    *a5 = a56;
  goto label8;
  label9: ;
  int a58 = *a5;
                     *a5 = a13;
  label8: ;
  pidSetDesired ( a9, a3 );
  int a66 = 2;
  real a67 = a0[ a66 ];
  bool a68 = true;
  real a63 = pidUpdate ( a9, a67, a68 );
  int a62 = (int) floor( a63 );
  a13 = a62;
  int a70 = 32768;
  int a71 = -a70;
  if ( a62 >= a71 ) { goto label10; };
  int a73 = *a6;
  int a74 = 32768;
  int a75 = -a74;
                       *a6 = a75;
  goto label11;
  label10: ;
  int a77 = 32768;
  if ( a62 <= a77 ) { goto label12; };
  int a79 = *a6;
  int a80 = 32768;
                       *a6 = a80;
  goto label11;
  label12: ;
  int a82 = *a6;
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

