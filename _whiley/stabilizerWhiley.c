#define LIBRARY_TESTING false

#include <stdio.h>
#include <stdbool.h>

#define STRINGMAX 10 // used in snprint functions
#define real float // can be changed to suit application
#include "mattCompiler.h"
#include "mattCompiler_library.c"
#include "cf_Lib.c"
#include "stm32f10x_conf.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "led.h"
#include "motors.h"
#include "task.h"
#include "system.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "log.h"

bool stabilizerTest ( void );
void stabilizerInit ( void );
void x1x_stabilizerTask ( void );
void x1x_distributePower ( int, int, int, int );
int x1x_limitThrust ( int );

bool stabilizerTest (void){
  bool a1 = true;
  bool a3 = true;
  if ( a1 == a3 ) { goto label0; };
  goto label1;
  label0: ;
  bool a4 = motorsTest (  );
  bool a5 = true;
  if ( a4 == a5 ) { goto label2; };
  label1: ;
  bool a6 = false;
  goto label3;
  label2: ;
  a6 = true;
  label3: ;
  bool a8 = true;
  if ( a6 == a8 ) { goto label4; };
  goto label5;
  label4: ;
  bool a9 = imu6Test (  );
  bool a10 = true;
  if ( a9 == a10 ) { goto label6; };
  label5: ;
  bool a11 = false;
  goto label7;
  label6: ;
  a11 = true;
  label7: ;
  bool a13 = true;
  if ( a11 == a13 ) { goto label8; };
  goto label9;
  label8: ;
  bool a14 = sensfusion6Test (  );
  bool a15 = true;
  if ( a14 == a15 ) { goto label10; };
  label9: ;
  bool a16 = false;
  goto label11;
  label10: ;
  a16 = true;
  label11: ;
  bool a18 = true;
  if ( a16 == a18 ) { goto label12; };
  goto label13;
  label12: ;
  bool a19 = controllerTest (  );
  bool a20 = true;
  if ( a19 == a20 ) { goto label14; };
  label13: ;
  bool a21 = false;
  goto label15;
  label14: ;
  a21 = true;
  label15: ;
  return a21;
}

void stabilizerInit (void){
  bool a0 = isStabilizerInit (  );
  bool a1 = true;
  if ( a0 == a1 ) { goto label16; };
  goto label17;
  label16: ;
  return;
  label17: ;
  motorsInit (  );
  imu6Init (  );
  sensfusion6Init (  );
  controllerInit (  );
  void (*a2)() = &x1x_stabilizerTask;
  char * a3 = "STABILIZER";
  int a4 = 200;
  int a5 = 0;
  int a6 = 2;
  int a7 = 0;
  cf_lib_xTaskCreate ( a2, a3, a4, a5, a6, a7 );
  return;
}

void x1x_stabilizerTask (void){
  real a1 = 0.0;
  real a2 = 0.0;
  real a3 = 0.0;
  real a4[3];
  a4[0] = a1;
  a4[1] = a2;
  a4[2] = a3;
  real *a5 = &(a4[0]);
  real a7 = 0.0;
  real a8 = 0.0;
  real a9 = 0.0;
  real a10[3];
  a10[0] = a7;
  a10[1] = a8;
  a10[2] = a9;
  real *a11 = &(a10[0]);
  real a13 = 0.0;
  real a14 = 0.0;
  real a15 = 0.0;
  real a16[3];
  a16[0] = a13;
  a16[1] = a14;
  a16[2] = a15;
  real *a17 = &(a16[0]);
  real a19 = 0.0;
  real *a20 = &a19;
  real a22 = 0.0;
  real *a23 = &a22;
  real a25 = 0.0;
  real *a26 = &a25;
  real a28 = 0.0;
  real *a29 = &a28;
  real a31 = 0.0;
  real *a32 = &a31;
  real a34 = 0.0;
  real *a35 = &a34;
  real a37 = 0.0;
  real *a38 = &a37;
  real a40 = 0.0;
  real *a41 = &a40;
  real a43 = 0.0;
  real *a44 = &a43;
  char * a46 = "ANGLE";
  char *a47 = a46;
  char * a49 = "ANGLE";
  char *a50 = a49;
  char * a52 = "ANGLE";
  char *a53 = a52;
  int a55 = 0;
  int *a56 = &a55;
  int a58 = 0;
  int *a59 = &a58;
  int a61 = 0;
  int *a62 = &a61;
  int a64 = 0;
  int *a65 = &a64;
  int a67 = 0;
  int a69 = 0;
  int a70 = 3;
  cf_lib_vTaskSetApplicationTaskTag ( a69, a70 );
  systemWaitStart (  );
  int a71 = cf_lib_xTaskGetTickCount (  );
  loop_start_label18: ;
  goto label19;
  label19: ;
  int a74 = 2;
  int a72 = cf_lib_vTaskDelayUntil ( a71, a74 );
  a71 = a72;
  cf_lib_imu9Read ( a5, a11, a17 );
  bool a78 = imu6IsCalibrated (  );
  bool a79 = true;
  if ( a78 == a79 ) { goto label20; };
  goto label21;
  label20: ;
  commanderGetRPY ( a29, a32, a35 );
  cf_lib_commanderGetRPYType ( a47, a50, a53 );
  int a87 = 1;
  int a88 = a67 + a87;
  a67 = a88;
  int a90 = 2;
  if ( a88 < a90 ) { goto label22; };
  real a92 = 1.0;
  real a93 = 500.0;
  real a94 = 2.0;
  real a95 = a93 / a94;
  real a96 = a92 / a95;
  cf_lib_sensfusion6UpdateQ ( a5, a11, a96 );
  sensfusion6GetEulerRPY ( a20, a23, a26 );
  real a104 = *a20;
  real a106 = *a23;
  real a108 = *a26;
  real a110 = *a29;
  real a112 = *a32;
  real a114 = *a35;
  controllerCorrectAttitudePID ( a104, a106, a108, a110, a112, a114, a38, a41, a44 );
  int a118 = 0;
  a67 = a118;
  label22: ;
  cf_lib_LHS_Equals_Neg_RHS ( a44, a35 );
  cf_lib_controllerCorrectRatePID ( a5, a38, a41, a44 );
  cf_lib_controllerGetActuatorOutput ( a59, a62, a65 );
  cf_lib_commanderGetThrust ( a56 );
  int a130 = *a56;
  int a131 = 0;
  if ( a130 <= a131 ) { goto label23; };
  int a133 = *a56;
  int a135 = *a59;
  int a137 = *a62;
  int a139 = *a65;
  int a140 = -a139;
  x1x_distributePower ( a133, a135, a137, a140 );
  goto label21;
  label23: ;
  int a141 = 0;
  int a142 = 0;
  int a143 = 0;
  int a144 = 0;
  x1x_distributePower ( a141, a142, a143, a144 );
  controllerResetAllPID (  );
  label21: ;
  goto loop_start_label18;
  
  return;
}

void x1x_distributePower ( int a0, int a1, int a2, int a3 ){
  int a8 = a0 + a2;
  int a10 = a8 + a3;
  int a5 = x1x_limitThrust ( a10 );
  int a15 = a0 - a1;
  int a17 = a15 - a3;
  int a12 = x1x_limitThrust ( a17 );
  int a22 = a0 - a2;
  int a24 = a22 + a3;
  int a19 = x1x_limitThrust ( a24 );
  int a29 = a0 + a1;
  int a31 = a29 - a3;
  int a26 = x1x_limitThrust ( a31 );
  int a32 = 0;
  cf_lib_motorsSetRatio ( a32, a5 );
  int a34 = 1;
  cf_lib_motorsSetRatio ( a34, a12 );
  int a36 = 2;
  cf_lib_motorsSetRatio ( a36, a19 );
  int a38 = 3;
  cf_lib_motorsSetRatio ( a38, a26 );
  return;
}

int x1x_limitThrust ( int a0 ){
  int a4 = 65535;
  if ( a0 <= a4 ) { goto label24; };
  a0 = a4;
  goto label25;
  label24: ;
  int a9 = 0;
  if ( a0 >= a9 ) { goto label25; };
  int a10 = 0;
  a0 = a10;
  label25: ;
  return a0;
}

