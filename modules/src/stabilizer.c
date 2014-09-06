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
#include "_whiley/mattCompiler.h"
#include "_whiley/mattCompiler_library.c"
#include "_whiley/cf_Lib.c"

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

bool stabilizerTest ( void );
void stabilizerInit ( void );
void stabilizerTask ( void );
void distributePower ( int, int, int, int );
int limitThrust ( int );

void controllerInit ( PidObject** ); 
bool controllerTest ( void ); 
void controllerCorrectRatePID ( real*, real, real, real, int*, int*, int*, PidObject*, PidObject*, PidObject* ); 
void controllerCorrectAttitudePID ( real, real, real, real, real, real, real*, real*, real*, PidObject*, PidObject*, PidObject* ); 
void controllerResetAllPID ( PidObject** ); 


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
  void (*a2)() = &stabilizerTask;
  char * a3 = "STABILIZER";
  int a4 = 200;
  int a5 = 0;
  int a6 = 2;
  int a7 = 0;
  cf_lib_xTaskCreate ( a2, a3, a4, a5, a6, a7 );
  return;
}

void stabilizerTask (void){
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
  real a70 = 0.0;
  real a71 = 0.0;
  real a72 = 0.0;
  real a73 = 0.0;
  real a74 = 0.0;
  real a75 = 0.0;
  real a76 = 0.0;
  real a77 = 0.0;
  real a78 = 0.0;
  real a79 = 0.0;
  real a80 = 0.0;
  real a81 = 0.0;
  real a82 = 0.0;
  real a83 = 0.0;
  PidObject a84 = { a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83 };
  PidObject *a85 = &a84;
  real a87 = 0.0;
  real a88 = 0.0;
  real a89 = 0.0;
  real a90 = 0.0;
  real a91 = 0.0;
  real a92 = 0.0;
  real a93 = 0.0;
  real a94 = 0.0;
  real a95 = 0.0;
  real a96 = 0.0;
  real a97 = 0.0;
  real a98 = 0.0;
  real a99 = 0.0;
  real a100 = 0.0;
  PidObject a101 = { a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100 };
  PidObject *a102 = &a101;
  real a104 = 0.0;
  real a105 = 0.0;
  real a106 = 0.0;
  real a107 = 0.0;
  real a108 = 0.0;
  real a109 = 0.0;
  real a110 = 0.0;
  real a111 = 0.0;
  real a112 = 0.0;
  real a113 = 0.0;
  real a114 = 0.0;
  real a115 = 0.0;
  real a116 = 0.0;
  real a117 = 0.0;
  PidObject a118 = { a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117 };
  PidObject *a119 = &a118;
  real a121 = 0.0;
  real a122 = 0.0;
  real a123 = 0.0;
  real a124 = 0.0;
  real a125 = 0.0;
  real a126 = 0.0;
  real a127 = 0.0;
  real a128 = 0.0;
  real a129 = 0.0;
  real a130 = 0.0;
  real a131 = 0.0;
  real a132 = 0.0;
  real a133 = 0.0;
  real a134 = 0.0;
  PidObject a135 = { a121, a122, a123, a124, a125, a126, a127, a128, a129, a130, a131, a132, a133, a134 };
  PidObject *a136 = &a135;
  real a138 = 0.0;
  real a139 = 0.0;
  real a140 = 0.0;
  real a141 = 0.0;
  real a142 = 0.0;
  real a143 = 0.0;
  real a144 = 0.0;
  real a145 = 0.0;
  real a146 = 0.0;
  real a147 = 0.0;
  real a148 = 0.0;
  real a149 = 0.0;
  real a150 = 0.0;
  real a151 = 0.0;
  PidObject a152 = { a138, a139, a140, a141, a142, a143, a144, a145, a146, a147, a148, a149, a150, a151 };
  PidObject *a153 = &a152;
  real a155 = 0.0;
  real a156 = 0.0;
  real a157 = 0.0;
  real a158 = 0.0;
  real a159 = 0.0;
  real a160 = 0.0;
  real a161 = 0.0;
  real a162 = 0.0;
  real a163 = 0.0;
  real a164 = 0.0;
  real a165 = 0.0;
  real a166 = 0.0;
  real a167 = 0.0;
  real a168 = 0.0;
  PidObject a169 = { a155, a156, a157, a158, a159, a160, a161, a162, a163, a164, a165, a166, a167, a168 };
  PidObject *a170 = &a169;
  PidObject *a178[6];
  a178[0] = a85;
  a178[1] = a102;
  a178[2] = a119;
  a178[3] = a136;
  a178[4] = a153;
  a178[5] = a170;
  controllerInit ( a178 );
  int a180 = 0;
  int a181 = 3;
  cf_lib_vTaskSetApplicationTaskTag ( a180, a181 );
  systemWaitStart (  );
  int a182 = cf_lib_xTaskGetTickCount (  );
  loop_start_label18: ;
  goto label19;
  label19: ;
  int a185 = 2;
  int a183 = cf_lib_vTaskDelayUntil ( a182, a185 );
  a182 = a183;
  cf_lib_imu9Read ( a5, a11, a17 );
  bool a189 = imu6IsCalibrated (  );
  bool a190 = true;
  if ( a189 == a190 ) { goto label20; };
  goto label21;
  label20: ;
  commanderGetRPY ( a29, a32, a35 );
  cf_lib_commanderGetRPYType ( a47, a50, a53 );
  int a198 = 1;
  int a199 = a67 + a198;
  a67 = a199;
  int a201 = 2;
  if ( a199 < a201 ) { goto label22; };
  real a203 = 1.0;
  real a204 = 500.0;
  real a205 = 2.0;
  real a206 = a204 / a205;
  real a207 = a203 / a206;
  cf_lib_sensfusion6UpdateQ ( a5, a11, a207 );
  sensfusion6GetEulerRPY ( a20, a23, a26 );
  real a215 = *a20;
  real a217 = *a23;
  real a219 = *a26;
  real a221 = *a29;
  real a223 = *a32;
  real a225 = *a35;
  real a226 = -a225;
  controllerCorrectAttitudePID ( a215, a217, a219, a221, a223, a226, a38, a41, a44, a136, a153, a170 );
  int a233 = 0;
  a67 = a233;
  label22: ;
  cf_lib_LHS_Equals_Neg_RHS ( a44, a35 );
  real * a237 = a5;
  real a239 = *a38;
  real a241 = *a41;
  real a243 = *a44;
  controllerCorrectRatePID ( a237, a239, a241, a243, a59, a62, a65, a85, a102, a119 );
  cf_lib_commanderGetThrust ( a56 );
  int a252 = *a56;
  int a253 = 0;
  if ( a252 <= a253 ) { goto label23; };
  int a255 = *a56;
  int a257 = *a59;
  int a259 = *a62;
  int a261 = *a65;
  int a262 = -a261;
  distributePower ( a255, a257, a259, a262 );
  goto label21;
  label23: ;
  int a263 = 0;
  int a264 = 0;
  int a265 = 0;
  int a266 = 0;
  distributePower ( a263, a264, a265, a266 );
  controllerResetAllPID ( a178 );
  label21: ;
  goto loop_start_label18;
  
  return;
}

void distributePower ( int a0, int a1, int a2, int a3 ){
  int a8 = a0 + a2;
  int a10 = a8 + a3;
  int a5 = limitThrust ( a10 );
  int a15 = a0 - a1;
  int a17 = a15 - a3;
  int a12 = limitThrust ( a17 );
  int a22 = a0 - a2;
  int a24 = a22 + a3;
  int a19 = limitThrust ( a24 );
  int a29 = a0 + a1;
  int a31 = a29 - a3;
  int a26 = limitThrust ( a31 );
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

int limitThrust ( int a0 ){
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

