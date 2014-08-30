#define LIBRARY_TESTING false

#include <stdio.h>
#include <stdbool.h>

#define STRINGMAX 10 // used in snprint functions
#define real float // can be changed to suit application
#include "mattCompiler.h"
#include "mattCompiler_library.c"
#include "cf_Lib.c"

bool stabilizerTest ( void );
void stabilizerInit ( void );
void x1x_stabilizerTask ( void );
void x1x_distributePower ( int , int , int , int  );
int x1x_limitThrust ( int  );

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
//  char * a3 = malloc(11 * sizeof(char));
  char a3[11];
  a3[0] = 'S';
  a3[1] = 'T';
  a3[2] = 'A';
  a3[3] = 'B';
  a3[4] = 'I';
  a3[5] = 'L';
  a3[6] = 'I';
  a3[7] = 'Z';
  a3[8] = 'E';
  a3[9] = 'R';
  a3[10] = '\0';
  int a4 = 200;
  int a5 = 0;
  int a6 = 2;
  int a7 = 0;
	  char s3[] = "STABILIZER";
    char* t3 = "STABILIZER";
  xTaskCreate ( (pdTASK_CODE) a2, 
                       (signed char *) a3, (unsigned short) a4, 
                       (void *) a5, (unsigned portBASE_TYPE) a6, (void *) a7 );
//  xTaskCreate ( a2, "STABILIZER", a4, a5, a6, a7 );
  return;
}

void x1x_stabilizerTask (void){
  real a1 = 0.0;
  real a2 = 0.0;
  real a3 = 0.0;
  Axis3f a4 = {a1, a2, a3};
//  real* a4 = malloc(3 * sizeof(real));
//  a4[0] = a1;
//  a4[1] = a2;
//  a4[2] = a3;
  real a6 = 0.0;
  real a7 = 0.0;
  real a8 = 0.0;
  Axis3f a9 = {a1, a2, a3};
//  real* a9 = malloc(3 * sizeof(real));
//  a9[0] = a6;
//  a9[1] = a7;
//  a9[2] = a8;
  real a11 = 0.0;
  real a12 = 0.0;
  real a13 = 0.0;
  Axis3f a14 = {a1, a2, a3};
//  real* a14 = malloc(3 * sizeof(real));
//  a14[0] = a11;
//  a14[1] = a12;
//  a14[2] = a13;
  real a16 = 0.0;
  real a18 = 0.0;
  real a20 = 0.0;
  real a22 = 0.0;
  real a24 = 0.0;
  real a26 = 0.0;
  real a28 = 0.0;
  real a30 = 0.0;
  real a32 = 0.0;
//  char * a34 = "ANGLE";
  char * a34 = pvPortMalloc(6 * sizeof(char));
//  a34[0] = 'A';
//  a34[1] = 'N';
//  a34[2] = 'G';
//  a34[3] = 'L';
//  a34[4] = 'E';
//  a34[5] = '\0';
  char * a36 = "ANGLE";
//  char * a36 = malloc(6 * sizeof(char));
//  a36[0] = 'A';
//  a36[1] = 'N';
//  a36[2] = 'G';
//  a36[3] = 'L';
//  a36[4] = 'E';
//  a36[5] = '\0';
  char * a38 = "ANGLE";
//  char * a38 = malloc(6 * sizeof(char));
//  a38[0] = 'A';
//  a38[1] = 'N';
//  a38[2] = 'G';
//  a38[3] = 'L';
//  a38[4] = 'E';
//  a38[5] = '\0';
  int a40 = 0;
  int a42 = 0;
  int a44 = 0;
  int a46 = 0;
  int a48 = 0;
  int a50 = 0;
  int a51 = 3;
  cf_lib_vTaskSetApplicationTaskTag ( a50, a51 );
  systemWaitStart (  );
  int a52 = cf_lib_xTaskGetTickCount (  );
  loop_start_label18: ;
  goto label19;
  label19: ;
  int a54 = 2;
  cf_lib_vTaskDelayUntil ( a52, a54 );
  imu9Read ( &a4, &a9, &a14 );                 // addresses
  //  real * a58 = cf_lib_getGyro (  );
  //  a4 = a58;
  //  real * a59 = cf_lib_getAcc (  );
  //  a9 = a59;
  //  real * a60 = cf_lib_getMag (  );
  //  a14 = a60;
  bool a61 = imu6IsCalibrated (  );
  bool a62 = true;
  if ( a61 == a62 ) { goto label20; };
  goto label21;
  label20: ;
  commanderGetRPY ( &a22, &a24, &a26 );             // addresses
  // real a66 = cf_lib_getEulerYawDesired (  );
  //  a26 = a66;
  commanderGetRPYType ( a34, a36, a38 );        // addresses
  //  char * a70 = cf_lib_getRollType (  );
  //  a34 = a70;
  //  char * a71 = cf_lib_getPitchType (  );
  //  a36 = a71;
  //  char * a72 = cf_lib_getYawType (  );
  //  a38 = a72;
  int a74 = 1;
  int a75 = a48 + a74;
  a48 = a75;
  int a77 = 2;
  if ( a75 < a77 ) { goto label22; };
  real a79 = 1.0;
  real a80 = 500.0;
  real a81 = 2.0;
  real a82 = a80 / a81;
  real a83 = a79 / a82;
  int a85 = 0;
  real a86 = a4.x;
  int a88 = 1;
  real a89 = a4.y;
  int a91 = 2;
  real a92 = a4.z;
  int a94 = 0;
  real a95 = a9.x;
  int a97 = 1;
  real a98 = a9.y;
  int a100 = 2;
  real a101 = a9.z;
  sensfusion6UpdateQ ( a86, a89, a92, a95, a98, a101, a83 );
  sensfusion6GetEulerRPY ( &a16, &a18, &a20 );                 // addresses
  //  real a106 = cf_lib_getEulerRollActual (  );
  //  a16 = a106;
  //  real a107 = cf_lib_getEulerPitchActual (  );
  //  a18 = a107;
  //  real a108 = cf_lib_getEulerYawActual (  );
  //  a20 = a108;
  real a115 = -a26; // a26 was a66
  controllerCorrectAttitudePID ( a16, a18, a20, a22, a24, a115, &a28, &a30, &a32 ); // addresses
  //  real a119 = cf_lib_getRollRateDesired (  );
  //  a28 = a119;
  //  real a120 = cf_lib_getPitchRateDesired (  );
  //  a30 = a120;
  int a121 = 0;
  a48 = a121;
  label22: ;
  real a123 = -a26; // a26 was a66
  a32 = a123;
  int a125 = 0;
  real a126 = a4.x;
  int a128 = 1;
  real a129 = a4.y;
  real a130 = -a129;
  int a132 = 2;
  real a133 = a4.z;
  controllerCorrectRatePID ( a126, a130, a133, a28, a30, a32 );
  controllerGetActuatorOutput ( &a42, &a44, &a46 );                         // addresses
  //  int a140 = cf_lib_getActuatorRoll (  );
  //  a42 = a140;
  //  int a141 = cf_lib_getActuatorPitch (  );
  //  a44 = a141;
  //  int a142 = cf_lib_getActuatorYaw (  );
  //  a46 = a142;
  int a143 = cf_lib_commanderGetThrust ( a40 );
  a40 = a143;
  int a146 = 0;
  if ( a143 <= a146 ) { goto label23; };
  int a151 = -a46; // a46 was a142
  x1x_distributePower ( a40, a42, a44, a151 );
  goto label21;
  label23: ;
  int a152 = 0;
  int a153 = 0;
  int a154 = 0;
  int a155 = 0;
  x1x_distributePower ( a152, a153, a154, a155 );
  controllerResetAllPID (  );
  label21: ;
  goto loop_start_label18;
//  label18: ;
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

