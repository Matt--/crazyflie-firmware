/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#include "_whiley/mattCompiler.h"
#include "_whiley/mattCompiler_library.c"
#include "_whiley/cf_Lib.c"

bool stabilizerTest ( void );
void stabilizerInit ( void );
void x1x_stabilizerTask ( void );
void x1x_distributePower ( Any , Any , Any , Any  );
Any x1x_limitThrust ( Any  );

#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "math.h"

#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "ledseq.h"
#include "param.h"
#include "ms5611.h"

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;

// Baro variables
static float temperature; // temp from barometer
static float pressure;    // pressure from barometer
static float asl;     // smoothed asl
static float aslRaw;  // raw asl
static float aslLong; // long term asl

// Altitude hold variables
static PidObject altHoldPID; // Used for altitute hold mode. I gets reset when the bat status changes
bool altHold = false;          // Currently in altitude hold mode
bool setAltHold = false;      // Hover mode has just been activated
static float accWZ     = 0.0;
static float accMAG    = 0.0;
static float vSpeedASL = 0.0;
static float vSpeedAcc = 0.0;
static float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float altHoldPIDVal;                    // Output of the PID controller
static float altHoldErr;                       // Different between target and current altitude

// Altitude hold & Baro Params
static float altHoldKp              = 0.5;  // PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.18;
static float altHoldKd              = 0.0;
static float altHoldChange          = 0;     // Change in target altitude
static float altHoldTarget          = -1;    // Target altitude
static float altHoldErrMax          = 1.0;   // max cap on current estimated altitude vs target altitude in meters
static float altHoldChange_SENS     = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float pidAslFac              = 13000; // relates meters asl to thrust
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float vSpeedASLFac           = 0;    // multiplier
static float vSpeedAccFac           = -48;  // multiplier
static float vAccDeadband           = 0.05;  // Vertical acceleration deadband
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vSpeedLimit            = 0.05;  // used to constrain vertical velocity
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vBiasAlpha             = 0.91; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float aslAlpha               = 0.92; // Short term smoothing
static float aslAlphaLong           = 0.93; // Long term smoothing
static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static uint16_t altHoldMaxThrust    = 60000; // max altitude hold thrust


RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

static bool isInit;

static void stabilizerAltHoldUpdate(void);
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);


void stabilizerInit (void){
  
  systemInit();

  Any a0 = Bool(isStabilizerInit (  ));
  Any a1 = Bool(true);
  if ( dataAsInt( a0 ) == dataAsInt( a1 ) ) { goto label16; };
  goto label17;
  label16: ;
  label17: ;
  motorsInit (  );
  imu6Init (  );
  sensfusion6Init (  );
  controllerInit (  );
  Any a2 = Fptr( &x1x_stabilizerTask, 0 );
  Any a3 = Str("STABILIZER");
  Any a4 = Int(200);
  Any a5 = Int(0);
  Any a6 = Int(2);
  Any a7 = Int(0);
  cf_lib_xTaskCreate ( a2, a3.s, a4.i, a5.i, a6.i, a7.i );
}
/*
void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit();
  imu6Init();
  sensfusion6Init();
  controllerInit();

  rollRateDesired = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;

  xTaskCreate(stabilizerTask, (const signed char * const)"STABILIZER",
              2*configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  isInit = TRUE;
}
*/
bool stabilizerTest (void){

  Any a1 = Bool(true);
  Any a3 = Bool(true);
  if ( dataAsInt( a1 ) == dataAsInt( a3 ) ) { goto label0; };
  goto label1;
  label0: ;

  ledSetGreen(1);

  Any a4 = Bool(motorsTest (  ));
//  Any a4 = Bool(1);

  Any a5 = Bool(true);
  if ( dataAsInt( a4 ) == dataAsInt( a5 ) ) { goto label2; };
  label1: ;
  Any a6 = Bool(false);
  goto label3;
  label2: ;
  a6 = Bool(true);
  label3: ;
  Any a8 = Bool(true);
  if ( dataAsInt( a6 ) == dataAsInt( a8 ) ) { goto label4; };
  goto label5;
  label4: ;
  Any a9 = Bool(imu6Test (  ));
  Any a10 = Bool(true);
  if ( dataAsInt( a9 ) == dataAsInt( a10 ) ) { goto label6; };
  label5: ;
  Any a11 = Bool(false);
  goto label7;
  label6: ;
  a11 = Bool(true);
  label7: ;
  Any a13 = Bool(true);
  if ( dataAsInt( a11 ) == dataAsInt( a13 ) ) { goto label8; };
  goto label9;
  label8: ;
  Any a14 = Bool(sensfusion6Test (  ));
  Any a15 = Bool(true);
  if ( dataAsInt( a14 ) == dataAsInt( a15 ) ) { goto label10; };
  label9: ;
  Any a16 = Bool(false);
  goto label11;
  label10: ;
  a16 = Bool(true);
  label11: ;
  Any a18 = Bool(true);
  if ( dataAsInt( a16 ) == dataAsInt( a18 ) ) { goto label12; };
  goto label13;
  label12: ;
  Any a19 = Bool(controllerTest (  ));
  Any a20 = Bool(true);
  if ( dataAsInt( a19 ) == dataAsInt( a20 ) ) { goto label14; };
  label13: ;
  Any a21 = Bool(false);
  goto label15;
  label14: ;
  a21 = Bool(true);
  label15: ;

  return a21.b;
}
/*
bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}
*/

void x1x_stabilizerTask (void){


  Any a1 = Int(0);
  Any a2 = Int(0);
  Any a3 = Int(0);
  Any a4[] = {a1, a2, a3};
  Any *a0 = a4;
  Any a6 = Int(0);
  Any a7 = Int(0);
  Any a8 = Int(0);
  Any a9[] = {a6, a7, a8};
  Any *a5 = a9;
  Any a11 = Int(0);
  Any a12 = Int(0);
  Any a13 = Int(0);
  Any a14[] = {a11, a12, a13};
  Any *a10 = a14;
  Any a16 = Int(0);
  Any a15 = a16;
  Any a18 = Int(0);
  Any a17 = a18;
  Any a20 = Int(0);
  Any a19 = a20;
  Any a22 = Int(0);
  Any a21 = a22;
  Any a24 = Int(0);
  Any a23 = a24;
  Any a26 = Int(0);
  Any a25 = a26;
  Any a28 = Int(0);
  Any a27 = a28;
  Any a30 = Int(0);
  Any a29 = a30;
  Any a32 = Int(0);
  Any a31 = a32;
  Any a34 = Str("ANGLE");
  Any a33 = a34;
  Any a36 = Str("ANGLE");
  Any a35 = a36;
  Any a38 = Str("ANGLE");
  Any a37 = a38;
  Any a40 = Int(0);
  Any a39 = a40;
  Any a42 = Int(0);
  Any a41 = a42;
  Any a44 = Int(0);
  Any a43 = a44;
  Any a46 = Int(0);
  Any a45 = a46;
  Any a48 = Int(0);
  Any a47 = a48;
  Any a50 = Int(0);
  Any a51 = Int(3);
  cf_lib_vTaskSetApplicationTaskTag ( a50.i, a51.i );

  systemWaitStart (  );


  Any a52 = Int(cf_lib_xTaskGetTickCount (  ));
  Any a49 = a52;



  loop_start_label18: ;
  goto label19;
  label19: ;

ledSetRed(1);

  Any a53 = a49;
  Any a54 = Int(2);
  cf_lib_vTaskDelayUntil ( a53.i, a54.i );


  Any *a55 = a0;
  Any *a56 = a5;
  Any *a57 = a10;
//  cf_lib_imu9Read ( a55, a56, a57 );


  Any *a58 = cf_lib_getGyro (  );
  a0 = a58;
  Any *a59 = cf_lib_getAcc (  );
  a5 = a59;
  Any *a60 = cf_lib_getMag (  );
  a10 = a60;
  Any a61 = Bool(imu6IsCalibrated (  ));
  Any a62 = Bool(true);



  if ( dataAsInt( a61 ) == dataAsInt( a62 ) ) { goto label20; };
  goto label21;
  label20: ;
  Any a63 = a21;
  a63.type = REAL_TYPE;
  Any a9999 = Int(a63.i);
  a63.r = (double) a9999.i;
  Any a64 = a23;
  a64.type = REAL_TYPE;
  a9999 = Int(a64.i);
  a64.r = (double) a9999.i;
  Any a65 = a25;




  cf_lib_commanderGetRPY ( a63.r, a64.r, a65.r );
  Any a66 = cf_lib_getEulerYawDesired (  );
  a25 = a66;
  Any a67 = a33;
  Any a68 = a35;
  Any a69 = a37;


  cf_lib_commanderGetRPYType ( a67.s, a68.s, a69.s );


  Any a70 = cf_lib_getRollType (  );


  a33 = a70;

  Any a71 = cf_lib_getPitchType (  );


  a35 = a71;
  Any a72 = cf_lib_getYawType (  );



  a37 = a72;
  Any a74 = Int(1);
  Any a75 = wyce_add( a47 , a74);
  a47 = a75;
  Any a77 = Int(2);
  if ( dataAsInt( a75 ) < dataAsInt( a77 ) ) { goto label22; };
  Any a79 = Real(1.0);
  Any a80 = Real(500.0);
  Any a81 = Real(2.0);
  Any a156 = Real(0);
  Any a82 = wyce_div( a80 , a81);
  a156 = Real(0);
  Any a83 = wyce_div( a79 , a82);
  Any a78 = a83;
  Any a85 = Int(0);
  a156 = Int(0);
  Any a157 = Int( sizeof( a0 ) / sizeof( a0[0] ) );
  Any a86 = a0[a85.i];
  Any a88 = Int(1);
  a156 = Int(0);
  a157 = Int( sizeof( a0 ) / sizeof( a0[0] ) );
  Any a89 = a0[a88.i];
  Any a91 = Int(2);
  a156 = Int(0);
  a157 = Int( sizeof( a0 ) / sizeof( a0[0] ) );
  Any a92 = a0[a91.i];
  Any a94 = Int(0);
  a156 = Int(0);
  a157 = Int( sizeof( a5 ) / sizeof( a5[0] ) );
  Any a95 = a5[a94.i];
  Any a97 = Int(1);
  a156 = Int(0);
  a157 = Int( sizeof( a5 ) / sizeof( a5[0] ) );
  Any a98 = a5[a97.i];
  Any a100 = Int(2);
  a156 = Int(0);
  a157 = Int( sizeof( a5 ) / sizeof( a5[0] ) );
  Any a101 = a5[a100.i];
  Any a102 = a78;
  cf_lib_sensfusion6UpdateQ ( a86.r, a89.r, a92.r, a95.r, a98.r, a101.r, a102.r );
  Any a103 = a15;
  Any a104 = a17;
  Any a105 = a19;




  cf_lib_sensfusion6GetEulerRPY ( a103.r, a104.r, a105.r );
  Any a106 = cf_lib_getEulerRollActual (  );
  a15 = a106;
  Any a107 = cf_lib_getEulerPitchActual (  );
  a17 = a107;
  Any a108 = cf_lib_getEulerYawActual (  );
  a19 = a108;
  Any a109 = a15;
  Any a110 = a17;
  Any a111 = a19;
  Any a112 = a21;
  a112.type = REAL_TYPE;
  a9999 = Int(a112.i);
  a112.r = (double) a9999.i;
  Any a113 = a23;
  a113.type = REAL_TYPE;
  a9999 = Int(a113.i);
  a113.r = (double) a9999.i;
  Any a115 = wyce_neg(a66);
  Any a116 = a27;
  Any a117 = a29;
  Any a118 = a31;
  cf_lib_controllerCorrectAttitudePID ( a109.r, a110.r, a111.r, a112.r, a113.r, a115.r, a116.r, a117.r, a118.r );
  Any a119 = cf_lib_getRollRateDesired (  );
  a27 = a119;
  Any a120 = cf_lib_getPitchRateDesired (  );
  a29 = a120;
  Any a121 = Int(0);
  a47 = a121;
  label22: ;
  Any a123 = wyce_neg(a66);
  a31 = a123;
  Any a125 = Int(0);
  a156 = Int(0);
  a157 = Int( sizeof( a0 ) / sizeof( a0[0] ) );
  Any a126 = a0[a125.i];
  Any a128 = Int(1);
  a156 = Int(0);
  a157 = Int( sizeof( a0 ) / sizeof( a0[0] ) );
  Any a129 = a0[a128.i];
  Any a130 = wyce_neg(a129);
  Any a132 = Int(2);
  a156 = Int(0);
  a157 = Int( sizeof( a0 ) / sizeof( a0[0] ) );
  Any a133 = a0[a132.i];
  Any a134 = a27;
  Any a135 = a29;
  Any a136 = a31;



  cf_lib_controllerCorrectRatePID ( a126.r, a130.r, a133.r, a134.r, a135.r, a136.r );
  Any a137 = a41;
  Any a138 = a43;
  Any a139 = a45;
  cf_lib_controllerGetActuatorOutput ( a137.i, a138.i, a139.i );
  Any a140 = Int(cf_lib_getActuatorRoll (  ));
  a41 = a140;
  Any a141 = Int(cf_lib_getActuatorPitch (  ));
  a43 = a141;
  Any a142 = Int(cf_lib_getActuatorYaw (  ));
  a45 = a142;
  Any a143 = Int(cf_lib_commanderGetThrust ( a39.i ));
  a39 = a143;
  Any a146 = Int(0);
  if ( dataAsInt( a143 ) <= dataAsInt( a146 ) ) { goto label23; };
  Any a147 = a39;
  Any a148 = a41;
  Any a149 = a43;
  Any a151 = wyce_neg(a142);
  distributePower ( (uint16_t) a147.i, (uint16_t) a148.i, (uint16_t) a149.i, (uint16_t) a151.i );
  goto label21;
  label23: ;
  Any a152 = Int(0);
  Any a153 = Int(0);
  Any a154 = Int(0);
  Any a155 = Int(0);
  distributePower ( (uint16_t) a152.i, (uint16_t) a153.i, (uint16_t) a154.i, (uint16_t) a155.i );

  controllerResetAllPID (  );

  label21: ;
  goto loop_start_label18;
  label18: ;
}
/*
void x1x_distributePower ( Any a0, Any a1, Any a2, Any a3 ){
  Any a8 = wyce_add( a0 , a2);
  Any a10 = wyce_add( a8 , a3);
  Any a5 = x1x_limitThrust ( a10 );
  Any a4 = a5;
  Any a15 = wyce_sub( a0 , a1);
  Any a17 = wyce_sub( a15 , a3);
  Any a12 = x1x_limitThrust ( a17 );
  Any a11 = a12;
  Any a22 = wyce_sub( a0 , a2);
  Any a24 = wyce_add( a22 , a3);
  Any a19 = x1x_limitThrust ( a24 );
  Any a18 = a19;
  Any a29 = wyce_add( a0 , a1);
  Any a31 = wyce_sub( a29 , a3);
  Any a26 = x1x_limitThrust ( a31 );
  Any a25 = a26;
  Any a32 = Int(0);
  Any a33 = a4;
  cf_lib_motorsSetRatio ( a32.i, a33.i );
  Any a34 = Int(1);
  Any a35 = a11;
  cf_lib_motorsSetRatio ( a34.i, a35.i );
  Any a36 = Int(2);
  Any a37 = a18;
  cf_lib_motorsSetRatio ( a36.i, a37.i );
  Any a38 = Int(3);
  Any a39 = a25;
  cf_lib_motorsSetRatio ( a38.i, a39.i );
}

Any x1x_limitThrust ( Any a0 ){
  Any a2 = a0;
  Any a1 = a2;
  Any a4 = Int(65535);
  Any a3 = a4;
  if ( dataAsInt( a0 ) <= dataAsInt( a4 ) ) { goto label24; };
  Any a7 = a3;
  a1 = a7;
  goto label25;
  label24: ;
  Any a9 = Int(0);
  if ( dataAsInt( a0 ) >= dataAsInt( a9 ) ) { goto label25; };
  Any a10 = Int(0);
  a1 = a10;
  label25: ;
  return a1;
}
*/

/*
static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t altHoldCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    if (imu6IsCalibrated())
    {
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
      commanderGetRPYType(&rollType, &pitchType, &yawType);

      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

        accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
        accMAG = (acc.x*acc.x) + (acc.y*acc.y) + (acc.z*acc.z);
        // Estimate speed from acc (drifts)
        vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;

        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
        attitudeCounter = 0;
      }

      // 100HZ
      if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))
      {
        stabilizerAltHoldUpdate();
        altHoldCounter = 0;
      }

      if (rollType == RATE)
      {
        rollRateDesired = eulerRollDesired;
      }
      if (pitchType == RATE)
      {
        pitchRateDesired = eulerPitchDesired;
      }
      if (yawType == RATE)
      {
        yawRateDesired = -eulerYawDesired;
      }

      // TODO: Investigate possibility to subtract gyro drift.
      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                               rollRateDesired, pitchRateDesired, yawRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

      if (!altHold || !imuHasBarometer())
      {
        // Use thrust from controller if not in altitude hold mode
        commanderGetThrust(&actuatorThrust);
      }
      else
      {
        // Added so thrust can be set to 0 while in altitude hold mode after disconnect
        commanderWatchdog();
      }

      if (actuatorThrust > 0)
      {
#if defined(TUNE_ROLL)
        distributePower(actuatorThrust, actuatorRoll, 0, 0);
#elif defined(TUNE_PITCH)
        distributePower(actuatorThrust, 0, actuatorPitch, 0);
#elif defined(TUNE_YAW)
        distributePower(actuatorThrust, 0, 0, -actuatorYaw);
#else
        distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
#endif
      }
      else
      {
        distributePower(0, 0, 0, 0);
        controllerResetAllPID();
      }
    }
  }
}

static void stabilizerAltHoldUpdate(void)
{
  // Get altitude hold commands from pilot
  commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);

  // Get barometer height estimates
  //TODO do the smoothing within getData
  ms5611GetData(&pressure, &temperature, &aslRaw);
  asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
  aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);

  // Estimate vertical speed based on successive barometer readings. This is ugly :)
  vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband);

  // Estimate vertical speed based on Acc - fused with baro to reduce drift
  vSpeed = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);
  vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);
  vSpeedAcc = vSpeed;

  // Reset Integral gain of PID controller if being charged
  if (!pmIsDischarging())
  {
    altHoldPID.integ = 0.0;
  }

  // Altitude hold mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
  if (setAltHold)
  {
    // Set to current altitude
    altHoldTarget = asl;

    // Cache last integral term for reuse after pid init
    const float pre_integral = altHoldPID.integ;

    // Reset PID controller
    pidInit(&altHoldPID, asl, altHoldKp, altHoldKi, altHoldKd,
            ALTHOLD_UPDATE_DT);
    // TODO set low and high limits depending on voltage
    // TODO for now just use previous I value and manually set limits for whole voltage range
    //                    pidSetIntegralLimit(&altHoldPID, 12345);
    //                    pidSetIntegralLimitLow(&altHoldPID, 12345);              /

    altHoldPID.integ = pre_integral;

    // Reset altHoldPID
    altHoldPIDVal = pidUpdate(&altHoldPID, asl, false);
  }

  // In altitude hold mode
  if (altHold)
  {
    // Update target altitude from joy controller input
    altHoldTarget += altHoldChange / altHoldChange_SENS;
    pidSetDesired(&altHoldPID, altHoldTarget);

    // Compute error (current - target), limit the error
    altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
                           -altHoldErrMax, altHoldErrMax);
    pidSetError(&altHoldPID, -altHoldErr);

    // Get control from PID controller, dont update the error (done above)
    // Smooth it and include barometer vspeed
    // TODO same as smoothing the error??
    altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((vSpeedAcc * vSpeedAccFac) +
                    (vSpeedASL * vSpeedASLFac) + pidUpdate(&altHoldPID, asl, false));

    // compute new thrust
    actuatorThrust =  max(altHoldMinThrust, min(altHoldMaxThrust,
                          limitThrust( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));

    // i part should compensate for voltage drop

  }
  else
  {
    altHoldTarget = 0.0;
    altHoldErr = 0.0;
    altHoldPIDVal = 0.0;
  }
}

*/
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
  roll = roll >> 1;
  pitch = pitch >> 1;
  motorPowerM1 = limitThrust( (int32_t) (thrust - roll + pitch + yaw));
  motorPowerM2 = limitThrust( (int32_t) (thrust - roll - pitch - yaw));
  motorPowerM3 =  limitThrust( (int32_t) (thrust + roll - pitch + yaw));
  motorPowerM4 =  limitThrust( (int32_t) (thrust + roll + pitch - yaw));
#else // QUAD_FORMATION_NORMAL
  motorPowerM1 = limitThrust( (int32_t) (thrust + pitch + yaw));
  motorPowerM2 = limitThrust( (int32_t) (thrust - roll - yaw));
  motorPowerM3 =  limitThrust( (int32_t) (thrust - pitch + yaw));
  motorPowerM4 =  limitThrust( (int32_t) (thrust + roll - yaw));
#endif

  motorsSetRatio(MOTOR_M1, motorPowerM1);
  motorsSetRatio(MOTOR_M2, motorPowerM2);
  motorsSetRatio(MOTOR_M3, motorPowerM3);
  motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}


// Constrain value between min and max
static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

// Deadzone
static float deadband(float value, const float threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}
/*
LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_ADD(LOG_FLOAT, mag2, &accMAG)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

// LOG altitude hold PID controller states
LOG_GROUP_START(vpid)
LOG_ADD(LOG_FLOAT, pid, &altHoldPID)
LOG_ADD(LOG_FLOAT, p, &altHoldPID.outP)
LOG_ADD(LOG_FLOAT, i, &altHoldPID.outI)
LOG_ADD(LOG_FLOAT, d, &altHoldPID.outD)
LOG_GROUP_STOP(vpid)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, err, &altHoldErr)
LOG_ADD(LOG_FLOAT, target, &altHoldTarget)
LOG_ADD(LOG_FLOAT, zSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
LOG_GROUP_STOP(altHold)

// Params for altitude hold
PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
PARAM_ADD(PARAM_FLOAT, altHoldChangeSens, &altHoldChange_SENS)
PARAM_ADD(PARAM_FLOAT, altHoldErrMax, &altHoldErrMax)
PARAM_ADD(PARAM_FLOAT, kd, &altHoldKd)
PARAM_ADD(PARAM_FLOAT, ki, &altHoldKi)
PARAM_ADD(PARAM_FLOAT, kp, &altHoldKp)
PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
PARAM_ADD(PARAM_UINT16, baseThrust, &altHoldBaseThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &altHoldMaxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &altHoldMinThrust)
PARAM_GROUP_STOP(altHold)
*/
