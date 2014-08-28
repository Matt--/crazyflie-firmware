/*
 * Library for interfacing Whiley methods with Crazyflie methods
 *
 * Solving two problems.
 * 1. Allowing a global variable, to let callers of stabilizerInit(void) to return immediately if the method has already been run.
 * 2. Whiley does not allow pointers as parameters in methods. Values are passed and returned back to Whiley methods.
 */

#include "led.h"

/* declared in FreeRTOS commander.h
typedef enum
{
  RATE,
  ANGLE
} RPYType; */

/* declared in FreeRTOC hal/interface/imu_types.h
typedef struct {
        float x;
        float y;
        float z;
} Axis3f; */

/* declared in FreeRTOS include/projdefs.h
typedef void (*pdTASK_CODE)( void * ); */

typedef void(*func)();



/* == Initialized? ==
 * called to utilise global variable
 */
static bool stabilizerIsInit = false;

static bool isStabilizerInit(){
  bool temp = stabilizerIsInit;
  stabilizerIsInit = true;
  return temp;
}


/**** NOTES - IMPORTANT ****
 * naming pattern; cf_lib_ plus original name
 * simple solution pattern to this interface; first call to update globals, then specify the getters for Whiley
 */

/* =============================================
 * == FreeRTOS ==
 */

/* portTickType xTaskGetTickCount( void ) PRIVILEGED_FUNCTION; */
static int cf_lib_xTaskGetTickCount(){
  return (int) xTaskGetTickCount();
}

/* xTaskCreate(&stabilizerTask, *//*(const signed char * const)*//* "STABILIZER", 200, null, *//*Piority*//*2, null)
   portmacro.h #define portBASE_TYPE	long
*/
static void cf_lib_xTaskCreate(func stabilizerTask, char* stabilizerName, int depth, int n1, int priority, int n2){
/*  pdTASK_CODE pvTaskCode = (pdTASK_CODE) stabilizerTask; 
  const signed char * const pcName = (signed char*) stabilizerName; 
  unsigned short usStackDepth = depth; 
  void *pvParameters = (void*) n1; 
  unsigned long uxPriority = priority; 
  void *pvCreatedTask = (void*) n2; 
*/
  xTaskCreate( (pdTASK_CODE) stabilizerTask /* method pointer */
			, (signed char*) stabilizerName /* "STABILIZER" */
			, depth /* 200 */
			, (void*) n1 /* null */
			, priority /* 2 */
			, (void*) n2 ); /* null */
}

/* vTaskSetApplicationTaskTag(0, (void*)*//*TASK_STABILIZER_ID_NBR*//*3) // TODO not sure how this (void*)3 is meant to work...
   FreeRTOSConfig.h #define TASK_STABILIZER_ID_NBR  3
   vTaskSetApplicationTaskTag( xTaskHandle xTask, pdTASK_HOOK_CODE pxHookFunction ) PRIVILEGED_FUNCTION;
   typedef void * xTaskHandle;
   typedef portBASE_TYPE (*pdTASK_HOOK_CODE)( void * );
*/
/*static*/ int g_xTask;
/*static*/ pdTASK_HOOK_CODE g_taskStabilizerIdNmr;
static void cf_lib_vTaskSetApplicationTaskTag(int xTask, int taskStabilizerIdNmr){
  g_xTask = xTask;
  g_taskStabilizerIdNmr = (pdTASK_HOOK_CODE) taskStabilizerIdNmr;
  vTaskSetApplicationTaskTag(&g_xTask, g_taskStabilizerIdNmr);
}


/*void vTaskDelayUntil( portTickType * const pxPreviousWakeTime, portTickType xTimeIncrement ) PRIVILEGED_FUNCTION; */
//== portTickType defined in portable/GCC/ARM_CM3/portmacro.h
//#if( configUSE_16_BIT_TICKS == 1 )
//	typedef unsigned portSHORT portTickType;
//	#define portMAX_DELAY ( portTickType ) 0xffff
//#else
//	typedef unsigned portLONG portTickType;
//	#define portMAX_DELAY ( portTickType ) 0xffffffff
//#endif
/*static*/ portTickType lastWakeTime;
static void cf_lib_vTaskDelayUntil( int _lastWakeTime, int xTimeIncrement ){
  lastWakeTime = (portTickType) _lastWakeTime;

  vTaskDelayUntil( &lastWakeTime, xTimeIncrement );
}

/* =====================================================
 * == Firmware ==
 */

/** commanderGetThrust **/
/*static*/ uint16_t thrust;
static int cf_lib_commanderGetThrust(int _thrust){
  thrust = (uint16_t) _thrust;
  commanderGetThrust(&thrust);
  return (int) thrust;
}

/** commanderGetRPY **/
/*static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static void cf_lib_commanderGetRPY(float _eulerRollDesired, float _eulerPitchDesired, float _eulerYawDesired){
  eulerRollDesired  = _eulerRollDesired;
  eulerPitchDesired = _eulerPitchDesired;
  eulerYawDesired   = _eulerYawDesired;
  commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
}*/
/* gcc complains, not used
static Any cf_lib_getEulerRollDesired() { return Real(eulerRollDesired);}
static Any cf_lib_getEulerPitchDesired(){ return Real(eulerPitchDesired);}
*/
//static float cf_lib_getEulerYawDesired()  { return eulerYawDesired;}

/** commanderGetRPYType **/
//static RPYType rollType;
//static RPYType pitchType;
//static RPYType yawType;
/* need to accept a Str() type and return a Str() type */
RPYType rollType;
RPYType pitchType;
RPYType yawType;
static void cf_lib_commanderGetRPYType(char* _rollType, char* _pitchType, char* _yawType){
  rollType  = strcmp(_rollType, "ANGLE") == 0 ? 0 :
		        strcmp(_rollType, "RATE") == 0 ? 1 : 99;
  pitchType = strcmp(_pitchType, "ANGLE") == 0 ? 0 :
		        strcmp(_pitchType, "RATE") == 0 ? 1 : 99;
  yawType   = strcmp(_yawType, "ANGLE") == 0 ? 0 :
		        strcmp(_yawType, "RATE") == 0 ? 1 : 99;
  commanderGetRPYType(&rollType, &pitchType, &yawType);
}
char * cf_lib_getRollType() { char *a = rollType == 0 ? "ANGLE" : rollType == 2 ? "RATE" : ""; return a;}
char * cf_lib_getPitchType(){ char *a = pitchType == 0 ? "ANGLE" : pitchType == 2 ? "RATE" : ""; return a;}
char * cf_lib_getYawType()  { char *a = yawType == 0 ? "ANGLE" : yawType == 2 ? "RATE" : ""; return a;}


/** controllerCorrectAttitudePID **/

/*static float rollRateDesired = 0;
static float pitchRateDesired = 0;
static float yawRateDesired = 0;
static void cf_lib_controllerCorrectAttitudePID(
       float eulerRollActual,  float eulerPitchActual,  float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float _rollRateDesired,  float _pitchRateDesired,  float _yawRateDesired){

  rollRateDesired   = _rollRateDesired;
  pitchRateDesired  = _pitchRateDesired;
  yawRateDesired    = _yawRateDesired;

  controllerCorrectAttitudePID(
       eulerRollActual,  eulerPitchActual,  eulerYawActual,
       eulerRollDesired, eulerPitchDesired, eulerYawDesired,
       &rollRateDesired, &pitchRateDesired, &yawRateDesired);
}
static float cf_lib_getRollRateDesired() { return rollRateDesired;}
static float cf_lib_getPitchRateDesired(){ return pitchRateDesired;}
*/

/* controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw) */
//static short actuatorRoll;
//static short actuatorPitch;
//static short actuatorYaw;
/*short actuatorRoll;
short actuatorPitch;
short actuatorYaw;
static void cf_lib_controllerGetActuatorOutput(int _actuatorRoll, int _actuatorPitch, int _actuatorYaw){
  actuatorRoll  = (short) _actuatorRoll;
  actuatorPitch = (short) _actuatorPitch;
  actuatorYaw   = (short) _actuatorYaw;
  controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);
}
static int cf_lib_getActuatorRoll() { return (int) actuatorRoll;}
static int cf_lib_getActuatorPitch(){ return (int) actuatorPitch;}
static int cf_lib_getActuatorYaw()  { return (int) actuatorYaw;}
*/
/* ========================================================
 * == i/o ==
 */

/** imu9Read **/
static Axis3f gyro; // Gyro axis data in degrees
static Axis3f acc;  // Accelerometer axis data in mG 
static Axis3f mag;  // Magnetometer axis data in testla
static float gyroArray[3];
static float accArray[3];
static float magArray[3];
// translater method 
void axis3f_to_array(Axis3f axis, float* array){
	array[0] = axis.x;
	array[1] = axis.y;
	array[2] = axis.z;
}
static void cf_lib_imu9Read(float* _gyro, float* _acc, float* _mag){
  gyro.x = _gyro[0]; gyro.y = _gyro[1]; gyro.z = _gyro[2];
  acc.x  = _acc[0];  acc.y  = _acc[1];  acc.z  = _acc[2];
  mag.x  = _mag[0];  mag.y  = _mag[1];  mag.z  = _mag[2];
  imu9Read(&gyro, &acc, &mag);
}
static float* cf_lib_getGyro(){ axis3f_to_array(gyro, gyroArray); return gyroArray;}
static float* cf_lib_getAcc() { axis3f_to_array( acc,  accArray); return  accArray;}
static float* cf_lib_getMag() { axis3f_to_array( mag,  magArray); return  magArray;}



/*static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;

static void cf_lib_sensfusion6GetEulerRPY(float _eulerRollActual, float _eulerPitchActual, float _eulerYawActual){
  eulerRollActual  = _eulerRollActual;
  eulerPitchActual = _eulerPitchActual;
  eulerYawActual   = _eulerYawActual;
  sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
}
static float cf_lib_getEulerRollActual() { return eulerRollActual;}
static float cf_lib_getEulerPitchActual(){ return eulerPitchActual;}
static float cf_lib_getEulerYawActual()  { return eulerYawActual;}
*/
/* void motorsSetRatio(int id, uint16_t ratio); */
static void cf_lib_motorsSetRatio(int motor, int power){
	motorsSetRatio(motor, (uint16_t) power);
}

