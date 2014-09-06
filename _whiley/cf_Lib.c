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


#ifndef CF_LIB_
#define CF_LIB_

typedef void(*func)();





// motor test, used for print line type debugging. Yes, painfull...
void mattTest(int i){
  if(i == 0){
    motorsTestTask(0); //##############################
  }
}


/* == Initialized? ==
 * called to utilise global variable
 */
static bool stabilizerIsInit = false;

bool isStabilizerInit(){
  bool temp = stabilizerIsInit;
  stabilizerIsInit = true;
  return temp;
}

static bool controllerIsInit = false;
bool getControllerIsInit(){ return controllerIsInit; }
bool isControllerInit(){
  bool temp = controllerIsInit;
  controllerIsInit = true;
  return temp;
}

/**** NOTES - IMPORTANT ****
 * naming pattern; cf_lib_ plus original name
 * simple solution pattern to this interface; first call to update globals, then specify the getters for Whiley
 */

static void cf_lib_LHS_Equals_Neg_RHS( float* yawRateDesired, float* eulerYawDesired){
		*yawRateDesired = -(*eulerYawDesired);
}

/* =============================================
 * == FreeRTOS ==
 */

/* portTickType xTaskGetTickCount( void ) PRIVILEGED_FUNCTION; */
int cf_lib_xTaskGetTickCount(){
  return (int) xTaskGetTickCount();
}

/* xTaskCreate(&stabilizerTask, *//*(const signed char * const)*//* "STABILIZER", 200, null, *//*Piority*//*2, null)
   portmacro.h #define portBASE_TYPE	long
*/
void cf_lib_xTaskCreate(func stabilizerTask, char* stabilizerName, int depth, int n1, int priority, int n2){
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
static int g_xTask;
static pdTASK_HOOK_CODE g_taskStabilizerIdNmr;
void cf_lib_vTaskSetApplicationTaskTag(int xTask, int taskStabilizerIdNmr){
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
static portTickType lastWakeTime;
int cf_lib_vTaskDelayUntil( int _lastWakeTime, int xTimeIncrement ){
  lastWakeTime = (portTickType) _lastWakeTime;

  vTaskDelayUntil( &lastWakeTime, (unsigned int) xTimeIncrement );

  return (int) lastWakeTime;
}

/* =====================================================
 * == Firmware ==
 */

/** commanderGetThrust **/
static uint16_t thrust;
// needed to convert int to uint16_t
void cf_lib_commanderGetThrust(int* _thrust){
  thrust = (uint16_t) *_thrust;

  commanderGetThrust(&thrust);
  
  *_thrust = (int) thrust;
}

/** commanderGetRPYType **/
/* declared in FreeRTOS commander.h
typedef enum
{
  RATE,
  ANGLE
} RPYType; */
RPYType rollType;
RPYType pitchType;
int getRPYEnum(char* s){
  if(strcmp(s, "RATE") == 0) return 0;
  if(strcmp(s, "ANGLE") == 0) return 1;
  return 99;
}
RPYType yawType;
char* getRPYString(int x){
  if(x == 0) return "RATE";
  if(x == 1) return "ANGLE";
  return "ERROR";
}
// needed to translate between Whiley string and Crazyflie C enums
void cf_lib_commanderGetRPYType(char* _rollType, char* _pitchType, char* _yawType){
  rollType  = getRPYEnum(_rollType);
  pitchType = getRPYEnum(_pitchType);
  yawType   = getRPYEnum(_yawType);

  commanderGetRPYType(&rollType, &pitchType, &yawType);

  _rollType = getRPYString(rollType);
  _pitchType = getRPYString(pitchType);
  _yawType = getRPYString(yawType);
}


/* controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw) */
/*int16_t actuatorRoll;
int16_t actuatorPitch;
int16_t actuatorYaw;
// needed to cast from Whiley unbounded ints to Crazyflie C shorts, alias int16_t
void cf_lib_controllerGetActuatorOutput(int* _actuatorRoll, int* _actuatorPitch, int* _actuatorYaw){
  actuatorRoll  = (int16_t) *_actuatorRoll;
  actuatorPitch = (int16_t) *_actuatorPitch;
  actuatorYaw   = (int16_t) *_actuatorYaw;

  controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

  *_actuatorRoll  = (int) actuatorRoll;
  *_actuatorPitch = (int) actuatorPitch;
  *_actuatorYaw   = (int) actuatorYaw;
}*/

/* void controllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired); */
/*void cf_lib_controllerCorrectRatePID(
		float gyro[], float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired){
	controllerCorrectRatePID(
	       gyro[0], -(gyro[1]), gyro[2],
	       *rollRateDesired, *pitchRateDesired, *yawRateDesired);
}*/

/* ========================================================
 * == i/o ==
 */

/** imu9Read **/
static Axis3f gyro; // Gyro axis data in degrees
static Axis3f acc;  // Accelerometer axis data in mG 
static Axis3f mag;  // Magnetometer axis data in testla
// translater method 
// needed to translate between Whiley arrays and  Crazyflie C structs
void array_to_axis3f(float* array, Axis3f axis){
  axis.x = array[0];
  axis.y = array[1];
  axis.z = array[2];
}
void axis3f_to_array(Axis3f axis, float* array){
	array[0] = axis.x;
	array[1] = axis.y;
	array[2] = axis.z;
}
void cf_lib_imu9Read(float* _gyro, float* _acc, float* _mag){
  array_to_axis3f(_gyro, gyro);
  array_to_axis3f(_acc,  acc);
  array_to_axis3f(_mag,  mag);

  imu9Read(&gyro, &acc, &mag);

	axis3f_to_array(gyro, _gyro);
  axis3f_to_array( acc, _acc);
  axis3f_to_array( mag, _mag);
}

/* void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt); */
static void cf_lib_sensfusion6UpdateQ(float gyro[], float acc[], float dt){
	sensfusion6UpdateQ( gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], dt);
}

/* void motorsSetRatio(int id, uint16_t ratio); */
static void cf_lib_motorsSetRatio(int motor, int power){
	motorsSetRatio(motor, (uint16_t) power);
}

//===========================================================================================
//========== pid.c ===========
//============================


//  void pidInit(PidObject* pid, const float desired, const float kp,
//             const float ki, const float kd, const float dt);
//void PidObject pidInit( PidObject pid, real a1, real a2, real a3, real a4, real a5){


























#endif

