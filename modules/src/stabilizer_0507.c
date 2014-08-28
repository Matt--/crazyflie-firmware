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
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"


/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz



uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

static bool isInit;

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit();
  imu6Init();
  sensfusion6Init();
  controllerInit();

  xTaskCreate(stabilizerTask, (const signed char * const)"STABILIZER",
              /*2*configMINIMAL_STACK_SIZE*/200, NULL, /*Piority*/2, NULL);

  isInit = TRUE;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}

static void stabilizerTask(void* param)
{
  static Axis3f gyro; // Gyro axis data in deg/s
  static Axis3f acc;  // Accelerometer axis data in mG
  static Axis3f mag;  // Magnetometer axis data in testla

  static float eulerRollActual;
  static float eulerPitchActual;
  static float eulerYawActual;
  static float eulerRollDesired;
  static float eulerPitchDesired;
  static float eulerYawDesired;
  static float rollRateDesired = 0;
  static float pitchRateDesired = 0;
  static float yawRateDesired = 0;

  RPYType rollType;
  RPYType pitchType;
  RPYType yawType;

  uint16_t actuatorThrust;
  int16_t  actuatorRoll;
  int16_t  actuatorPitch;
  int16_t  actuatorYaw;

  uint32_t attitudeCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)/*TASK_STABILIZER_ID_NBR*/3);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {

    vTaskDelayUntil(&lastWakeTime, (unsigned int)((/*configTICK_RATE_HZ*/ /*( portTickType ) cast to short*/ 1000 / /*IMU_UPDATE_FREQ*/500)) ); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    if (imu6IsCalibrated())
    {
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
      commanderGetRPYType(&rollType, &pitchType, &yawType);

      // 250HZ
      if (++attitudeCounter >= /*ATTITUDE_UPDATE_RATE_DIVIDER*/2)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, 
          /*FUSION_UPDATE_DT*/(float)(1.0/(/*IMU_UPDATE_FREQ*/500 / /*ATTITUDE_UPDATE_RATRE_DIVIDER*/2)) );
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);


        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
        attitudeCounter = 0;
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

      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                               rollRateDesired, pitchRateDesired, yawRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

      commanderGetThrust(&actuatorThrust);


      if (actuatorThrust > 0)
      {

        distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);

      }
      else
      {
        distributePower(0, 0, 0, 0);
        controllerResetAllPID();
      }
    }
  }
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{

 // QUAD_FORMATION_NORMAL
  motorPowerM1 = limitThrust(thrust + pitch + yaw);
  motorPowerM2 = limitThrust(thrust - roll - yaw);
  motorPowerM3 =  limitThrust(thrust - pitch + yaw);
  motorPowerM4 =  limitThrust(thrust + roll - yaw);

  motorsSetRatio(/*MOTOR_M1*/0, motorPowerM1);
  motorsSetRatio(/*MOTOR_M2*/1, motorPowerM2);
  motorsSetRatio(/*MOTOR_M3*/2, motorPowerM3);
  motorsSetRatio(/*MOTOR_M4*/3, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
  if(value > /*UINT16_MAX*/65535)
  {
    value = /*UINT16_MAX*/65535;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}
