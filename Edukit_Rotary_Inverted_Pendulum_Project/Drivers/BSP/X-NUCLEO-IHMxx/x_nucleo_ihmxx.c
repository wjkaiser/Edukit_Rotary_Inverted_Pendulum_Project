 	/**
 ******************************************************************************
 * @file    x_nucleo_ihmxx.c
 * @author  IPC Rennes
 * @version V1.7.0
 * @date    March 16th, 2018
 * @brief   This file provides common functions for motor control 
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "x_nucleo_ihmxx.h"

/** @addtogroup BSP
 * @{
 */

/** @defgroup MOTOR_CONTROL MOTOR CONTROL
 * @{
 */

/** @defgroup MOTOR_CONTROL_Private_Types_Definitions MOTOR CONTROL Private Types Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup MOTOR_CONTROL_Private_Defines MOTOR CONTROL Private Defines
 * @{
 */

/**
 * @}
 */

/** @defgroup MOTOR_CONTROL_Private_Constants MOTOR CONTROL Private Constants
 * @{
 */

/**
 * @}
 */

/** @defgroup MOTOR_CONTROL_Private_Macros MOTOR CONTROL Private Macros
 * @{
 */
/// Error when trying to call undefined functions via motorDrvHandle
#define MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(errorNb)   (BSP_MotorControl_ErrorHandler(MOTOR_CONTROL_ERROR_TAG|(errorNb)))   

/**
 * @}
 */   
   
/** @defgroup MOTOR_CONTROL_Private_Variables MOTOR CONTROL Private Variables
 * @{
 */

static motorDrv_t *motorDrvHandle = 0;
static uint16_t MotorControlBoardId;

/**
 * @}
 */

/** @defgroup MOTOR_CONTROL_Weak_Private_Functions MOTOR CONTROL Weak Private Functions
 * @{
 */
/// Get motor handle for L6474
__weak motorDrv_t* L6474_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for L647x
__weak motorDrv_t* l647x_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for L648x
__weak motorDrv_t* l648x_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for Powerstep
__weak motorDrv_t* Powerstep01_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for L6206
__weak motorDrv_t* L6206_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for L6208
__weak motorDrv_t* L6208_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for STSPIN220
__weak motorDrv_t* Stspin220_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for STSPIN240
__weak motorDrv_t* Stspin240_250_GetMotorHandle(void){return ((motorDrv_t* )0);}
/**
 * @}
 */

/** @defgroup MOTOR_CONTROL_Private_Functions MOTOR CONTROL Private Functions
 * @{
 */

/******************************************************//**
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void BSP_MotorControl_AttachErrorHandler(void (*callback)(uint16_t))
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->AttachErrorHandler != 0))
  {
    motorDrvHandle->AttachErrorHandler(callback);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(2);
  }
}

/******************************************************//**
 * @brief  Attaches a user callback to the Flag interrupt Handler.
 * The call back will be then called each time the library 
 * detects a FLAG signal falling edge.
 * @param[in] callback Name of the callback to attach 
 * to the Flag interrupt Hanlder
 * @retval None
 **********************************************************/
void BSP_MotorControl_AttachFlagInterrupt(void (*callback)(void))
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->AttachFlagInterrupt != 0))
  {
    motorDrvHandle->AttachFlagInterrupt(callback);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(3);
  }  
}

/******************************************************//**
 * @brief  Attaches a user callback to the Busy interrupt Handler.
 * The call back will be then called each time the library 
 * detects a BUSY signal falling edge.
 * @param[in] callback Name of the callback to attach 
 * to the Busy interrupt Hanlder
 * @retval None
 **********************************************************/
void BSP_MotorControl_AttachBusyInterrupt(void (*callback)(void))
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->AttachBusyInterrupt != 0))
  {
    motorDrvHandle->AttachBusyInterrupt(callback);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(4);
  }  
}

/******************************************************//**
 * @brief Motor control error handler
 * @param[in] error number of the error
 * @retval None
 **********************************************************/
void BSP_MotorControl_ErrorHandler(uint16_t error)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->ErrorHandler != 0))
  {
    motorDrvHandle->ErrorHandler(error);
  }  
  else
  {
    while(1)
    {
      /* Infinite loop as Error handler must be defined*/
    }
  }
}
/******************************************************//**
 * @brief Initialises the motor driver. This function has to be called one time
 * for each device. The number of devices is incremented in the driver up to
 * the maximum allowed. Calling this function a number of times greater than the
 * maximum number triggers an error in the driver.
 * @param[in] id Component Id (L6474, Powerstep01,...)
 * @param[in] initDeviceParameters Initialization structure for one device
 * @retval None
 **********************************************************/
void BSP_MotorControl_Init(uint16_t id, void* initDeviceParameters)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->Init != 0))
  {
    motorDrvHandle->Init(initDeviceParameters);
  }  
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(0);
  }  
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @retval None
 **********************************************************/
void BSP_MotorControl_FlagInterruptHandler(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->FlagInterruptHandler != 0))
  {
    motorDrvHandle->FlagInterruptHandler();
  }    
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(5);
  }  
}
/******************************************************//**
 * @brief Returns the acceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval Acceleration in pps^2
 **********************************************************/
uint16_t BSP_MotorControl_GetAcceleration(uint8_t deviceId)
{                                                  
  uint16_t acceleration = 0;

  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetAcceleration != 0))
  {
    acceleration = motorDrvHandle->GetAcceleration(deviceId);
  }      
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(6);
  }  
  return(acceleration);    
}            

/******************************************************//**
 * @brief Get board Id  the motor driver
 * @retval Motor control board Id
 **********************************************************/
uint16_t BSP_MotorControl_GetBoardId(void)
{
  return (MotorControlBoardId);
}
/******************************************************//**
 * @brief Returns the current speed of the specified device
 * @param[in] deviceId from 0 to (MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval Speed in pps for stepper motor
 *               in % for Brush DC motor (0-100)   
 **********************************************************/
uint16_t BSP_MotorControl_GetCurrentSpeed(uint8_t deviceId)
{
  uint16_t currentSpeed = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetCurrentSpeed != 0))
  {
    currentSpeed = motorDrvHandle->GetCurrentSpeed(deviceId);
  }      
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(7);
  }  
  return(currentSpeed); 
}

/******************************************************//**
 * @brief Returns the deceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval Deceleration in pps^2
 **********************************************************/
uint16_t BSP_MotorControl_GetDeceleration(uint8_t deviceId)
{                                                  
  uint16_t deceleration = 0;

  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetDeceleration != 0))
  {
    deceleration = motorDrvHandle->GetDeceleration(deviceId);
  }      
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(8);
  }  
  return(deceleration);   
}          

/******************************************************//**
 * @brief Returns the device state
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval State ACCELERATING, DECELERATING, STEADY or INACTIVE for stepper motor,
                 STEADY or INACTIVE for Brush DC motor
 **********************************************************/
motorState_t BSP_MotorControl_GetDeviceState(uint8_t deviceId)
{
  motorState_t state = INACTIVE;

  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetDeviceState != 0))
  {
    state = motorDrvHandle->GetDeviceState(deviceId);
  }      
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(9);
  }  
  return(state);   
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval BSP_MotorControl_FW_VERSION
 * @note the format is (MAJOR_VERSION<<16)|(MINOR_VERSION<<8)|(PATCH_VERSION)
 * with major, minor and patch versions coded on 8 bits. 
 **********************************************************/
uint32_t BSP_MotorControl_GetFwVersion(void)
{
  uint32_t version = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetFwVersion != 0))
  {
    version = motorDrvHandle->GetFwVersion();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(10);
  }  
  return(version);
}

/******************************************************//**
 * @brief  Returns the mark position  of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t BSP_MotorControl_GetMark(uint8_t deviceId)
{
  int32_t mark = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetMark != 0))
  {
    mark = motorDrvHandle->GetMark(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(11);
  }  
  return(mark);
}

/******************************************************//**
 * @brief  Returns the max speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval maxSpeed in pps for stepper motor
 *                  in % for Brush DC motor (0-100)
 **********************************************************/
uint16_t BSP_MotorControl_GetMaxSpeed(uint8_t deviceId)
{                                                  
  uint16_t maxSpeed = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetMaxSpeed != 0))
  {
    maxSpeed = motorDrvHandle->GetMaxSpeed(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(12);
  }    
  return(maxSpeed);
}

/******************************************************//**
 * @brief  Returns the min speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval minSpeed in pps
 **********************************************************/
uint16_t BSP_MotorControl_GetMinSpeed(uint8_t deviceId)
{                                                  
  uint16_t minSpeed = 0;

  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetMinSpeed != 0))
  {
    minSpeed = motorDrvHandle->GetMinSpeed(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(13);
  }    
  return(minSpeed);  
}                                                     

/******************************************************//**
 * @brief  Returns the ABS_POSITION of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval ABS_POSITION register value converted in a 32b signed integer
 **********************************************************/
int32_t BSP_MotorControl_GetPosition(uint8_t deviceId)
{
  int32_t pos = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetPosition != 0))
  {
    pos = motorDrvHandle->GetPosition(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(14);
  }      
  return(pos);
}

/******************************************************//**
 * @brief  Requests the motor to move to the home position (ABS_POSITION = 0)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_GoHome(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GoHome != 0))
  {
    motorDrvHandle->GoHome(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(15);
  }      
} 
  
/******************************************************//**
 * @brief  Requests the motor to move to the mark position 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_GoMark(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GoMark != 0))
  {
    motorDrvHandle->GoMark(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(16);
  }     
}

/******************************************************//**
 * @brief  Requests the motor to move to the specified position 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] targetPosition absolute position in steps
 * @retval None
 **********************************************************/
void BSP_MotorControl_GoTo(uint8_t deviceId, int32_t targetPosition)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GoTo != 0))
  {
    motorDrvHandle->GoTo(deviceId, targetPosition);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(17);
  }      
}

/******************************************************//**
 * @brief  Immediatly stops the motor.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval None
 **********************************************************/
void BSP_MotorControl_HardStop(uint8_t deviceId) 
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->HardStop != 0))
  {
    motorDrvHandle->HardStop(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(18);
  }      
}

/******************************************************//**
 * @brief  Moves the motor of the specified number of steps
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount Number of steps to perform
 * @retval None
 **********************************************************/
void BSP_MotorControl_Move(uint8_t deviceId, motorDir_t direction, uint32_t stepCount)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->Move != 0))
  {
    motorDrvHandle->Move(deviceId, direction, stepCount);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(19);
  }      
}

/******************************************************//**
 * @brief Resets all motor driver devices
 * @retval None
 **********************************************************/
void BSP_MotorControl_ResetAllDevices(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->ResetAllDevices != 0))
  {
    motorDrvHandle->ResetAllDevices(); 
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(20);
  }      
}

/******************************************************//**
 * @brief  Runs the motor. It will accelerate from the min 
 * speed up to the max speed by using the device acceleration.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 * @note For unidirectionnal brush DC motor, direction parameter 
 * has no effect
 **********************************************************/
void BSP_MotorControl_Run(uint8_t deviceId, motorDir_t direction)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->Run != 0))
  {
    motorDrvHandle->Run(deviceId, direction); 
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(21);
  }      
}
/******************************************************//**
 * @brief  Changes the acceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] newAcc New acceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool BSP_MotorControl_SetAcceleration(uint8_t deviceId,uint16_t newAcc)
{                                                  
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetAcceleration != 0))
  {
    status = motorDrvHandle->SetAcceleration(deviceId, newAcc);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(22);
  }      
  return (status);
}            

/******************************************************//**
 * @brief  Changes the deceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] newDec New deceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool BSP_MotorControl_SetDeceleration(uint8_t deviceId, uint16_t newDec)
{                                                  
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetDeceleration != 0))
  {
    status = motorDrvHandle->SetDeceleration(deviceId, newDec);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(23);
  }        
  return (status);
}        

/******************************************************//**
 * @brief  Set current position to be the Home position (ABS pos set to 0)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] homePosition new absolute home position 
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetHome(uint8_t deviceId, int32_t homePosition)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetHome != 0))
  {
    motorDrvHandle->SetHome(deviceId, homePosition);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(24);
  }        
}
 
/******************************************************//**
 * @brief  Sets current position to be the Mark position 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] markPosition new absolute mark position 
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetMark(uint8_t deviceId, int32_t markPosition)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetMark != 0))
  {
    motorDrvHandle->SetMark(deviceId, markPosition);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(25);
  }    
}

/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] newMaxSpeed New max speed  to apply in pps for stepper motor,
              in % for Brush DC motor (0-100)                           
 * @retval true if the command is successfully executed, else false
 * @note For a stepper motor, the command is not performed if the device 
 * is executing a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool BSP_MotorControl_SetMaxSpeed(uint8_t deviceId, uint16_t newMaxSpeed)
{                                                  
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetMaxSpeed != 0))
  {
    status = motorDrvHandle->SetMaxSpeed(deviceId, newMaxSpeed);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(26);
  }     
  return (status);  
}                                                     

/******************************************************//**
 * @brief  Changes the min speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] newMinSpeed New min speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool BSP_MotorControl_SetMinSpeed(uint8_t deviceId, uint16_t newMinSpeed)
{                                                  
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetMinSpeed != 0))
  {
    status = motorDrvHandle->SetMinSpeed(deviceId, newMinSpeed);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(27);
  }     
  
  return (status);  
}                 

/******************************************************//**
 * @brief  Stops the motor by using the device deceleration
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is in INACTIVE state.
 **********************************************************/
bool BSP_MotorControl_SoftStop(uint8_t deviceId)
{	
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SoftStop != 0))
  {
    status = motorDrvHandle->SoftStop(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(28);
  }    
  return (status);  
}

/******************************************************//**
 * @brief  Handles the device state machine at each step
 * or at each tick
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 * @note Must only be called by the timer ISR
 **********************************************************/
void BSP_MotorControl_StepClockHandler(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->StepClockHandler != 0))
  {
    motorDrvHandle->StepClockHandler(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(29);
  }   
}
/******************************************************//**
 * @brief  Locks until the device state becomes Inactive
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_WaitWhileActive(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->WaitWhileActive != 0))
  {
    motorDrvHandle->WaitWhileActive(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(30);
  }    
}

/**
  * @}
  */

/** @defgroup BSP_MotorControl_Control_Functions BSP MotorControl Control Functions
  * @{
  */   

/******************************************************//**
 * @brief  Issue the Disable command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval None
 * @note For brush DC motor, when input of different brigdes are parallelized 
 * together, the disabling of one bridge leads to the disabling
 * of the second one
 **********************************************************/
void BSP_MotorControl_CmdDisable(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdDisable != 0))
  {
    motorDrvHandle->CmdDisable(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(31);
  }    
}

/******************************************************//**
 * @brief  Issues the Enable command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval None
 * @note For brush DC motor, when input of different brigdes are parallelized 
 * together, the enabling of one bridge leads to the enabling
 * of the second one
 **********************************************************/
void BSP_MotorControl_CmdEnable(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdEnable != 0))
  {
    motorDrvHandle->CmdEnable(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(32);
  }      
}

/******************************************************//**
 * @brief  Issues the GetParam command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] param Register adress (BSP_MotorControl_ABS_POS, BSP_MotorControl_MARK,...)
 * @retval Register value
 **********************************************************/
uint32_t BSP_MotorControl_CmdGetParam(uint8_t deviceId,
                                      uint32_t param)
{
  uint32_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdGetParam != 0))
  {
    value = motorDrvHandle->CmdGetParam(deviceId, param);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(33);
  }       
  return (value);
}

/******************************************************//**
 * @brief  Issues the GetStatus command to the motor driver of the specified 
 * device for stepper motor,
 * Get bridge status for Brush DC motor
 * @param[in] deviceId  from 0 to MAX_NUMBER_OF_DEVICES - 1 for stepper motor,
              bridgeId from 0 for bridge A, 1 for bridge B for brush DC motor
 * @retval Status Register value for stepper motor,
 *         Bridge state for brush DC motor
 * @note For stepper motor, once the GetStatus command is performed, 
 * the flags of the status register are reset. 
 * This is not the case when the status register is read with the
 * GetParam command (via the functions ReadStatusRegister or CmdGetParam).
 **********************************************************/
uint16_t BSP_MotorControl_CmdGetStatus(uint8_t deviceId)
{
  uint16_t status = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdGetStatus != 0))
  {
    status = motorDrvHandle->CmdGetStatus(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(34);
  }      
  return (status);
}

/******************************************************//**
 * @brief  Issues the Nop command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdNop(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdNop != 0))
  {
    motorDrvHandle->CmdNop(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(35);
  }   
}

/******************************************************//**
 * @brief  Issues the SetParam command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] param Register adress (BSP_MotorControl_ABS_POS, BSP_MotorControl_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdSetParam(uint8_t deviceId,
                                   uint32_t param,
                                   uint32_t value)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdSetParam != 0))
  {
    motorDrvHandle->CmdSetParam(deviceId, param, value);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(36);
  }     
}

/******************************************************//**
 * @brief  Reads the Status Register value
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval Status register valued
 * @note The status register flags are not cleared 
 * at the difference with CmdGetStatus()
 **********************************************************/
uint16_t BSP_MotorControl_ReadStatusRegister(uint8_t deviceId)
{
  uint16_t status = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->ReadStatusRegister != 0))
  {
    status = motorDrvHandle->ReadStatusRegister(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(37);
  }   
  return (status);
}

/******************************************************//**
 * @brief  Releases the motor driver (pin set to High) of all devices
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 **********************************************************/
void BSP_MotorControl_ReleaseReset(uint8_t deviceId)
{ 
  if ((motorDrvHandle != 0)&&(motorDrvHandle->ReleaseReset != 0))
  {
    motorDrvHandle->ReleaseReset(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(38);
  }   
}

/******************************************************//**
 * @brief  Resets the motor driver (reset pin set to low) of all devices
 * @retval None
 **********************************************************/
void BSP_MotorControl_Reset(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->Reset != 0))
  {
    motorDrvHandle->Reset(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(39);
  }   
}

/******************************************************//**
 * @brief  Set the stepping mode 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] stepMode from full step to 1/16 microstep as specified in 
 * enum BSP_MotorControl_STEP_SEL_t
 * @retval true if the command is successfully executed, else false
 **********************************************************/
bool BSP_MotorControl_SelectStepMode(uint8_t deviceId, motorStepMode_t stepMode)
{
  bool value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SelectStepMode != 0))
  {
    value = motorDrvHandle->SelectStepMode(deviceId, stepMode);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(40);
  }
  return (value);
}

/******************************************************//**
 * @brief  Specifies the direction 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device 
 * is in INACTIVE state
 * For L6208: In velocity mode a direction change forces the device to stop and 
 * then run in the new direction. In position mode, if the device is 
 * running, a direction change will generate an error.
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetDirection(uint8_t deviceId, motorDir_t dir)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetDirection != 0))
  {
    motorDrvHandle->SetDirection(deviceId, dir);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(41);
  }     
}

/******************************************************//**
 * @brief Issues Go To Dir command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] dir movement direction
 * @param[in] abs_pos absolute position where requested to move
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdGoToDir(uint8_t deviceId, motorDir_t dir, int32_t abs_pos)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdGoToDir != 0))
  {
    motorDrvHandle->CmdGoToDir(deviceId, dir, abs_pos);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(42);
  }
}

/******************************************************//**
 * @brief Checks if at least one device is busy by checking 
 * busy pin position. 
 * The busy pin is shared between all devices.
 * @retval One if at least one device is busy, otherwise zero
 **********************************************************/
uint8_t BSP_MotorControl_CheckBusyHw(void)
{
  uint8_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CheckBusyHw != 0))
  {
    value = motorDrvHandle->CheckBusyHw();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(43);
  }
  return (value);
}

/******************************************************//**
 * @brief Checks if at least one device has an alarm flag set
 * by reading flag pin position.
 * The flag pin is shared between all devices.
 * @retval One if at least one device has an alarm flag set ,
 * otherwise zero
 **********************************************************/
uint8_t BSP_MotorControl_CheckStatusHw(void)
{
  uint8_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CheckStatusHw != 0))
  {
    value = motorDrvHandle->CheckStatusHw();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(44);
  }
  return (value);
}

/******************************************************//**
 * @brief Issues Go Until command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] action ACTION_RESET or ACTION_COPY
 * @param[in] dir movement direction
 * @param[in] speed in 2^-28 step/tick
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdGoUntil(uint8_t deviceId, motorAction_t action, motorDir_t dir, uint32_t speed)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdGoUntil != 0))
  {
    motorDrvHandle->CmdGoUntil(deviceId, action, dir, speed);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(45);
  }
}

/******************************************************//**
 * @brief Immediately stops the motor and disable the power bridge.
 * @param[in] deviceId from 0 to MAX_NUMBER_OF_DEVICES-1  for stepper motor
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval None
 * @note if two Brush DC motors use the same power bridge, the 
 * power bridge will be disable only if the two motors are
 * stopped
 **********************************************************/
void BSP_MotorControl_CmdHardHiZ(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdHardHiZ != 0))
  {
    motorDrvHandle->CmdHardHiZ(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(46);
  }
}

/******************************************************//**
 * @brief Issues Release SW command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] action
 * @param[in] dir movement direction
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdReleaseSw(uint8_t deviceId, motorAction_t action, motorDir_t dir)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdReleaseSw != 0))
  {
    motorDrvHandle->CmdReleaseSw(deviceId, action, dir);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(47);
  }
}

/******************************************************//**
 * @brief Issues Reset Device command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdResetDevice(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdResetDevice != 0))
  {
    motorDrvHandle->CmdResetDevice(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(48);
  }
}

/******************************************************//**
 * @brief Issues Reset Pos command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdResetPos(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdResetPos != 0))
  {
    motorDrvHandle->CmdResetPos(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(49);
  }
}

/******************************************************//**
 * @brief Issues Run command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] dir Movement direction (FORWARD, BACKWARD)
 * @param[in] speed in 2^-28 step/tick
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdRun(uint8_t deviceId, motorDir_t dir, uint32_t speed)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdRun != 0))
  {
    motorDrvHandle->CmdRun(deviceId, dir, speed);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(50);
  }
}

/******************************************************//**
 * @brief Issues Soft HiZ command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdSoftHiZ(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdSoftHiZ != 0))
  {
    motorDrvHandle->CmdSoftHiZ(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(51);
  }
}

/******************************************************//**
 * @brief Issues Step Clock command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] dir Movement direction (FORWARD, BACKWARD)
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdStepClock(uint8_t deviceId, motorDir_t dir)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdStepClock != 0))
  {
    motorDrvHandle->CmdStepClock(deviceId, dir);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(52);
  }
}

/******************************************************//**
 * @brief Fetch and clear status flags of all devices 
 * by issuing a GET_STATUS command simultaneously  
 * to all devices.
 * Then, the fetched status of each device can be retrieved
 * by using the BSP_MotorControl_GetFetchedStatus function
 * provided there is no other calls to functions which 
 * use the SPI in between.
 * @retval None
 **********************************************************/
void BSP_MotorControl_FetchAndClearAllStatus(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->FetchAndClearAllStatus != 0))
  {
    motorDrvHandle->FetchAndClearAllStatus();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(53);
  }
}

/******************************************************//**
 * @brief Get the value of the STATUS register which was 
 * fetched by using BSP_MotorControl_FetchAndClearAllStatus.
 * The fetched values are available as long as there is
 * no other calls to functions which use the SPI.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval Last fetched value of the STATUS register
 **********************************************************/
uint16_t BSP_MotorControl_GetFetchedStatus(uint8_t deviceId)
{
  uint16_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetFetchedStatus != 0))
  {
    value = motorDrvHandle->GetFetchedStatus(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(54);
  }
  return (value);
}

/******************************************************//**
 * @brief Return the number of devices in the daisy chain 
 * @retval number of devices from 1 to MAX_NUMBER_OF_DEVICES
 **********************************************************/
uint8_t BSP_MotorControl_GetNbDevices(void)
{
  uint8_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetNbDevices != 0))
  {
    value = motorDrvHandle->GetNbDevices();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(55);
  }
  return (value);
}

/******************************************************//**
 * @brief Checks if the specified device is busy
 * by reading the Busy bit of its status Register
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval true if device is busy, else false
 **********************************************************/
bool BSP_MotorControl_IsDeviceBusy(uint8_t deviceId)
{
  bool value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->IsDeviceBusy != 0))
  {
    value = motorDrvHandle->IsDeviceBusy(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(56);
  }
  return (value);
}

/******************************************************//**
 * @brief Sends commands stored in the queue by previously
 * BSP_MotorControl_QueueCommands
 * @retval None
 *********************************************************/
void BSP_MotorControl_SendQueuedCommands(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SendQueuedCommands != 0))
  {
    motorDrvHandle->SendQueuedCommands();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(57);
  }
}

/******************************************************//**
 * @brief Put commands in queue before synchronous sending
 * done by calling BSP_MotorControl_SendQueuedCommands.
 * Any call to functions that use the SPI between the calls of 
 * BSP_MotorControl_QueueCommands and BSP_MotorControl_SendQueuedCommands 
 * will corrupt the queue.
 * A command for each device of the daisy chain must be 
 * specified before calling BSP_MotorControl_SendQueuedCommands.
 * @param[in] deviceId deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] command Command to queue (all BSP_MotorControl commmands 
 * except SET_PARAM, GET_PARAM, GET_STATUS)
 * @param[in] value argument of the command to queue
 * @retval None
 *********************************************************/
void BSP_MotorControl_QueueCommands(uint8_t deviceId, uint8_t command, int32_t value)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->QueueCommands != 0))
  {
    motorDrvHandle->QueueCommands(deviceId, command, value);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(58);
  }
}

/******************************************************//**
 * @brief  Locks until all devices become not busy
 * @retval None
 **********************************************************/
void BSP_MotorControl_WaitForAllDevicesNotBusy(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->WaitForAllDevicesNotBusy != 0))
  {
    motorDrvHandle->WaitForAllDevicesNotBusy();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(59);
  }  
}

/******************************************************//**
 * @brief Handler of the busy interrupt which calls the user callback (if defined)
 * @retval None
 **********************************************************/
void BSP_MotorControl_BusyInterruptHandler(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->BusyInterruptHandler != 0))
  {
    motorDrvHandle->BusyInterruptHandler();
  }    
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(61);
  }  
}

/******************************************************//**
 * @brief Issues Soft Stop command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdSoftStop(uint8_t deviceId)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdSoftStop != 0))
  {
    motorDrvHandle->CmdSoftStop(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(62);
  }    
}

/******************************************************//**
 * @brief  Start the step clock by using the given frequency
 * @param[in] newFreq in Hz of the step clock
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MotorControl_StartStepClock(uint16_t newFreq)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->StartStepClock != 0))
  {
    motorDrvHandle->StartStepClock(newFreq);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(63);
  }    
}

/******************************************************//**
 * @brief  Stops the PWM uses for the step clock
 * @retval None
 **********************************************************/
void BSP_MotorControl_StopStepClock(void)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->StopStepClock != 0))
  {
    motorDrvHandle->StopStepClock();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(64);
  }    
}

/******************************************************//**
 * @brief Set the dual full bridge configuration
 * @param[in] config bridge configuration to apply  
 * for L6206, see dualFullBridgeConfig_t enum 
 * for Stspin240, 0 for a mono brush DC configuration, 1 for a dual brush DC configuration
 * for Stspin250, 0 only as no dual brush DC configuration is supported
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetDualFullBridgeConfig(uint8_t config)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetDualFullBridgeConfig != 0))
  {
    motorDrvHandle->SetDualFullBridgeConfig(config);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(65);
  }    
}

/******************************************************//**
 * @brief  Get the PWM frequency of the  bridge input
 * @param[in] bridgeId from 0 for bridge A to 1 for bridge B for brush DC motor
 * bridgeId must be 0 for L6208 (both bridges are set with the same frequency)
 * @retval Freq in Hz
 **********************************************************/
uint32_t BSP_MotorControl_GetBridgeInputPwmFreq(uint8_t bridgeId)
{	
  uint32_t pwmFreq = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetBridgeInputPwmFreq != 0))
  {
    pwmFreq = motorDrvHandle->GetBridgeInputPwmFreq(bridgeId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(66);
  }    
  return (pwmFreq);
}

/******************************************************//**
 * @brief  Changes the PWM frequency of the  bridge input
 * @param[in] bridgeId from 0 for bridge A to 1 for bridge B for brush DC motor
 * bridgeId must be 0 for L6208 (both bridges are set with the same frequency)
 * @param[in] newFreq in Hz up to 100000Hz
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetBridgeInputPwmFreq != 0))
  {
    motorDrvHandle->SetBridgeInputPwmFreq(bridgeId, newFreq);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(67);
  }    
}

/******************************************************//**
 * @brief Select the mode to stop the motor. When the motor
 * is stopped, if autoHiZ is TRUE, the power bridges are disabled
 * if autoHiZ is FALSE, the power bridges are kept enabled.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            deviceId dummy parameter for compatibility with motor.h
 * @param[in] stopMode selected stop mode
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetStopMode(uint8_t deviceId, motorStopMode_t stopMode)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetStopMode != 0))
  {
    motorDrvHandle->SetStopMode(deviceId, stopMode);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(68);
  } 
}

/******************************************************//**
 * @brief Get the selected stop mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @retval the selected stop mode
 **********************************************************/
motorStopMode_t BSP_MotorControl_GetStopMode(uint8_t deviceId)
{	
  motorStopMode_t stopMode = UNKNOW_STOP_MODE;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetStopMode != 0))
  {
    stopMode = motorDrvHandle->GetStopMode(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(69);
  }    
  return (stopMode);
}
 
/******************************************************//**
 * @brief Select the motor decay mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @param[in] decayMode (SLOW_DECAY or FAST_DECAY)
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetDecayMode(uint8_t deviceId, motorDecayMode_t decayMode)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetDecayMode != 0))
  {
    motorDrvHandle->SetDecayMode(deviceId, decayMode);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(70);
  } 
}
 
/******************************************************//**
 * @brief Get the motor decay mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @retval decay mode
 **********************************************************/
motorDecayMode_t BSP_MotorControl_GetDecayMode(uint8_t deviceId)
{	
  motorDecayMode_t decayMode = UNKNOW_DECAY;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetDecayMode != 0))
  {
    decayMode = motorDrvHandle->GetDecayMode(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(71);
  }    
  return (decayMode);
}

/******************************************************//**
 * @brief Get the motor step mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @retval step mode
 **********************************************************/
motorStepMode_t BSP_MotorControl_GetStepMode(uint8_t deviceId)
{	
  motorStepMode_t stepMode = STEP_MODE_UNKNOW;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetStepMode != 0))
  {
    stepMode = motorDrvHandle->GetStepMode(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(72);
  }    
  return (stepMode);
}

/******************************************************//**
 * @brief Get the motor direction
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @retval direction
 **********************************************************/
motorDir_t BSP_MotorControl_GetDirection(uint8_t deviceId)
{	
  motorDir_t dir = UNKNOW_DIR;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetDirection != 0))
  {
    dir = motorDrvHandle->GetDirection(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(73);
  }    
  return (dir);
}

/******************************************************//**
 * @brief Exit specified device from reset
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void BSP_MotorControl_ExitDeviceFromReset(uint8_t deviceId)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->ExitDeviceFromReset != 0))
  {
    motorDrvHandle->ExitDeviceFromReset(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(74);
  }    
}

/******************************************************//**
 * @brief Get the motor torque
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] torqueMode Torque mode as specified in enum motorTorqueMode_t
 * @retval torque value in % (from 0 to 100)
 **********************************************************/
uint8_t BSP_MotorControl_GetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode)
{	
 uint8_t torqueValue = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetTorque != 0))
  {
    torqueValue = motorDrvHandle->GetTorque(deviceId, torqueMode);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(76);
  }    
  return (torqueValue);
}

/******************************************************//**
 * @brief Set the motor torque
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] torqueMode Torque mode as specified in enum motorTorqueMode_t
 * @param[in] torqueValue in % (from 0 to 100)
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode, uint8_t torqueValue)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetTorque != 0))
  {
    motorDrvHandle->SetTorque(deviceId, torqueMode, torqueValue);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(75);
  }    
}

/******************************************************//**
 * @brief Set the PWM frequency of Ref
 * @param[in] refId Id of the targeted Ref
 * @param[in] newFreq frequency in Hz
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetRefFreq(uint8_t refId, uint32_t newFreq)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetRefFreq != 0))
  {
    motorDrvHandle->SetRefFreq(refId, newFreq);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(77);
  }
}

/******************************************************//**
 * @brief Return the PWM frequency of Ref
 * @param[in] refId Id of the targeted Ref
 * @retval frequency in Hz
 **********************************************************/
uint32_t BSP_MotorControl_GetRefFreq(uint8_t refId)
{
  uint32_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetRefFreq != 0))
  {
    value = motorDrvHandle->GetRefFreq(refId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(78);
  }
  return (value);
}

/******************************************************//**
 * @brief Set the PWM duty cycle of Ref
 * @param[in] refId Id of the targeted Ref
 * @param[in] newDc duty cycle in % (from 0 to 100)
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetRefDc(uint8_t refId, uint8_t newDc)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetRefDc != 0))
  {
    motorDrvHandle->SetRefDc(refId, newDc);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(79);
  }
}

/******************************************************//**
 * @brief Return the PWM duty cycle of Ref
 * @param[in] refId Id of the targeted Ref
 * @retval duty cycle in % (from 0 to 100)
 **********************************************************/
uint8_t BSP_MotorControl_GetRefDc(uint8_t refId)
{
  uint8_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetRefDc != 0))
  {
    value = motorDrvHandle->GetRefDc(refId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(80);
  }
  return (value);
}

/******************************************************//**
 * @brief Set the number of devices in the daisy chain
 * @param[in] id Component Id (L6474, Powerstep01,...)
 * @param[in] nbDevices the number of devices to be used 
 * from 1 to MAX_NUMBER_OF_DEVICES
 * @retval TRUE if successfull, FALSE if failure, attempt 
 * to set a number of devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool BSP_MotorControl_SetNbDevices(uint16_t id, uint8_t nbDevices)
{
  MotorControlBoardId = id;
  bool status = FALSE;
  if (id == BSP_MOTOR_CONTROL_BOARD_ID_L6474)
  {
    motorDrvHandle = L6474_GetMotorHandle();
  }
  else if (id == BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01)
  {
    motorDrvHandle = Powerstep01_GetMotorHandle();
  }
  else if (id == BSP_MOTOR_CONTROL_BOARD_ID_L6206)
  {
    motorDrvHandle = L6206_GetMotorHandle();
  }
  else if (id == BSP_MOTOR_CONTROL_BOARD_ID_L6208)
  {
    motorDrvHandle = L6208_GetMotorHandle();
  }
  else if (id == BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220)
  {
    motorDrvHandle = Stspin220_GetMotorHandle();
  }
  else if ( (id == BSP_MOTOR_CONTROL_BOARD_ID_L6470) ||
		     (id == BSP_MOTOR_CONTROL_BOARD_ID_L6472) )
  {
    motorDrvHandle = l647x_GetMotorHandle();
  }
  else if ( (id == BSP_MOTOR_CONTROL_BOARD_ID_L6480) ||
		     (id == BSP_MOTOR_CONTROL_BOARD_ID_L6482) )
  {
    motorDrvHandle = l648x_GetMotorHandle();
  }
  else if ((id == BSP_MOTOR_CONTROL_BOARD_ID_STSPIN240)||
            (id == BSP_MOTOR_CONTROL_BOARD_ID_STSPIN250))
  {
    motorDrvHandle = Stspin240_250_GetMotorHandle();
  }  
  else
  {
    motorDrvHandle = 0;
  }
  if ((motorDrvHandle != 0)&&
      (motorDrvHandle->SetNbDevices != 0)&&
      (nbDevices !=0))
  {
    status = motorDrvHandle->SetNbDevices(nbDevices);
  }
  return (status);
}

/******************************************************//**
 * @brief Set the parameter param in the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] param Register adress (BSP_MotorControl_ABS_POS, BSP_MotorControl_MARK,...)
 * @param[in] value Floating point value to convert and set into the register
 * @retval None
 **********************************************************/
bool BSP_MotorControl_SetAnalogValue(uint8_t deviceId, uint32_t param, float value)
{
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdSetParam != 0))
  {
    status = motorDrvHandle->SetAnalogValue(deviceId, param, value);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(82);
  }
  return (status);
}

/******************************************************//**
 * @brief Get the parameter param in the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] param Register adress (BSP_MotorControl_ABS_POS, BSP_MotorControl_MARK,...)
 * @retval Floating point value corresponding to the register value
 **********************************************************/
float BSP_MotorControl_GetAnalogValue(uint8_t deviceId, uint32_t param)
{
  float value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetAnalogValue != 0))
  {
    value = motorDrvHandle->GetAnalogValue(deviceId, param);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(83);
  }       
  return (value);
}

/******************************************************//**
 * @brief Enable or disable the torque boost
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] enable true to enable torque boost, false to disable
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetTorqueBoostEnable(uint8_t deviceId, bool enable)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetTorqueBoostEnable != 0))
  {
    motorDrvHandle->SetTorqueBoostEnable(deviceId, enable);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(84);
  }
}

/******************************************************//**
 * @brief Get the torque boost feature status
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval TRUE if enabled, FALSE if disabled
 **********************************************************/
bool BSP_MotorControl_GetTorqueBoostEnable(uint8_t deviceId)
{
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetTorqueBoostEnable != 0))
  {
    status = motorDrvHandle->GetTorqueBoostEnable(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(85);
  }
  return status;
}

/******************************************************//**
 * @brief Set the torque boost threshold
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] speedThreshold speed threshold above which the step mode is
 * changed to full step
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetTorqueBoostThreshold(uint8_t deviceId, uint16_t speedThreshold)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetTorqueBoostThreshold != 0))
  {
    motorDrvHandle->SetTorqueBoostThreshold(deviceId, speedThreshold);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(86);
  }
}

/******************************************************//**
 * @brief Get the torque boost threshold
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval the torque boost threshold above which the step mode is
 * changed to full step
 **********************************************************/
uint16_t BSP_MotorControl_GetTorqueBoostThreshold(uint8_t deviceId)
{
  uint16_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetTorqueBoostThreshold != 0))
  {
    value = motorDrvHandle->GetTorqueBoostThreshold(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(87);
  }
  return value;
}

/******************************************************//**
 * @brief Get the dual full bridge configuration
 * return config bridge configuration to apply   
 * for L6206, see dualFullBridgeConfig_t enum 
 * for Stspin240, 0 for a mono brush DC configuration, 1 for a dual brush DC configuration
 * for Stspin250, 0 only as no dual brush DC configuration is supported
 **********************************************************/
uint8_t BSP_MotorControl_GetDualFullBridgeConfig(void)
{	
  uint8_t value = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetDualFullBridgeConfig != 0))
  {
    value = motorDrvHandle->GetDualFullBridgeConfig();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(88);
  }    
  
  return value;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
