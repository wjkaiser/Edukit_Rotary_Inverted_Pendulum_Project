/**
  ******************************************************************************
  * @file    l6474.c
  * @author  IPC Rennes
  * @version V1.10.0
  * @date    March 16th, 2018
  * @brief   L6474 driver (fully integrated microstepping motor driver)
  * @note    (C) COPYRIGHT 2018 STMicroelectronics
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
#include "l6474.h"

/* Private constants  ---------------------------------------------------------*/

    
/** @addtogroup BSP
  * @{
  */   
   
/** @defgroup L6474 L6474
  * @{
  */   

/* Private constants ---------------------------------------------------------*/    

/** @defgroup L6474_Private_Constants L6474 Private Constants
  * @{
  */   

/// Error while initialising the SPI
#define L6474_ERROR_0   (0x8000)   
/// Error: Bad SPI transaction
#define L6474_ERROR_1   (0x8001)
    
/// Maximum number of steps
#define MAX_STEPS         (0x7FFFFFFF)

/// Maximum frequency of the PWMs in Hz
#define L6474_MAX_PWM_FREQ   (40000)

/// Minimum frequency of the PWMs in Hz
#define L6474_MIN_PWM_FREQ   (1)
    
/**
  * @}
  */ 
    
/* Private variables ---------------------------------------------------------*/

/** @defgroup L6474_Private_Variables L6474 Private Variables
  * @{
  */       
    
/// Function pointer to flag interrupt call back
void (*flagInterruptCallback)(void);
/// Function pointer to error handler call back
void (*errorHandlerCallback)(uint16_t);
static volatile uint8_t numberOfDevices;
static uint8_t spiTxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
static uint8_t spiRxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
static volatile bool spiPreemtionByIsr = FALSE;
static volatile bool isrFlag = FALSE;
static uint16_t l6474DriverInstance = 0;

/// L6474 Device Paramaters structure
deviceParams_t devicePrm[MAX_NUMBER_OF_DEVICES];


/**
  * @}
  */ 

/* Private function prototypes -----------------------------------------------*/

/** @defgroup L6474_Private_functions L6474 Private functions
  * @{
  */  
void L6474_ApplySpeed(uint8_t pwmId, uint16_t newSpeed);
void L6474_ComputeSpeedProfile(uint8_t deviceId, uint32_t nbSteps);
int32_t L6474_ConvertPosition(uint32_t abs_position_reg); 
void L6474_ErrorHandler(uint16_t error);
void L6474_FlagInterruptHandler(void);                      
void L6474_SendCommand(uint8_t deviceId, uint8_t param);
void L6474_SetRegisterToGivenValues(uint8_t deviceId, L6474_Init_t *pInitPrm);
void L6474_SetRegisterToPredefinedValues(uint8_t deviceId);
void L6474_WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);    
void L6474_SetDeviceParamsToPredefinedValues(uint8_t deviceId);
void L6474_SetDeviceParamsToGivenValues(uint8_t deviceId, L6474_Init_t *pInitPrm);
void L6474_StartMovement(uint8_t deviceId);
void L6474_StepClockHandler(uint8_t deviceId);  
uint8_t L6474_Ocd_Th_to_Par(float Tval);
float L6474_Ocd_Par_to_Th(uint8_t Par);
uint8_t L6474_Tval_Current_to_Par(float Tval);
uint8_t L6474_Tmin_Time_to_Par(float Tmin);
float L6474_Tmin_Par_to_Time(uint8_t Par);
float L6474_Tval_Par_to_Current(uint8_t Par);

/**
  * @}
  */ 


/** @defgroup L6474_Exported_Variables L6474 Exported Variables
  * @{
  */       

/// L6474 motor driver functions pointer structure 
motorDrv_t   l6474Drv =
{
  L6474_Init,                             //void (*Init)(void*);
  L6474_ReadId,                           //uint16_t (*ReadID)(void);
  L6474_AttachErrorHandler,               //void (*AttachErrorHandler)(void (*callback)(uint16_t));
  L6474_AttachFlagInterrupt,              //void (*AttachFlagInterrupt)(void (*callback)(void));
  0,                                      //void (*AttachBusyInterrupt)(void (*callback)(void));
  L6474_FlagInterruptHandler,             //void (*FlagInterruptHandler)(void);
  L6474_GetAcceleration,                  //uint16_t (*GetAcceleration)(uint8_t);
  L6474_GetCurrentSpeed,                  //uint16_t (*GetCurrentSpeed)(uint8_t);
  L6474_GetDeceleration,                  //uint16_t (*GetDeceleration)(uint8_t);
  L6474_GetDeviceState,                   //motorState_t(*GetDeviceState)(uint8_t);
  L6474_GetFwVersion,                     //uint32_t (*GetFwVersion)(void);
  L6474_GetMark,                          //int32_t (*GetMark)(uint8_t);
  L6474_GetMaxSpeed,                      //uint16_t (*GetMaxSpeed)(uint8_t);
  L6474_GetMinSpeed,                      //uint16_t (*GetMinSpeed)(uint8_t);
  L6474_GetPosition,                      //int32_t (*GetPosition)(uint8_t);
  L6474_GoHome,                           //void (*GoHome)(uint8_t);
  L6474_GoMark,                           //void (*GoMark)(uint8_t);
  L6474_GoTo,                             //void (*GoTo)(uint8_t, int32_t);
  L6474_HardStop,                         //void (*HardStop)(uint8_t);
  L6474_Move,                             //void (*Move)(uint8_t, motorDir_t, uint32_t );
  L6474_ResetAllDevices,                  //void (*ResetAllDevices)(void);
  L6474_Run,                              //void (*Run)(uint8_t, motorDir_t);
  L6474_SetAcceleration,                  //bool(*SetAcceleration)(uint8_t ,uint16_t );
  L6474_SetDeceleration,                  //bool(*SetDeceleration)(uint8_t , uint16_t );
  L6474_SetHome,                          //void (*SetHome)(uint8_t, int32_t);
  L6474_SetMark,                          //void (*SetMark)(uint8_t, int32_t);
  L6474_SetMaxSpeed,                      //bool (*SetMaxSpeed)(uint8_t, uint16_t );
  L6474_SetMinSpeed,                      //bool (*SetMinSpeed)(uint8_t, uint16_t );
  L6474_SoftStop,                         //bool (*SoftStop)(uint8_t);
  L6474_StepClockHandler,                 //void (*StepClockHandler)(uint8_t deviceId);
  L6474_WaitWhileActive,                  //void (*WaitWhileActive)(uint8_t);
  L6474_CmdDisable,                       //void (*CmdDisable)(uint8_t);
  L6474_CmdEnable,                        //void (*CmdEnable)(uint8_t);
  L6474_CmdGetParam,                      //uint32_t (*CmdGetParam)(uint8_t, uint32_t);
  L6474_CmdGetStatus,                     //uint16_t (*CmdGetStatus)(uint8_t);
  L6474_CmdNop,                           //void (*CmdNop)(uint8_t);
  L6474_CmdSetParam,                      //void (*CmdSetParam)(uint8_t, uint32_t, uint32_t);
  L6474_ReadStatusRegister,               //uint16_t (*ReadStatusRegister)(uint8_t);
  L6474_ReleaseReset,                     //void (*ReleaseReset)(void);
  L6474_Reset,                            //void (*Reset)(void);
  L6474_SelectStepMode,                   //bool (*SelectStepMode)(uint8_t deviceId, motorStepMode_t);
  L6474_SetDirection,                     //void (*SetDirection)(uint8_t, motorDir_t);
  0,                                      //void (*CmdGoToDir)(uint8_t, motorDir_t, int32_t);
  0,                                      //uint8_t (*CheckBusyHw)(void);
  0,                                      //uint8_t (*CheckStatusHw)(void);
  0,                                      //void (*CmdGoUntil)(uint8_t, motorAction_t, motorDir_t, uint32_t);
  L6474_HizStop,                          //void (*CmdHardHiZ)(uint8_t);
  0,                                      //void (*CmdReleaseSw)(uint8_t, motorAction_t, motorDir_t);
  0,                                      //void (*CmdResetDevice)(uint8_t);
  0,                                      //void (*CmdResetPos)(uint8_t);
  0,                                      //void (*CmdRun)(uint8_t, motorDir_t, uint32_t);
  0,                                      //void (*CmdSoftHiZ)(uint8_t);
  0,                                      //void (*CmdStepClock)(uint8_t, motorDir_t);
  0,                                      //void (*FetchAndClearAllStatus)(void);
  0,                                      //uint16_t (*GetFetchedStatus)(uint8_t);
  L6474_GetNbDevices,                     //uint8_t (*GetNbDevices)(void);
  0,                                      //bool (*IsDeviceBusy)(uint8_t);
  0,                                      //void (*SendQueuedCommands)(void);
  0,                                      //void (*QueueCommands)(uint8_t, uint8_t, int32_t);
  0,                                      //void (*WaitForAllDevicesNotBusy)(void);
  L6474_ErrorHandler,                     //void (*ErrorHandler)(uint16_t);
  0,                                      //void (*BusyInterruptHandler)(void);
  0,                                      //void (*CmdSoftStop)(uint8_t);
  0,                                      //void (*StartStepClock)(uint16_t);
  0,                                      //void (*StopStepClock)(void);
  0,                                      //void (*SetDualFullBridgeConfig)(uint8_t);
  0,                                      //uint32_t (*GetBridgeInputPwmFreq)(uint8_t);
  0,                                      //void (*SetBridgeInputPwmFreq)(uint8_t, uint32_t);
  L6474_SetStopMode,                      //void (*SetStopMode)(uint8_t, motorStopMode_t);
  L6474_GetStopMode,                      //motorStopMode_t (*GetStopMode)(uint8_t);
  0,                                      //void (*SetDecayMode)(uint8_t, motorDecayMode_t);
  0,                                      //motorDecayMode_t (*GetDecayMode)(uint8_t);
  L6474_GetStepMode,                      //motorStepMode_t (*GetStepMode)(uint8_t);
  L6474_GetDirection,                     //motorDir_t (*GetDirection)(uint8_t);
  0,                                      //void (*ExitDeviceFromReset)(uint8_t);
  0,                                      //void (*SetTorque)(uint8_t, motorTorqueMode_t, uint8_t);
  0,                                      //uint8_t (*GetTorque)(uint8_t, motorTorqueMode_t);
  0,                                      //void (*SetRefFreq)(uint8_t, uint32_t);
  0,                                      //uint32_t (*GetRefFreq)(uint8_t);
  0,                                      //void (*SetRefDc)(uint8_t, uint8_t);
  0,                                      //uint8_t (*GetRefDc)(uint8_t);
  L6474_SetNbDevices,                     //bool (*SetNbDevices)(uint8_t);
  L6474_SetAnalogValue,                   //bool (*SetAnalogValue)(uint8_t, uint32_t, float);
  L6474_GetAnalogValue                    //float (*GetAnalogValue )(uint8_t, uint32_t);
};
/**
  * @}
  */ 

/** @defgroup L6474_Library_Functions L6474 Library Functions
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
void L6474_AttachErrorHandler(void (*callback)(uint16_t))
{
  errorHandlerCallback = (void (*)(uint16_t))callback;
}

/******************************************************//**
 * @brief  Attaches a user callback to the flag Interrupt
 * The call back will be then called each time the status 
 * flag pin will be pulled down due to the occurrence of 
 * a programmed alarms ( OCD, thermal pre-warning or 
 * shutdown, UVLO, wrong command, non-performable command)
 * @param[in] callback Name of the callback to attach 
 * to the Flag Interrupt
 * @retval None
 **********************************************************/
void L6474_AttachFlagInterrupt(void (*callback)(void))
{
  flagInterruptCallback = (void (*)())callback;
}

/******************************************************//**
 * @brief  Issue the Disable command to the L6474 of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_CmdDisable(uint8_t deviceId)
{
  L6474_SendCommand(deviceId, L6474_DISABLE);
}

/******************************************************//**
 * @brief  Issues the Enable command to the L6474 of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_CmdEnable(uint8_t deviceId)
{
  L6474_SendCommand(deviceId, L6474_ENABLE);
}

/******************************************************//**
 * @brief  Issues the GetParam command to the L6474 of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @param[in] param Register adress (L6474_ABS_POS, L6474_MARK,...)
 * @retval Register value
 **********************************************************/
uint32_t L6474_CmdGetParam(uint8_t deviceId, uint32_t param)
{
  uint32_t i;
  uint32_t spiRxData;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = numberOfDevices - deviceId - 1;
  bool itDisable = FALSE;  
  
  do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_Board_EnableIrq if disable in previous iteration */
      L6474_Board_EnableIrq();
      itDisable = FALSE;
    }
  
    for (i = 0; i < numberOfDevices; i++)
    {
      spiTxBursts[0][i] = L6474_NOP;
      spiTxBursts[1][i] = L6474_NOP;
      spiTxBursts[2][i] = L6474_NOP;
      spiTxBursts[3][i] = L6474_NOP;
      spiRxBursts[1][i] = 0;
      spiRxBursts[2][i] = 0;
      spiRxBursts[3][i] = 0;    
    }
    switch (param)
    {
      case L6474_ABS_POS: ;
      case L6474_MARK:
        spiTxBursts[0][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (param);
        maxArgumentNbBytes = 3;
        break;
      case L6474_EL_POS: ;
      case L6474_CONFIG: ;
      case L6474_STATUS:
        spiTxBursts[1][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (param);
        maxArgumentNbBytes = 2;
        break;
      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
    
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     L6474_WriteBytes(&spiTxBursts[i][0],
                          &spiRxBursts[i][0]);
  }
  
  spiRxData = ((uint32_t)spiRxBursts[1][spiIndex] << 16)|
              (spiRxBursts[2][spiIndex] << 8) |
              (spiRxBursts[3][spiIndex]);
  
  /* re-enable L6474_Board_EnableIrq after SPI transfers*/
  L6474_Board_EnableIrq();
    
  return (spiRxData);
}

/******************************************************//**
 * @brief  Issues the GetStatus command to the L6474 of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval Status Register value
 * @note Once the GetStatus command is performed, the flags of the status register
 * are reset. This is not the case when the status register is read with the
 * GetParam command (via the functions L6474ReadStatusRegister or L6474CmdGetParam).
 **********************************************************/
uint16_t L6474_CmdGetStatus(uint8_t deviceId)
{
  uint32_t i;
  uint16_t status;
  uint8_t spiIndex = numberOfDevices - deviceId - 1;
  bool itDisable = FALSE;  
  
  do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_Board_EnableIrq if disable in previous iteration */
      L6474_Board_EnableIrq();
      itDisable = FALSE;
    }

    for (i = 0; i < numberOfDevices; i++)
    {
       spiTxBursts[0][i] = L6474_NOP;
       spiTxBursts[1][i] = L6474_NOP;
       spiTxBursts[2][i] = L6474_NOP;
       spiRxBursts[1][i] = 0;
       spiRxBursts[2][i] = 0;
    }
    spiTxBursts[0][spiIndex] = L6474_GET_STATUS;

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  for (i = 0; i < L6474_CMD_ARG_NB_BYTES_GET_STATUS + L6474_RSP_NB_BYTES_GET_STATUS; i++)
  {
     L6474_WriteBytes(&spiTxBursts[i][0], &spiRxBursts[i][0]);
  }
  status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
  
  /* re-enable L6474_Board_EnableIrq after SPI transfers*/
  L6474_Board_EnableIrq();
  
  return (status);
}

/******************************************************//**
 * @brief  Issues the Nop command to the L6474 of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_CmdNop(uint8_t deviceId)
{
  L6474_SendCommand(deviceId, L6474_NOP);
}

/******************************************************//**
 * @brief  Issues the SetParam command to the L6474 of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @param[in] param Register adress (L6474_ABS_POS, L6474_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void L6474_CmdSetParam(uint8_t deviceId,
                       uint32_t param,
                       uint32_t value)
{
  uint32_t i;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = numberOfDevices - deviceId - 1;
  bool itDisable = FALSE;  
  do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_Board_EnableIrq if disable in previous iteration */
      L6474_Board_EnableIrq();
      itDisable = FALSE;
    }
    for (i = 0; i < numberOfDevices; i++)
    {
      spiTxBursts[0][i] = L6474_NOP;
      spiTxBursts[1][i] = L6474_NOP;
      spiTxBursts[2][i] = L6474_NOP;
      spiTxBursts[3][i] = L6474_NOP;
    }
    switch (param)
  {
    case L6474_ABS_POS: ;
    case L6474_MARK:
        spiTxBursts[0][spiIndex] = param;
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 3;
        break;
    case L6474_EL_POS: ;
    case L6474_CONFIG:
        spiTxBursts[1][spiIndex] = param;
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 2;
        break;
    default:
        spiTxBursts[2][spiIndex] = param;
        maxArgumentNbBytes = 1;
    }
    spiTxBursts[3][spiIndex] = (uint8_t)(value);
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
 
  /* SPI transfer */
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     L6474_WriteBytes(&spiTxBursts[i][0],&spiRxBursts[i][0]);
  }
  /* re-enable L6474_Board_EnableIrq after SPI transfers*/
  L6474_Board_EnableIrq();
}

/******************************************************//**
 * @brief Starts a new L6474 instance 
 * @param[in] pInit pointer to the initialization data
 * @retval None
 **********************************************************/
void L6474_Init(void* pInit)
{
  /* Initialise the GPIOs */
  L6474_Board_GpioInit(l6474DriverInstance);
  
  if(L6474_Board_SpiInit() != 0)
  {
    /* Initialization Error */
    L6474_ErrorHandler(L6474_ERROR_0);
  } 

  /* Initialise the PWMs used for the Step clocks ----------------------------*/
  L6474_Board_PwmInit(l6474DriverInstance);
 
  /* Standby-reset deactivation */
  L6474_Board_ReleaseReset(l6474DriverInstance);
  
  /* Let a delay after reset */
  L6474_Board_Delay(1); 
  
  /* Set all registers and context variables to the predefined values from l6474_target_config.h */
  if (pInit == 0)
  {
    L6474_SetDeviceParamsToPredefinedValues(l6474DriverInstance);
  }
  else
  {
    L6474_SetDeviceParamsToGivenValues(l6474DriverInstance, pInit);
  }
  /* Disable L6474 powerstage */
  L6474_CmdDisable(l6474DriverInstance);

  /* Get Status to clear flags after start up */
  L6474_CmdGetStatus(l6474DriverInstance);

  l6474DriverInstance++;
}

/******************************************************//**
 * @brief Returns the acceleration of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval Acceleration in pps^2
 **********************************************************/
uint16_t L6474_GetAcceleration(uint8_t deviceId)
{                                                  
  return (devicePrm[deviceId].acceleration);
}            

/******************************************************//**
 * @brief Issues L6474 Get analog values from register  
 * L6474_ABS_POS, L6474_MARK, L6474_TVAL, L6474_OCD_TH,
 * L6474_TON_MIN, L6474_TOFF_MIN. The raw register value is 
 * converted in alanog values.
 * For other registers, the returned value is the raw value.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param L6474 register address
 * @retval Register value - 1 to 3 bytes (depends on register)
 *********************************************************/
float L6474_GetAnalogValue(uint8_t deviceId, uint32_t param)
{
  uint32_t registerValue = L6474_CmdGetParam(deviceId, param);
  float value;
  switch (param)
  {
    case L6474_ABS_POS:
    case L6474_MARK:
      value = (float) L6474_ConvertPosition(registerValue);
      break;
    case L6474_TVAL:
      value = L6474_Tval_Par_to_Current(registerValue);    
      break;
    case L6474_OCD_TH:
      value = L6474_Ocd_Par_to_Th(registerValue);    
      break;      
    case L6474_TON_MIN:
    case L6474_TOFF_MIN:
      value = L6474_Tmin_Par_to_Time(registerValue);
      break;          
    default:
      value = (float) registerValue;
  }
  return value;
}

/******************************************************//**
 * @brief Returns the current speed of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval Speed in pps
 **********************************************************/
uint16_t L6474_GetCurrentSpeed(uint8_t deviceId)
{
  return devicePrm[deviceId].speed;
}

/******************************************************//**
 * @brief Returns the deceleration of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval Deceleration in pps^2
 **********************************************************/
uint16_t L6474_GetDeceleration(uint8_t deviceId)
{                                                  
  return (devicePrm[deviceId].deceleration);
}          

/******************************************************//**
 * @brief Returns the device state
 * @param[in] deviceId (from 0 to 2)
 * @retval State (ACCELERATING, DECELERATING, STEADY or INACTIVE)
 **********************************************************/
motorState_t L6474_GetDeviceState(uint8_t deviceId)
{
  return devicePrm[deviceId].motionState;
}

/******************************************************//**
 * @brief Get the motor current direction
 * @param[in] deviceId Unused parameter
 * @retval direction
 **********************************************************/
motorDir_t L6474_GetDirection(uint8_t deviceId)
{
  return devicePrm[deviceId].direction;
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval L6474_FW_VERSION
 **********************************************************/
uint32_t L6474_GetFwVersion(void)
{
  return (L6474_FW_VERSION);
}

/******************************************************//**
 * @brief  Return motor handle (pointer to the L6474 motor driver structure)
 * @retval Pointer to the motorDrv_t structure
 **********************************************************/
motorDrv_t* L6474_GetMotorHandle(void)
{
  return (&l6474Drv);
}

/******************************************************//**
 * @brief  Returns the mark position  of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t L6474_GetMark(uint8_t deviceId)
{
  return L6474_ConvertPosition(L6474_CmdGetParam(deviceId,L6474_MARK));
}

/******************************************************//**
 * @brief  Returns the max speed of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval maxSpeed in pps
 **********************************************************/
uint16_t L6474_GetMaxSpeed(uint8_t deviceId)
{                                                  
  return (devicePrm[deviceId].maxSpeed);
}

/******************************************************//**
 * @brief  Returns the min speed of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval minSpeed in pps
 **********************************************************/
uint16_t L6474_GetMinSpeed(uint8_t deviceId)
{                                                  
  return (devicePrm[deviceId].minSpeed);
}                                                     

/******************************************************//**
 * @brief  Returns the number of devices
 * @retval number of devices
 **********************************************************/
uint8_t L6474_GetNbDevices(void)
{
  return (numberOfDevices);
}
 
/******************************************************//**
 * @brief  Returns the ABS_POSITION of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval ABS_POSITION register value converted in a 32b signed integer
 **********************************************************/
int32_t L6474_GetPosition(uint8_t deviceId)
{
  return L6474_ConvertPosition(L6474_CmdGetParam(deviceId,L6474_ABS_POS));
}

/******************************************************//**
 * @brief Get the motor step mode
 * @param[in] deviceId Unused parameter
 * @retval step mode
 **********************************************************/
motorStepMode_t L6474_GetStepMode(uint8_t deviceId)
{
  motorStepMode_t stepMode;
  uint8_t stepSelValue;
  
  /* Get STEP_SEL field of step mode register  */
  stepSelValue = (uint8_t)((0x07 & L6474_CmdGetParam(deviceId,L6474_STEP_MODE))|0x08) ;
  
   switch (stepSelValue)
  {
    case L6474_STEP_SEL_1:
      stepMode = STEP_MODE_FULL;
      break;
    case L6474_STEP_SEL_1_2:
      stepMode = STEP_MODE_HALF;
      break;    
    case L6474_STEP_SEL_1_4:
      stepMode = STEP_MODE_1_4;
      break;        
    case L6474_STEP_SEL_1_8:
      stepMode = STEP_MODE_1_8;
      break;       
    case L6474_STEP_SEL_1_16:
      stepMode = STEP_MODE_1_16;
      break;
    default:
      stepMode = STEP_MODE_UNKNOW;
      break;       
  }
  
  return stepMode;
}

/******************************************************//**
 * @brief Get the selected stop mode
 * @param[in] deviceId Unused parameter
 * @retval the selected stop mode
 **********************************************************/
motorStopMode_t L6474_GetStopMode(uint8_t deviceId)
{
  return devicePrm[deviceId].stopMode;
}

/******************************************************//**
 * @brief  Requests the motor to move to the home position (ABS_POSITION = 0)
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_GoHome(uint8_t deviceId)
{
  L6474_GoTo(deviceId, 0);
} 
  
/******************************************************//**
 * @brief  Requests the motor to move to the mark position 
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_GoMark(uint8_t deviceId)
{
  uint32_t mark;

  mark = L6474_ConvertPosition(L6474_CmdGetParam(deviceId,L6474_MARK));
  L6474_GoTo(deviceId,mark);  
}

/******************************************************//**
 * @brief  Requests the motor to move to the specified position 
 * @param[in] deviceId (from 0 to 2)
 * @param[in] targetPosition absolute position in steps
 * @retval None
 **********************************************************/
void L6474_GoTo(uint8_t deviceId, int32_t targetPosition)
{
  motorDir_t direction;
  int32_t steps;
  
  /* If required deactivate motor */
  if (devicePrm[deviceId].motionState != INACTIVE) 
  {
    L6474_HardStop(deviceId);
  }

  /* Get current position */
  devicePrm[deviceId].currentPosition = L6474_ConvertPosition(L6474_CmdGetParam(deviceId,L6474_ABS_POS));
  
  /* Compute the number of steps to perform */
  steps = targetPosition - devicePrm[deviceId].currentPosition;
  //steps *= 2; // account for PWM clock divider
  
  if (steps >= 0) 
  {
    devicePrm[deviceId].stepsToTake = steps;
    direction = FORWARD;
    
  } 
  else 
  {
    devicePrm[deviceId].stepsToTake = -steps;
    direction = BACKWARD;
  }
  
  if (steps != 0) 
  {
    
    devicePrm[deviceId].commandExecuted = MOVE_CMD;
        
    /* Direction setup */
    L6474_SetDirection(deviceId,direction);

    L6474_ComputeSpeedProfile(deviceId, devicePrm[deviceId].stepsToTake);
    
    /* Motor activation */
    L6474_StartMovement(deviceId);
  }  
}

/******************************************************//**
 * @brief  Immediatly stops the motor 
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_HardStop(uint8_t deviceId) 
{
  if (devicePrm[deviceId].stopMode == HOLD_MODE)
  {
    /* Disable corresponding PWM */
    L6474_Board_PwmStop(deviceId);

    /* Set inactive state */
    devicePrm[deviceId].motionState = INACTIVE;
    devicePrm[deviceId].commandExecuted = NO_CMD;
    devicePrm[deviceId].stepsToTake = MAX_STEPS;  
    devicePrm[deviceId].speed = 0;
  }
  else
  {
    //same handling for HIZ_MODE and STANDBY_MODE
    L6474_HizStop(deviceId);
  }
}

/******************************************************//**
 * @brief  Immediatly stops the motor and disable the power bridge
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_HizStop(uint8_t deviceId) 
{
  /* Disable corresponding PWM */
  L6474_Board_PwmStop(deviceId);

  /* Disable power stage */
  L6474_CmdDisable(deviceId);

  /* Set inactive state */
  devicePrm[deviceId].motionState = INACTIVE;
  devicePrm[deviceId].commandExecuted = NO_CMD;
  devicePrm[deviceId].stepsToTake = MAX_STEPS;  
  devicePrm[deviceId].speed = 0;
}

/******************************************************//**
 * @brief  Moves the motor of the specified number of steps
 * @param[in] deviceId (from 0 to 2)
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount Number of steps to perform
 * @retval None
 **********************************************************/
void L6474_Move(uint8_t deviceId, motorDir_t direction, uint32_t stepCount)
{
  /* If required deactivate motor */
  if (devicePrm[deviceId].motionState != INACTIVE) 
  {
    L6474_HardStop(deviceId);
  }
  
  if (stepCount != 0) 
  {
    //stepCount *= 2; // account for PWM clock divider

    devicePrm[deviceId].stepsToTake = stepCount;
    
    devicePrm[deviceId].commandExecuted = MOVE_CMD;
    
    devicePrm[deviceId].currentPosition = L6474_ConvertPosition(L6474_CmdGetParam(deviceId,L6474_ABS_POS));
    
    /* Direction setup */
    L6474_SetDirection(deviceId,direction);

    L6474_ComputeSpeedProfile(deviceId, stepCount);
    
    /* Motor activation */
    L6474_StartMovement(deviceId);
  }  
}

/******************************************************//**
 * @brief Read id
 * @retval Id of the l6474 Driver Instance
 **********************************************************/
uint16_t L6474_ReadId(void)
{
  return(l6474DriverInstance);
}

/******************************************************//**
 * @brief  Reads the Status Register value
 * @param[in] deviceId (from 0 to 2)
 * @retval Status register valued
 * @note The status register flags are not cleared 
 * at the difference with L6474CmdGetStatus()
 **********************************************************/
uint16_t L6474_ReadStatusRegister(uint8_t deviceId)
{
  return (L6474_CmdGetParam(deviceId,L6474_STATUS));
}

/******************************************************//**
 * @brief  Releases the L6474 reset (pin set to High) of all devices
 * @retval None
 **********************************************************/
void L6474_ReleaseReset(uint8_t deviceId)
{ 
  L6474_Board_ReleaseReset(deviceId); 
}

/******************************************************//**
 * @brief  Resets the L6474 (reset pin set to low) of all devices
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_Reset(uint8_t deviceId)
{
  L6474_Board_Reset(deviceId); 
}

/******************************************************//**
 * @brief Resets all L6474 devices
 * @retval None
 **********************************************************/
void L6474_ResetAllDevices(void)
{
 	uint8_t loop;
 	
 	for (loop = 0; loop < numberOfDevices; loop++)
 	{
   	/* Stop movement and disable power stage*/
  	L6474_HizStop(loop);
    L6474_Reset(loop);
    L6474_Board_Delay(1); // Reset pin must be forced low for at least 10us
    L6474_Board_ReleaseReset(loop);
    L6474_Board_Delay(1); 
  }
}

/******************************************************//**
 * @brief  Runs the motor. It will accelerate from the min 
 * speed up to the max speed by using the device acceleration.
 * @param[in] deviceId (from 0 to 2)
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 **********************************************************/
void L6474_Run(uint8_t deviceId, motorDir_t direction)
{
  /* If required deactivate motor */
  if (devicePrm[deviceId].motionState != INACTIVE) 
  {
    L6474_HardStop(deviceId);
  }
  
	/* Direction setup */
	L6474_SetDirection(deviceId,direction);

	devicePrm[deviceId].commandExecuted = RUN_CMD;

	/* Motor activation */
	L6474_StartMovement(deviceId); 
}

/******************************************************//**
 * @brief  Set the stepping mode 
 * @param[in] deviceId (from 0 to 2)
 * @param[in] stepMod from full step to 1/16 microstep as specified in enum motorStepMode_t
 * @retval None
 **********************************************************/
bool L6474_SelectStepMode(uint8_t deviceId, motorStepMode_t stepMod)
{
  uint8_t stepModeRegister;
  L6474_STEP_SEL_t l6474StepMod;
  
  switch (stepMod)
  {
    case STEP_MODE_FULL:
      l6474StepMod = L6474_STEP_SEL_1;
      break;
    case STEP_MODE_HALF:
      l6474StepMod = L6474_STEP_SEL_1_2;
      break;    
    case STEP_MODE_1_4:
      l6474StepMod = L6474_STEP_SEL_1_4;
      break;        
    case STEP_MODE_1_8:
      l6474StepMod = L6474_STEP_SEL_1_8;
      break;       
    case STEP_MODE_1_16:
    default:
      l6474StepMod = L6474_STEP_SEL_1_16;
      break;       
  }

  /* Deactivate motor*/
  L6474_HizStop(deviceId);
  
  /* Read Step mode register and clear STEP_SEL field */
  stepModeRegister = (uint8_t)(0xF8 & L6474_CmdGetParam(deviceId,L6474_STEP_MODE)) ;
  
  /* Apply new step mode */
  L6474_CmdSetParam(deviceId, L6474_STEP_MODE, stepModeRegister | (uint8_t)l6474StepMod);

  /* Reset abs pos register */
  L6474_CmdSetParam(deviceId, L6474_ABS_POS, 0);
  
  return (1);
}

/******************************************************//**
 * @brief  Changes the acceleration of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @param[in] newAcc New acceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool L6474_SetAcceleration(uint8_t deviceId,uint16_t newAcc)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newAcc != 0)&&
      ((devicePrm[deviceId].motionState == INACTIVE)||
       (devicePrm[deviceId].commandExecuted == RUN_CMD)))
  {
    devicePrm[deviceId].acceleration = newAcc;
    cmdExecuted = TRUE;
  }    
  return cmdExecuted;
}            

/******************************************************//**
 * @brief Set registers L6474_ABS_POS, L6474_MARK, L6474_TVAL, L6474_OCD_TH,
 * L6474_TON_MIN, L6474_TOFF_MIN by analog values. These 
 * analog values are automatically converted in bit values
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Register adress (L6474_ABS_POS, L6474_EL_POS, L6474_MARK, 
 * L6474_TVAL, L6474_TON_MIN, L6474_TOFF_MIN, L6474_OCD_TH)
 * @param[in] value Analog value to convert and set into the register
 * @retval TRUE if param and param is valid, FALSE otherwise
 *********************************************************/
bool L6474_SetAnalogValue(uint8_t deviceId, uint32_t param, float value)
{
  uint32_t registerValue;
  bool result = TRUE;
  if ((value < 0)&&(param != L6474_ABS_POS)&&(param != L6474_MARK)) 
  {
    result = FALSE;
  }
  else
  {
    switch (param)
    {
      case L6474_EL_POS:
        if  ((value !=0)&&
            ((value > (L6474_ELPOS_STEP_MASK | L6474_ELPOS_MICROSTEP_MASK))||
             (value < (1<<(7-(L6474_STEP_MODE_STEP_SEL & L6474_CmdGetParam(0,L6474_STEP_MODE)))))))
        {
          result = FALSE;
        }
        else
        {
          registerValue = ((uint32_t) value)& (L6474_ELPOS_STEP_MASK | L6474_ELPOS_MICROSTEP_MASK);
        }
        break;
      case L6474_ABS_POS:
      case L6474_MARK:
        if ((value >= L6474_MIN_POSITION) &&
            (value <= L6474_MAX_POSITION))
        {
          if (value >= 0)
          {
            registerValue = ((uint32_t) value)& L6474_ABS_POS_VALUE_MASK;
          }
          else
          {
            registerValue = L6474_ABS_POS_VALUE_MASK - (((uint32_t) (-value))& L6474_ABS_POS_VALUE_MASK) + 1;
          }
        }
        else 
        {
          result = FALSE;
        }
        break;
      case L6474_TVAL:
        if (value > L6474_TVAL_MAX_VALUE)
        {
          result = FALSE;
        }
        else
        {
          registerValue = L6474_Tval_Current_to_Par(value);
        }
        break;
      case L6474_OCD_TH:
        if (value > L6474_OCD_TH_MAX_VALUE)
        {
          result = FALSE;
        }
        else 
        {
          registerValue = L6474_Ocd_Th_to_Par(value);
        }
        break;
      case L6474_TON_MIN:
      case L6474_TOFF_MIN:
        if (value > L6474_TOFF_TON_MIN_MAX_VALUE)
        {
          result = FALSE;
        }
        else
        {
          registerValue = L6474_Tmin_Time_to_Par(value);
        }
        break;    
      default:
        result = FALSE;
    }
    if (result != FALSE)
    {
      L6474_CmdSetParam(deviceId, param, registerValue);
    }
  }
  return result;
}

/******************************************************//**
 * @brief  Changes the deceleration of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @param[in] newDec New deceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool L6474_SetDeceleration(uint8_t deviceId, uint16_t newDec)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newDec != 0)&& 
      ((devicePrm[deviceId].motionState == INACTIVE)||
       (devicePrm[deviceId].commandExecuted == RUN_CMD)))
  {
    devicePrm[deviceId].deceleration = newDec;
    cmdExecuted = TRUE;
  }      
  return cmdExecuted;
}        

/******************************************************//**
 * @brief  Specifies the direction 
 * @param[in] deviceId (from 0 to 2)
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device 
 * is in INACTIVE state
 * @retval None
 **********************************************************/
void L6474_SetDirection(uint8_t deviceId, motorDir_t dir)
{
  if (devicePrm[deviceId].motionState == INACTIVE)
  {
    devicePrm[deviceId].direction = dir;
    L6474_Board_SetDirectionGpio(deviceId, dir);
  }
}
/******************************************************//**
 * @brief  Set the Home position
 * (ABS pos set to new home Pos)
 * @param[in] deviceId (from 0 to 2)
 * @param[in] markPos new home position
 * @retval None
 **********************************************************/
void L6474_SetHome(uint8_t deviceId, int32_t homePos)
{
  int32_t absHomePos = L6474_ConvertPosition(L6474_CmdGetParam(deviceId,L6474_ABS_POS)) - homePos;
  uint32_t registerValue;
  
  if (absHomePos >= 0)
  {
    registerValue = ((uint32_t) absHomePos)& L6474_ABS_POS_VALUE_MASK;
  }
  else
  {
    registerValue = L6474_ABS_POS_VALUE_MASK - (((uint32_t) (-absHomePos))& L6474_ABS_POS_VALUE_MASK) + 1;
  }
 
  L6474_CmdSetParam(deviceId, L6474_ABS_POS, registerValue);
}
 
/******************************************************//**
 * @brief  Sets the number of devices to be used 
 * @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval TRUE if successfull, FALSE if failure, attempt to set a number of 
 * devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool L6474_SetNbDevices(uint8_t nbDevices)
{
  if (nbDevices <= MAX_NUMBER_OF_DEVICES)
  {
    l6474DriverInstance = 0;
    numberOfDevices = nbDevices;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

/******************************************************//**
 * @brief  Sets the Mark position 
 * @param[in] deviceId (from 0 to 2)
 * @param[in] markPos new Mark position
 * @retval None
 **********************************************************/
void L6474_SetMark(uint8_t deviceId, int32_t markPos)
{
  uint32_t registerValue;
  if (markPos >= 0)
  {
    registerValue = ((uint32_t) markPos)& L6474_ABS_POS_VALUE_MASK;
  }
  else
  {
    registerValue = L6474_ABS_POS_VALUE_MASK - (((uint32_t) (-markPos))& L6474_ABS_POS_VALUE_MASK) + 1;
  }
  
  L6474_CmdSetParam(deviceId,L6474_MARK, registerValue);
}

/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @param[in] newMaxSpeed New max speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool L6474_SetMaxSpeed(uint8_t deviceId, uint16_t newMaxSpeed)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newMaxSpeed >= L6474_MIN_PWM_FREQ)&&
      (newMaxSpeed <= L6474_MAX_PWM_FREQ) &&
      (devicePrm[deviceId].minSpeed <= newMaxSpeed) &&
      ((devicePrm[deviceId].motionState == INACTIVE)||
       (devicePrm[deviceId].commandExecuted == RUN_CMD)))
  {
    devicePrm[deviceId].maxSpeed = newMaxSpeed;
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}                                                     

/******************************************************//**
 * @brief  Changes the min speed of the specified device
 * @param[in] deviceId (from 0 to 2)
 * @param[in] newMinSpeed New min speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool L6474_SetMinSpeed(uint8_t deviceId, uint16_t newMinSpeed)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newMinSpeed >= L6474_MIN_PWM_FREQ)&&
      (newMinSpeed <= L6474_MAX_PWM_FREQ) &&
      (newMinSpeed <= devicePrm[deviceId].maxSpeed) && 
      ((devicePrm[deviceId].motionState == INACTIVE)||
       (devicePrm[deviceId].commandExecuted == RUN_CMD)))
  {
    devicePrm[deviceId].minSpeed = newMinSpeed;
    cmdExecuted = TRUE;
  }  
  return cmdExecuted;
}                 

/******************************************************//**
 * @brief Select the mode to stop the motor.
 * @param[in] deviceId Unused parameter
 * @param[in] stopMode HOLD_MODE to let power bridge enabled
 * @retval None
 **********************************************************/
void L6474_SetStopMode(uint8_t deviceId, motorStopMode_t stopMode)
{
  devicePrm[deviceId].stopMode = stopMode;
}

/******************************************************//**
 * @brief  Stops the motor by using the device deceleration
 * @param[in] deviceId (from 0 to 2)
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is in INACTIVE state.
 **********************************************************/
bool L6474_SoftStop(uint8_t deviceId)
{	
  bool cmdExecuted = FALSE;
  if (devicePrm[deviceId].motionState != INACTIVE)
  {
    devicePrm[deviceId].commandExecuted = SOFT_STOP_CMD;
    cmdExecuted = TRUE;
  }
  return (cmdExecuted);
}

/******************************************************//**
 * @brief  Locks until the device state becomes Inactive
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_WaitWhileActive(uint8_t deviceId)
 {
	/* Wait while motor is running */
	while (L6474_GetDeviceState(deviceId) != INACTIVE);
}

/**
  * @}
  */

/** @addtogroup L6474_Private_functions
  * @{
  */  

/******************************************************//**
 * @brief  Updates the current speed of the device
 * @param[in] deviceId (from 0 to 2)
 * @param[in] newSpeed in pps
 * @retval None
 **********************************************************/
void L6474_ApplySpeed(uint8_t deviceId, uint16_t newSpeed)
{
  if (newSpeed < L6474_MIN_PWM_FREQ)
  {
    newSpeed = L6474_MIN_PWM_FREQ;  
  }
  if (newSpeed > L6474_MAX_PWM_FREQ)
  {
    newSpeed = L6474_MAX_PWM_FREQ;
  }
  
  devicePrm[deviceId].speed = newSpeed;

  switch (deviceId)
  {
    case  0:
      L6474_Board_Pwm1SetFreq(newSpeed);
      break;
    case 1:
      L6474_Board_Pwm2SetFreq(newSpeed);
      break;
    case 2:
      L6474_Board_Pwm3SetFreq(newSpeed);
      break;
    default:
      break; //ignore error
  }
}

/******************************************************//**
 * @brief  Computes the speed profile according to the number of steps to move
 * @param[in] deviceId (from 0 to 2)
 * @param[in] nbSteps number of steps to perform
 * @retval None
 * @note Using the acceleration and deceleration of the device,
 * this function determines the duration in steps of the acceleration,
 * steady and deceleration phases.
 * If the total number of steps to perform is big enough, a trapezoidal move
 * is performed (i.e. there is a steady phase where the motor runs at the maximum
 * speed.
 * Else, a triangular move is performed (no steady phase: the maximum speed is never
 * reached.
 **********************************************************/
void L6474_ComputeSpeedProfile(uint8_t deviceId, uint32_t nbSteps)
{
  uint32_t reqAccSteps; 
	uint32_t reqDecSteps;
   
  /* compute the number of steps to get the targeted speed */
  uint16_t minSpeed = devicePrm[deviceId].minSpeed;
  reqAccSteps = (devicePrm[deviceId].maxSpeed - minSpeed);
  reqAccSteps *= (devicePrm[deviceId].maxSpeed + minSpeed);
  reqDecSteps = reqAccSteps;
  reqAccSteps /= (uint32_t)devicePrm[deviceId].acceleration;
  reqAccSteps /= 2;

  /* compute the number of steps to stop */
  reqDecSteps /= (uint32_t)devicePrm[deviceId].deceleration;
  reqDecSteps /= 2;

	if(( reqAccSteps + reqDecSteps ) > nbSteps)
	{	
    /* Triangular move  */
    /* reqDecSteps = (Pos * Dec) /(Dec+Acc) */
    uint32_t dec = devicePrm[deviceId].deceleration;
    uint32_t acc = devicePrm[deviceId].acceleration;
    
    reqDecSteps =  ((uint32_t) dec * nbSteps) / (acc + dec);
    if (reqDecSteps > 1)
    {
      reqAccSteps = reqDecSteps - 1;
      if(reqAccSteps == 0)
      {
        reqAccSteps = 1;
      }      
    }
    else
    {
      reqAccSteps = 0;
    }
    devicePrm[deviceId].endAccPos = reqAccSteps;
    devicePrm[deviceId].startDecPos = reqDecSteps;
	}
	else
	{	 
    /* Trapezoidal move */
    /* accelerating phase to endAccPos */
    /* steady phase from  endAccPos to startDecPos */
    /* decelerating from startDecPos to stepsToTake*/
    devicePrm[deviceId].endAccPos = reqAccSteps;
    devicePrm[deviceId].startDecPos = nbSteps - reqDecSteps - 1;
	}
}

/******************************************************//**
 * @brief  Converts the ABS_POSITION register value to a 32b signed integer
 * @param[in] abs_position_reg value of the ABS_POSITION register
 * @retval operation_result 32b signed integer corresponding to the absolute position 
 **********************************************************/
int32_t L6474_ConvertPosition(uint32_t abs_position_reg)
{
  int32_t operation_result;

  if (abs_position_reg & L6474_ABS_POS_SIGN_BIT_MASK) 
  {
    /* Negative register value */
    abs_position_reg = ~abs_position_reg;
    abs_position_reg += 1;

    operation_result = (int32_t) (abs_position_reg & L6474_ABS_POS_VALUE_MASK);
    operation_result = -operation_result;
  } 
  else 
  {
    operation_result = (int32_t) abs_position_reg;
  }
  return operation_result;
}

/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void L6474_ErrorHandler(uint16_t error)
{
  if (errorHandlerCallback != 0)
  {
    (void) errorHandlerCallback(error);
  }
  else   
  {
    while(1)
    {
      /* Infinite loop */
    }
  }
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @retval None
 **********************************************************/
void L6474_FlagInterruptHandler(void)
{
  if (flagInterruptCallback != 0)
  {
    /* Set isr flag */
    isrFlag = TRUE;
    
    flagInterruptCallback();
    
    /* Reset isr flag */
    isrFlag = FALSE;   
  }
}

/******************************************************//**
 * @brief  Sends a command without arguments to the L6474 via the SPI
 * @param[in] deviceId (from 0 to 2)
 * @param[in] param Command to send 
 * @retval None
 **********************************************************/
void L6474_SendCommand(uint8_t deviceId, uint8_t param)
{
  uint32_t i;
  uint8_t spiIndex = numberOfDevices - deviceId - 1;
  bool itDisable = FALSE;  
  
  do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_Board_EnableIrq if disable in previous iteration */
      L6474_Board_EnableIrq();
      itDisable = FALSE;
    }
  
    for (i = 0; i < numberOfDevices; i++)
    {
      spiTxBursts[3][i] = L6474_NOP;     
    }
    spiTxBursts[3][spiIndex] = param;
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  L6474_WriteBytes(&spiTxBursts[3][0], &spiRxBursts[3][0]); 
  
  /* re-enable L6474_Board_EnableIrq after SPI transfers*/
  L6474_Board_EnableIrq();
}

/******************************************************//**
 * @brief  Set the parameters of the device to values of pInitPrm structure
 * @param[in] deviceId (from 0 to 2)
 * @param pInitPrm pointer to a structure containing the initial device parameters 
 * @retval None
 **********************************************************/
void L6474_SetDeviceParamsToGivenValues(uint8_t deviceId, L6474_Init_t *pInitPrm)
{
  devicePrm[deviceId].acceleration = pInitPrm->acceleration_step_s2;
  devicePrm[deviceId].deceleration = pInitPrm->deceleration_step_s2;
  devicePrm[deviceId].maxSpeed = pInitPrm->maximum_speed_step_s;
  devicePrm[deviceId].minSpeed = pInitPrm->minimum_speed_step_s;
  
  devicePrm[deviceId].accu = 0;
  devicePrm[deviceId].currentPosition = 0;
  devicePrm[deviceId].endAccPos = 0;
  devicePrm[deviceId].relativePos = 0;
  devicePrm[deviceId].startDecPos = 0;
  devicePrm[deviceId].stepsToTake = 0;
  devicePrm[deviceId].speed = 0;
  devicePrm[deviceId].commandExecuted = NO_CMD;
  devicePrm[deviceId].direction = FORWARD;
  devicePrm[deviceId].motionState = INACTIVE;  
 
  L6474_SetRegisterToGivenValues(deviceId, pInitPrm);
}

/******************************************************//**
 * @brief  Sets the parameters of the device to predefined values 
 * from l6474_target_config.h
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_SetDeviceParamsToPredefinedValues(uint8_t deviceId)
{
  devicePrm[deviceId].acceleration = L6474_CONF_PARAM_ACC_DEVICE_0;
  devicePrm[deviceId].deceleration = L6474_CONF_PARAM_DEC_DEVICE_0;
  devicePrm[deviceId].maxSpeed = L6474_CONF_PARAM_MAX_SPEED_DEVICE_0;
  devicePrm[deviceId].minSpeed = L6474_CONF_PARAM_MIN_SPEED_DEVICE_0;
  
  devicePrm[deviceId].accu = 0;
  devicePrm[deviceId].currentPosition = 0;
  devicePrm[deviceId].endAccPos = 0;
  devicePrm[deviceId].relativePos = 0;
  devicePrm[deviceId].startDecPos = 0;
  devicePrm[deviceId].stepsToTake = 0;
  devicePrm[deviceId].speed = 0;
  devicePrm[deviceId].commandExecuted = NO_CMD;
  devicePrm[deviceId].direction = FORWARD;
  devicePrm[deviceId].motionState = INACTIVE;
  
  L6474_SetRegisterToPredefinedValues(deviceId);

}

/******************************************************//**
 * @brief  Sets the registers of the L6474 to the given values from pInitPrm
 * @param[in] deviceId (from 0 to 2)
 * @param pInitPrm pointer to a structure containing the initial device parameters 
 * @retval None
 **********************************************************/
void L6474_SetRegisterToGivenValues(uint8_t deviceId, L6474_Init_t *pInitPrm)
{
  L6474_CmdSetParam(deviceId,
                    L6474_ABS_POS,
                    0);
  L6474_CmdSetParam(deviceId,
                    L6474_EL_POS,
                    0);
  L6474_CmdSetParam(deviceId,
                    L6474_MARK,
                    0);
  L6474_SetAnalogValue(deviceId,
                       L6474_TVAL,
                       pInitPrm->torque_regulation_current_mA);
  L6474_CmdSetParam(deviceId,
                    L6474_T_FAST,
                    (uint8_t) pInitPrm->maximum_fast_decay_time |
                    (uint8_t) pInitPrm->fall_time);
  L6474_SetAnalogValue(deviceId,
                       L6474_TON_MIN,
                       pInitPrm->minimum_ON_time_us);
  L6474_SetAnalogValue(deviceId,
                       L6474_TOFF_MIN,
                       pInitPrm->minimum_OFF_time_us);
  L6474_SetAnalogValue(deviceId,
                       L6474_OCD_TH,
                       pInitPrm->overcurrent_threshold);
  L6474_CmdSetParam(deviceId,
                  L6474_STEP_MODE,
                  (uint8_t) pInitPrm->step_selection |
                  (uint8_t) pInitPrm->sync_selection);
  L6474_CmdSetParam(deviceId,
                    L6474_ALARM_EN,
                    pInitPrm->alarm);
  L6474_CmdSetParam(deviceId,
                    L6474_CONFIG,
                    (uint16_t) pInitPrm->clock |
                    (uint16_t) pInitPrm->torque_regulation_method |
                    (uint16_t) pInitPrm->overcurrent_shutwdown |
                    (uint16_t) pInitPrm->slew_rate |
                    (uint16_t) pInitPrm->target_swicthing_period);
  
}


/******************************************************//**
 * @brief  Sets the registers of the L6474 to their predefined values 
 * from l6474_target_config.h
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_SetRegisterToPredefinedValues(uint8_t deviceId)
{
  L6474_CmdSetParam(deviceId,
                    L6474_ABS_POS,
                    0);
  L6474_CmdSetParam(deviceId,
                    L6474_EL_POS,
                    0);
  L6474_CmdSetParam(deviceId,
                    L6474_MARK,
                    0);
  switch (deviceId)
  {
    case 0:
      L6474_CmdSetParam(deviceId,
                        L6474_TVAL,
                        L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_0));
      L6474_CmdSetParam(deviceId,
                              L6474_T_FAST,
                              (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_0 |
                              (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_0);
      L6474_CmdSetParam(deviceId,
                              L6474_TON_MIN,
                              L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_0)
                                );
      L6474_CmdSetParam(deviceId,
                              L6474_TOFF_MIN,
                              L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_0));
      L6474_CmdSetParam(deviceId,
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_0);
      L6474_CmdSetParam(deviceId,
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_0 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_0);
      L6474_CmdSetParam(deviceId,
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_0);
      L6474_CmdSetParam(deviceId,
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_0);
#ifdef L6474_CONF_PARAM_AUTO_HIZ_STOP_DEVICE_0
      L6474_SetStopMode(deviceId, L6474_CONF_PARAM_AUTO_HIZ_STOP_DEVICE_0);
#else
      //to keep compatibility with old l6474_target_config_file.h
      L6474_SetStopMode(deviceId, HOLD_MODE);
#endif      
      break;

#if (MAX_NUMBER_OF_DEVICES > 1)
  case 1:
      L6474_CmdSetParam(deviceId,
                        L6474_TVAL,
                        L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_1));
      L6474_CmdSetParam(deviceId,
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_1 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_1);
      L6474_CmdSetParam(deviceId,
                        L6474_TON_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_1));
      L6474_CmdSetParam(deviceId,
                        L6474_TOFF_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_1));
      L6474_CmdSetParam(deviceId,
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_1);
      L6474_CmdSetParam(deviceId,
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_1 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_1);
      L6474_CmdSetParam(deviceId,
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_1);
      L6474_CmdSetParam(deviceId,
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_1);
#ifdef L6474_CONF_PARAM_AUTO_HIZ_STOP_DEVICE_1
      L6474_SetStopMode(deviceId, L6474_CONF_PARAM_AUTO_HIZ_STOP_DEVICE_1);
#else
      //to keep compatibility with old l6474_target_config_file.h
      L6474_SetStopMode(deviceId, HOLD_MODE);
#endif      
      break;
#endif      
#if (MAX_NUMBER_OF_DEVICES > 2)      
    case 2:
      L6474_CmdSetParam(deviceId,
                        L6474_TVAL,
                        L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_2));
      L6474_CmdSetParam(deviceId,
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_2 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_2);
      L6474_CmdSetParam(deviceId,
                        L6474_TON_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_2));
      L6474_CmdSetParam(deviceId,
                        L6474_TOFF_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_2));
      L6474_CmdSetParam(deviceId,
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_2);
      L6474_CmdSetParam(deviceId,
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_2 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_2);
      L6474_CmdSetParam(deviceId,
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_2);
      L6474_CmdSetParam(deviceId,
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_2);
#ifdef L6474_CONF_PARAM_AUTO_HIZ_STOP_DEVICE_2
      L6474_SetStopMode(deviceId, L6474_CONF_PARAM_AUTO_HIZ_STOP_DEVICE_2);
#else
      //to keep compatibility with old l6474_target_config_file.h
      L6474_SetStopMode(deviceId, HOLD_MODE);
#endif         
      break;
#endif      
    default: ;
  }
}

/******************************************************//**
 * @brief Initialises the bridge parameters to start the movement
 * and enable the power bridge
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void L6474_StartMovement(uint8_t deviceId)  
{
  /* Enable L6474 powerstage */
  L6474_CmdEnable(deviceId);
  if (devicePrm[deviceId].endAccPos != 0)
  {
    devicePrm[deviceId].motionState = ACCELERATING;
  }
  else
  {
    devicePrm[deviceId].motionState = DECELERATING;    
  }
  devicePrm[deviceId].accu = 0;
  devicePrm[deviceId].relativePos = 0;
  L6474_ApplySpeed(deviceId, devicePrm[deviceId].minSpeed);
}

/******************************************************//**
 * @brief  Handles the device state machine at each ste
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 * @note Must only be called by the timer ISR
 **********************************************************/
void L6474_StepClockHandler(uint8_t deviceId)
{
  /* Set isr flag */
  isrFlag = TRUE;
  
  /* Incrementation of the relative position */
  devicePrm[deviceId].relativePos++;

  switch (devicePrm[deviceId].motionState) 
  {
    case ACCELERATING: 
    {
        uint32_t relPos = devicePrm[deviceId].relativePos;
        uint32_t endAccPos = devicePrm[deviceId].endAccPos;
        uint16_t speed = devicePrm[deviceId].speed;
        uint32_t acc = ((uint32_t)devicePrm[deviceId].acceleration << 16);
        
        if ((devicePrm[deviceId].commandExecuted == SOFT_STOP_CMD)||
            ((devicePrm[deviceId].commandExecuted != RUN_CMD)&&  
             (relPos == devicePrm[deviceId].startDecPos)))
        {
          devicePrm[deviceId].motionState = DECELERATING;
          devicePrm[deviceId].accu = 0;
        }
        else if ((speed >= devicePrm[deviceId].maxSpeed)||
                 ((devicePrm[deviceId].commandExecuted != RUN_CMD)&&
                  (relPos == endAccPos)))
        {
          devicePrm[deviceId].motionState = STEADY;
        }
        else
        {
          bool speedUpdated = FALSE;
          /* Go on accelerating */
          if (speed == 0) speed =1;
          devicePrm[deviceId].accu += acc / speed;
          while (devicePrm[deviceId].accu >= (0X10000L))
          {
            devicePrm[deviceId].accu -= (0X10000L);
            speed +=1;
            speedUpdated = TRUE;
          }
          
          if (speedUpdated)
          {
            if (speed > devicePrm[deviceId].maxSpeed)
            {
              speed = devicePrm[deviceId].maxSpeed;
            }    
            devicePrm[deviceId].speed = speed;
            L6474_ApplySpeed(deviceId, devicePrm[deviceId].speed);
          }
        }
        break;
    }
    case STEADY: 
    {
      uint16_t maxSpeed = devicePrm[deviceId].maxSpeed;
      uint32_t relativePos = devicePrm[deviceId].relativePos;
      if  ((devicePrm[deviceId].commandExecuted == SOFT_STOP_CMD)||
           ((devicePrm[deviceId].commandExecuted != RUN_CMD)&&
            (relativePos >= (devicePrm[deviceId].startDecPos))) ||
           ((devicePrm[deviceId].commandExecuted == RUN_CMD)&&
            (devicePrm[deviceId].speed > maxSpeed)))
      {
        devicePrm[deviceId].motionState = DECELERATING;
        devicePrm[deviceId].accu = 0;
      }
      else if ((devicePrm[deviceId].commandExecuted == RUN_CMD)&&
               (devicePrm[deviceId].speed < maxSpeed))
      {
        devicePrm[deviceId].motionState = ACCELERATING;
        devicePrm[deviceId].accu = 0;
      }
      break;
    }
    case DECELERATING: 
    {
      uint32_t relativePos = devicePrm[deviceId].relativePos;
      uint16_t speed = devicePrm[deviceId].speed;
      uint32_t deceleration = ((uint32_t)devicePrm[deviceId].deceleration << 16);
      if (((devicePrm[deviceId].commandExecuted == SOFT_STOP_CMD)&&(speed <=  devicePrm[deviceId].minSpeed))||
          ((devicePrm[deviceId].commandExecuted != RUN_CMD)&&
           (relativePos >= devicePrm[deviceId].stepsToTake)))
      {
        /* Motion process complete */
        L6474_HardStop(deviceId);
      }
      else if ((devicePrm[deviceId].commandExecuted == RUN_CMD)&&
               (speed <= devicePrm[deviceId].maxSpeed))
      {
        devicePrm[deviceId].motionState = STEADY;
      }
      else
      {
        /* Go on decelerating */
        if (speed > devicePrm[deviceId].minSpeed)
        {
          bool speedUpdated = FALSE;
          if (speed == 0) speed =1;
          devicePrm[deviceId].accu += deceleration / speed;
          while (devicePrm[deviceId].accu >= (0X10000L))
          {
            devicePrm[deviceId].accu -= (0X10000L);
            if (speed > 1)
            {  
              speed -=1;
            }
            speedUpdated = TRUE;
          }
        
          if (speedUpdated)
          {
            if (speed < devicePrm[deviceId].minSpeed)
            {
              speed = devicePrm[deviceId].minSpeed;
            }  
            devicePrm[deviceId].speed = speed;
            L6474_ApplySpeed(deviceId, devicePrm[deviceId].speed);
          }
        }
      }
      break;
    }
    default: 
    {
      break;
    }
  }  
  /* Set isr flag */
  isrFlag = FALSE;
}

/******************************************************//**
 * @brief Converts mA in compatible values for OCD_TH register
 * @param[in] Tval
 * @retval OCD_TH values
 **********************************************************/
inline uint8_t L6474_Ocd_Th_to_Par(float Tval)
{
  return ((uint8_t)(((Tval - 375)*0.002666f)+0.5f));
}

/******************************************************//**
 * @brief Converts  OCD_TH register values in mA 
 * @param[in] Par OCD regiser value
 * @retval mA
 **********************************************************/
inline float L6474_Ocd_Par_to_Th(uint8_t Par)
{
  return (((float)(Par + 1))*375.f);
}

/******************************************************//**
 * @brief Converts mA in compatible values for TVAL register 
 * @param[in] Tval
 * @retval TVAL values
 **********************************************************/
inline uint8_t L6474_Tval_Current_to_Par(float Tval)
{
  return ((uint8_t)(((Tval - 31.25f)*0.032f)+0.5f));
}

/******************************************************//**
 * @brief Converts  TVAL register values in mA 
 * @param[in] Par TVAL regiser value
 * @retval mA
 **********************************************************/
inline float L6474_Tval_Par_to_Current(uint8_t Par)
{
  return (((float)(Par + 1))*31.25f);
}

/******************************************************//**
 * @brief Convert TON/TOFF values in time (us)
 * @param[in] Par Values from TON_MIN/TOFF_MIN 
 * @retval time in us
 **********************************************************/
inline float L6474_Tmin_Par_to_Time(uint8_t Par)
{
  return (((float)(Par + 1)) * 0.5f);
}

/******************************************************//**
 * @brief Convert time in us in compatible values 
 * for TON_MIN register
 * @param[in] Tmin
 * @retval TON_MIN values
 **********************************************************/
inline uint8_t L6474_Tmin_Time_to_Par(float Tmin)
{
  return ((uint8_t)(((Tmin - 0.5f)*2.0f)+0.5f));
}

/******************************************************//**
 * @brief  Write and receive a byte via SPI
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @retval None
 **********************************************************/
void L6474_WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
{
  if (L6474_Board_SpiWriteBytes(pByteToTransmit, pReceivedByte, numberOfDevices) != 0)
  {
    L6474_ErrorHandler(L6474_ERROR_1);
  }
  
  if (isrFlag)
  {
    spiPreemtionByIsr = TRUE;
  }
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
