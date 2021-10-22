




/*
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 *
 *    Acknowledgments to the invaluable development, support and guidance by
 *    Marco De Fazio, Giorgio Mariano, Enrico Poli, and Davide Ghezzi
 *    of STMicroelectronics
 *
 *              Motor Control Curriculum Feedback Control System
 *
 * Includes:
 * 		Stepper Motor Control interface based on the IHM01A1 and Nucleo F401RE
 * 		Optical Encoder interface supported by the Nucleo F401RE
 * 		Primary PID controller for Pendulum Angle Control
 * 		Secondary PID controller for Rotor Angle Control
 * 		User interfaces for remote access to system configuration including
 * 				Stepper Motor speed profile, current limits, and others
 * 				Control system parameters
 *
 * @author  William J. Kaiser (UCLA Electrical and Computer Engineering).
 *
 * Includes: Rotor Swing Up innovation by Markus Dauberschmidt
 * Please see https://github.com/OevreFlataeker/steval_edukit_swingup
 *
 * Application based on development by STMicroelectronics as described below
 *
 * @version V1.0
 * @date    May 15th, 2019
 *
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 * @author  IPC Rennes
 * @version V1.10.0
 * @date    March 16th, 2018
 * @brief   This example shows how to use 1 IHM01A1 expansion board
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "main.h"
#include "edukit_system.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/*
 * PID Controller with low pass filter operating on derivative component
 */

__INLINE void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float * current_error,
		float * sample_period, float * Deriv_Filt) {

		float int_term, diff, diff_filt;

	  /* Compute time integral of error by trapezoidal rule */
	  int_term = PID->Ki*(*sample_period)*((*current_error) + PID->state_a[0])/2;

	  /* Compute time derivative of error */
	  diff = PID->Kd*((*current_error) - PID->state_a[0])/(*sample_period);

	  /* Compute first order low pass filter of time derivative */
	  diff_filt = Deriv_Filt[0] * diff
				+ Deriv_Filt[0] * PID->state_a[2]
				- Deriv_Filt[1] * PID->state_a[3];

	  /* Accumulate PID output with Integral, Derivative and Proportional contributions*/

	  PID->control_output = diff_filt + int_term + PID->Kp*(*current_error);

	  /* Update state variables */
	  PID->state_a[1] = PID->state_a[0];
	  PID->state_a[0] = *current_error;
	  PID->state_a[2] = diff;
	  PID->state_a[3] = diff_filt;
	  PID->int_term = int_term;
}


/*
 * Encoder position read (returns signed integer)
 *
 * Includes capability for tracking pendulum excursion for Swing Up control.
 * This is developed and provided by Markus Dauberschmidt.  Please see
 * https://github.com/OevreFlataeker/steval_edukit_swingup
 *
 */

__INLINE int encoder_position_read(int *encoder_position, int encoder_position_init, TIM_HandleTypeDef *htim3) {

	cnt3 = __HAL_TIM_GET_COUNTER(htim3);

	if (cnt3 >= 32768) {
		*encoder_position = (int) (cnt3);
		*encoder_position = *encoder_position - 65536;
	} else {
		*encoder_position = (int) (cnt3);
	}

	range_error = 0;
	if (*encoder_position <= -32768) {
		range_error = -1;
		*encoder_position = -32768;
	}
	if (*encoder_position >= 32767) {
		range_error = 1;
		*encoder_position = 32767;
	}

	*encoder_position = *encoder_position - encoder_position_init;

	/*
	 *  Detect if we passed the bottom, then re-arm peak flag
	 *  oppositeSigns returns true when we pass the bottom position
	 */


	if (oppositeSigns(*encoder_position, previous_encoder_position))
	{
		peaked = 0;
		zero_crossed = 1;
	}

	if (!peaked) // We don't need to evaluate anymore if we hit a maximum when we're still in downward motion and didn't cross the minimum
	{
		// Add global maximum
		if (abs(*encoder_position) >= abs(global_max_encoder_position))
		{
			global_max_encoder_position = *encoder_position;
		}
		// Check if new maximum
		if (abs(*encoder_position) >= abs(max_encoder_position))
		{
			max_encoder_position = *encoder_position;
		}
		else
		{
			// We are at the peak and disable further checks until we traversed the minimum position again
			peaked = 1;
			handled_peak = 0;
		}
	}

	previous_encoder_position = *encoder_position;


	return range_error;
}

/*
 * Returns true if the two arguments have opposite sign, false if not
 * @retval bool.
 * Developed and provided by Markus Dauberschmidt
 */

__INLINE bool oppositeSigns(int x, int y) {
	return ((x ^ y) < 0);
}


/*
 * Rotor position set
 */

void rotor_position_set(void) {
	uint32_t rotor_position_u;
	rotor_position_u = BSP_MotorControl_GetPosition(0);
	BSP_MotorControl_SetHome(0, rotor_position_u);
}


/*
 * Rotor position read (returns signed integer)
 *
 * Returns error if overflow detected
 *
 */

__INLINE int rotor_position_read(int *rotor_position) {
	uint32_t rotor_position_u;
	int range_error;
	rotor_position_u = BSP_MotorControl_GetPosition(0);

	if (rotor_position_u > 2147483648) {
		*rotor_position = (int) (rotor_position_u) - 4294967296;
	} else {
		*rotor_position = (int) (rotor_position_u);
	}
	range_error = 0;
	if (*rotor_position <= -2147483648) {
		range_error = -1;
		*rotor_position = -2147483648;
	}
	if (*rotor_position >= 2147483647) {
		range_error = 1;
		*rotor_position = 2147483647;
	}
	return range_error;
}

/*
 * Single float value read
 */

void read_float(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, float *float_return) {

	int k;

	while (1) {
		*RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		*readBytes = Extract_Msg(RxBuffer, *RxBuffer_ReadIdx, *RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

		if (*readBytes)
		{
			*RxBuffer_ReadIdx = (*RxBuffer_ReadIdx + *readBytes)
											% UART_RX_BUFFER_SIZE;
			*float_return = atof((char*) Msg.Data);
			for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
				Msg.Data[k] = 0;
			}
			*readBytes = 0;
			break;
		}
		HAL_Delay(100);
	}
}

/*
 * Single integer value read
 */

void read_int(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, int * int_return) {

	int k;

	while (1) {
		*RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		*readBytes = Extract_Msg(RxBuffer, *RxBuffer_ReadIdx, *RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

		if (*readBytes)
		{
			*RxBuffer_ReadIdx = (*RxBuffer_ReadIdx + *readBytes)
											% UART_RX_BUFFER_SIZE;

			*int_return = atoi((char*)(Msg.Data));
			for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
				Msg.Data[k] = 0;
			}
			*readBytes = 0;
			break;
		}
		HAL_Delay(100);
	}
}

/*
 * Single character value read
 */

void read_char(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, char * char_return) {

	int k;

	while (1) {
		*RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		*readBytes = Extract_Msg(RxBuffer, *RxBuffer_ReadIdx, *RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

		if (*readBytes)
		{
			*RxBuffer_ReadIdx = (*RxBuffer_ReadIdx + *readBytes)
											% UART_RX_BUFFER_SIZE;
			char_return = (char*)(Msg.Data);
			for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
				Msg.Data[k] = 0;
			}
			*readBytes = 0;
			break;
		}
		HAL_Delay(100);
	}
}

/*
 * Assign operating modes for
 * 			PID Output Feedback Mode 1
 * 			PID Output Feedback Mode 2
 * 			PID Output Feedback Mode 3
 *
 * Mode assignments may be user configured below
 */

void assign_mode_1(arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor){
	select_suspended_mode = 0;
	proportional = PRIMARY_PROPORTIONAL_MODE_1;
	integral = PRIMARY_INTEGRAL_MODE_1;
	derivative = PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;
	PID_Pend->Kp = proportional;
	PID_Pend->Ki = integral;
	PID_Pend->Kd = derivative;
	PID_Rotor->Kp = rotor_p_gain;
	PID_Rotor->Ki = rotor_i_gain;
	PID_Rotor->Kd = rotor_d_gain;
	torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
}

void assign_mode_2(arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor){
	select_suspended_mode = 0;
	proportional = PRIMARY_PROPORTIONAL_MODE_2;
	integral = PRIMARY_INTEGRAL_MODE_2;
	derivative = PRIMARY_DERIVATIVE_MODE_2;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_2;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_2;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_2;
	PID_Pend->Kp = proportional;
	PID_Pend->Ki = integral;
	PID_Pend->Kd = derivative;
	PID_Rotor->Kp = rotor_p_gain;
	PID_Rotor->Ki = rotor_i_gain;
	PID_Rotor->Kd = rotor_d_gain;
	torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
}

void assign_mode_3(arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor){
	select_suspended_mode = 0;
	proportional = PRIMARY_PROPORTIONAL_MODE_3;
	integral = PRIMARY_INTEGRAL_MODE_3;
	derivative = PRIMARY_DERIVATIVE_MODE_3;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_3;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_3;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_3;
	PID_Pend->Kp = proportional;
	PID_Pend->Ki = integral;
	PID_Pend->Kd = derivative;
	PID_Rotor->Kp = rotor_p_gain;
	PID_Rotor->Ki = rotor_i_gain;
	PID_Rotor->Kd = rotor_d_gain;
	torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
}

/*
 * Identify real time user character input and perform configuration
 *
 * Return mode value if identified
 *
 */

int mode_index_identification(char * user_config_input, int config_command_control,
		float *adjust_increment, arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor){

	if (strcmp(user_config_input, mode_string_inc_pend_p) == 0){
		PID_Pend->Kp = PID_Pend->Kp + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_pend_p) == 0) {
		PID_Pend->Kp = PID_Pend->Kp - *adjust_increment;
		//if (PID_Pend->Kp <= 0) { PID_Pend->Kp = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_pend_d) == 0) {
		PID_Pend->Kd = PID_Pend->Kd + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_pend_d) == 0) {
		PID_Pend->Kd = PID_Pend->Kd - *adjust_increment;
		//if (PID_Pend->Kd <= 0) { PID_Pend->Kd = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_pend_i) == 0) {
		PID_Pend->Ki = PID_Pend->Ki + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_pend_i) == 0) {
		PID_Pend->Ki = PID_Pend->Ki - *adjust_increment;
		//if (PID_Pend->Ki <= 0) { PID_Pend->Ki = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_rotor_p) == 0){
		PID_Rotor->Kp = PID_Rotor->Kp + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_rotor_p) == 0) {
		PID_Rotor->Kp = PID_Rotor->Kp - *adjust_increment;
		//if (PID_Rotor->Kp <= 0) { PID_Rotor->Kp = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_rotor_d) == 0) {
		PID_Rotor->Kd = PID_Rotor->Kd + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_rotor_d) == 0) {
		PID_Rotor->Kd = PID_Rotor->Kd - *adjust_increment;
		//if (PID_Rotor->Kd <= 0) { PID_Rotor->Kd = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_rotor_i) == 0) {
		PID_Rotor->Ki = PID_Rotor->Ki + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_rotor_i) == 0) {
		PID_Rotor->Ki = PID_Rotor->Ki - *adjust_increment;
		//if (PID_Rotor->Ki <= 0) { PID_Rotor->Ki = 0; }
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_torq_c) == 0) {
		torq_current_val = L6474_GetAnalogValue(0, L6474_TVAL);
		torq_current_val = torq_current_val - *adjust_increment;
		if (torq_current_val < 200){ torq_current_val = 200; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_torq_c) == 0) {
		torq_current_val = L6474_GetAnalogValue(0, L6474_TVAL);
		torq_current_val = torq_current_val + *adjust_increment;
		if (torq_current_val > MAX_TORQUE_CONFIG){ torq_current_val = MAX_TORQUE_CONFIG; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_max_s) == 0) {
		max_speed = L6474_GetMaxSpeed(0);
		max_speed = max_speed - *adjust_increment;
		if (max_speed < 100){ max_speed = 100; }
		if (max_speed < min_speed){ max_speed = min_speed;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMaxSpeed(0, max_speed);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_max_s) == 0) {
		max_speed = L6474_GetMaxSpeed(0);
		max_speed = max_speed + *adjust_increment;
		if (max_speed > 1000){ max_speed = 1000; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMaxSpeed(0, max_speed);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_min_s) == 0) {
		min_speed = L6474_GetMinSpeed(0);
		min_speed = min_speed - *adjust_increment;
		if (min_speed < 100){ min_speed = 100; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMinSpeed(0, min_speed);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_min_s) == 0) {
		min_speed = L6474_GetMinSpeed(0);
		min_speed = min_speed + *adjust_increment;
		if (min_speed > 1000){ min_speed = 1000; }
		if (min_speed > max_speed){ min_speed = max_speed;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMinSpeed(0, min_speed);
		config_command = 1;
		mode_index_command = -1;
	} else if (strcmp(user_config_input, mode_string_dec_max_a) == 0) {
		max_accel = L6474_GetAcceleration(0);
		max_accel = max_accel - *adjust_increment;
		if (max_accel <  0){ max_accel = 0;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAcceleration(0, max_accel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_max_a) == 0) {
		max_accel = L6474_GetAcceleration(0);
		max_accel = max_accel + *adjust_increment;
		if (max_accel >  10000){ max_accel = 10000;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAcceleration(0, max_accel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_max_d) == 0) {
		max_decel = L6474_GetDeceleration(0);
		max_decel = max_decel - *adjust_increment;
		if (max_decel <  0){ max_decel = 0;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetDeceleration(0, max_decel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_max_d) == 0) {
		max_decel = L6474_GetDeceleration(0);
		max_decel = max_decel + *adjust_increment;
		if (max_decel > 10000) { max_decel = 10000; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetDeceleration(0, max_decel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_select_mode_5) == 0) {
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetDeceleration(0, MAX_DECEL);
		L6474_SetAcceleration(0, MAX_ACCEL);
		L6474_SetMinSpeed(0, MIN_SPEED_MODE_5);
		L6474_SetMaxSpeed(0, MAX_SPEED_MODE_5);
		PID_Pend->Kp = PRIMARY_PROPORTIONAL_MODE_5;
		PID_Pend->Ki = PRIMARY_INTEGRAL_MODE_5;
		PID_Pend->Kd = PRIMARY_DERIVATIVE_MODE_5;
		PID_Rotor->Kp = SECONDARY_PROPORTIONAL_MODE_5;
		PID_Rotor->Ki = SECONDARY_INTEGRAL_MODE_5;
		PID_Rotor->Kd = SECONDARY_DERIVATIVE_MODE_5;
		enable_adaptive_mode = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_step ) == 0 ){
		enable_rotor_position_step_response_cycle = 1;
		enable_sensitivity_fnc_step = 0;
		enable_noise_rejection_step = 0;
		enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_step ) == 0 ){
		enable_rotor_position_step_response_cycle = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_pendulum_impulse) == 0 ){
		enable_pendulum_position_impulse_response_cycle = 1;
		enable_rotor_position_step_response_cycle = 0;
		enable_sensitivity_fnc_step = 0;
		enable_noise_rejection_step = 0;
		enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_pendulum_impulse ) == 0 ){
		enable_pendulum_position_impulse_response_cycle = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_noise_rej_step ) == 0 ){
		enable_noise_rejection_step = 1;
		enable_rotor_position_step_response_cycle = 1;
		enable_disturbance_rejection_step = 0;
		enable_sensitivity_fnc_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_noise_rej_step ) == 0 ){
		enable_noise_rejection_step = 0;
		enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_sensitivity_fnc_step ) == 0 ){
		enable_sensitivity_fnc_step = 1;
		enable_rotor_position_step_response_cycle = 1;
		enable_disturbance_rejection_step = 0;
		enable_noise_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_sensitivity_fnc_step ) == 0 ){
		enable_sensitivity_fnc_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_load_dist ) == 0 ){
		enable_sensitivity_fnc_step = 0;
		enable_rotor_position_step_response_cycle = 1;
		enable_disturbance_rejection_step = 1;
		enable_noise_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_load_dist ) == 0 ){
		enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_step_size ) == 0 ){
		step_size = step_size + 1;
		if (step_size > 4) { step_size = 4; }
		if (step_size == 0) { *adjust_increment = 2;}
		else if (step_size == 1) { *adjust_increment = 5;}
		else if (step_size == 2) { *adjust_increment = 20;}
		else if (step_size == 3) { *adjust_increment = 50;}
		else if (step_size == 4) { *adjust_increment = 100;}
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_step_size ) == 0 ){
		step_size = step_size - 1;
		if (step_size < 0) { step_size = 0; }
		if (step_size == 0) { *adjust_increment = 2;}
		else if (step_size == 1) { *adjust_increment = 5;}
		else if (step_size == 2) { *adjust_increment = 20;}
		else if (step_size == 3) { *adjust_increment = 50;}
		else if (step_size == 4) { *adjust_increment = 100;}
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_high_speed_sampling ) == 0 ){
		enable_high_speed_sampling = 1;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_high_speed_sampling ) == 0 ){
		enable_high_speed_sampling = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_speed_prescale ) == 0 ){
		enable_speed_prescale = 1;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_speed_prescale ) == 0 ){
		enable_speed_prescale = 0;
		config_command = 1;
	} else if  (strcmp(user_config_input, mode_string_disable_speed_governor ) == 0 ){
		speed_governor = 0;
		config_command = 1;
	} else if  (strcmp(user_config_input, mode_string_enable_speed_governor ) == 0 ){
		speed_governor = 1;
		config_command = 1;
	} else {
		mode_index_command = atoi((char*) Msg.Data);
	}
	/*
	 * Disable sin drive tracking
	 */
	if (mode_index_command == mode_9){
		disable_mod_sin_rotor_tracking = 1;
		sine_drive_transition = 1;
		mode_index_command = -1;
	}
	if (config_command_control == 0){
		/*
		 * Enable sin drive tracking
		 */
		if (mode_index_command == mode_5){
			disable_mod_sin_rotor_tracking = 0;
			sine_drive_transition = 1;
			mode_index_command = -1;
		}
	}
	return mode_index_command;
}


/*
 * Initialize mode identification strings
 */

void set_mode_strings(void){
	/*
	 * Set user interactive command string values
	 */

	sprintf(mode_string_mode_1, "1");
	sprintf(mode_string_mode_2, "2");
	sprintf(mode_string_mode_3, "3");
	sprintf(mode_string_mode_4, "4");
	sprintf(mode_string_mode_5, "m");
	sprintf(mode_string_mode_8, "g");
	sprintf(mode_string_mode_single_pid, "s");
	sprintf(mode_string_mode_test, "t");
	sprintf(mode_string_mode_control, "r");
	sprintf(mode_string_mode_motor_characterization_mode, "c");
	sprintf(mode_string_mode_full_sysid, "I");
	sprintf(mode_string_dec_accel, "d");
	sprintf(mode_string_inc_accel, "i");
	sprintf(mode_string_inc_amp, "j");
	sprintf(mode_string_dec_amp, "k");
	sprintf(mode_string_stop, "q");


	sprintf(mode_string_dec_pend_p, "a");
	sprintf(mode_string_inc_pend_p, "A");
	sprintf(mode_string_dec_pend_i, "b");
	sprintf(mode_string_inc_pend_i, "B");
	sprintf(mode_string_dec_pend_d, "c");
	sprintf(mode_string_inc_pend_d, "C");
	sprintf(mode_string_dec_rotor_p, "d");
	sprintf(mode_string_inc_rotor_p, "D");
	sprintf(mode_string_dec_rotor_i, "e");
	sprintf(mode_string_inc_rotor_i, "E");
	sprintf(mode_string_dec_rotor_d, "f");
	sprintf(mode_string_inc_rotor_d, "F");
	sprintf(mode_string_dec_torq_c, "t");
	sprintf(mode_string_inc_torq_c, "T");
	sprintf(mode_string_dec_max_s, "s");
	sprintf(mode_string_inc_max_s, "S");
	sprintf(mode_string_dec_min_s, "m");
	sprintf(mode_string_inc_min_s, "M");
	sprintf(mode_string_dec_max_a, "n");
	sprintf(mode_string_inc_max_a, "N");
	sprintf(mode_string_dec_max_d, "o");
	sprintf(mode_string_inc_max_d, "O");
	sprintf(mode_string_enable_step, "P");
	sprintf(mode_string_disable_step, "p");
	sprintf(mode_string_enable_pendulum_impulse, "H");
	sprintf(mode_string_disable_pendulum_impulse, "h");
	sprintf(mode_string_enable_load_dist, "L");
	sprintf(mode_string_disable_load_dist, "l");
	sprintf(mode_string_enable_noise_rej_step, "R");
	sprintf(mode_string_disable_noise_rej_step, "r");
	sprintf(mode_string_enable_sensitivity_fnc_step, "V");
	sprintf(mode_string_disable_sensitivity_fnc_step, "v");
	sprintf(mode_string_inc_step_size, "J");
	sprintf(mode_string_dec_step_size, "j");
	sprintf(mode_string_select_mode_5, "u");
	sprintf(mode_string_enable_high_speed_sampling, "Y");
	sprintf(mode_string_disable_high_speed_sampling, "y");
	sprintf(mode_string_enable_speed_prescale, ":");
	sprintf(mode_string_disable_speed_prescale, ";");
	sprintf(mode_string_enable_speed_governor, "(");
	sprintf(mode_string_disable_speed_governor, ")");
	sprintf(mode_string_reset_system, "<");


	mode_1 = 1;				// Enable Model 1Enable Suspended Mode Motor Model M
	mode_2 = 2;				// Enable Suspended Mode Motor Model M
	mode_3 = 3;				// Enable Model 2
	mode_4 = 4;				// Enable Model 3
	mode_5 = 5;				// Enable sin drive track signal
	mode_adaptive_off = 6;	// Disable adaptive control
	mode_adaptive = 7;		// Enable adaptive control
	mode_8 = 8;				// Enable custom configuration entry
	mode_9 = 9;				// Disable sin drive track signal
	mode_10 = 10;			// Enable Single PID Mode with Motor Model M
	mode_11 = 11;			// Enable rotor actuator and encoder test mode
	mode_13 = 13;			// Enable rotor control system evaluation
	mode_15 = 15;			// Enable interactive control of rotor actuator
	mode_16 = 16;			// Enable load disturbance function mode
	mode_17 = 17;			// Enable noise disturbance function step mode
	mode_18 = 18;			// Enable sensitivity function step mode
	mode_19 = 19;           // Enable full system identification mode
	mode_quit = 0;			// Initiate exit from control loop
}

/*
 * Print user prompt for serial interface
 */

void user_prompt(void){
	sprintf(msg, "\n\r********  System Start Mode Selections  ********\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 1 at prompt for Inverted Pendulum Control............................... \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 2 at prompt for Suspended Pendulum Control.............................. \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 's' at prompt for Single PID Pendulum Controller Gain Entry ............ \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 'g' at prompt for General Mode: Full State Feedback and PID Controller.. \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 't' at prompt for Test of Rotor Actuator and Pendulum Angle Encoder..... \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "Enter 'r' at prompt for Direct Control of Rotor Actuator Operation............ \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

/*
 * Return selected user mode
 */

void get_user_mode_index(char * user_string, int * char_mode_select, int * mode_index, int * mode_interactive){

	*char_mode_select = 0;

	if (strcmp(user_string,mode_string_mode_single_pid)==0){
		*mode_index = 10;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_test)==0){
		*mode_index = 11;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_control)==0){
		*mode_index = 15;
		*char_mode_select = 1;
	}

	/* Motor Characterization Option */

	if (strcmp(user_string,mode_string_mode_motor_characterization_mode)==0){
		*mode_index = 13;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_8)==0){
		*mode_index = 8;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_full_sysid)==0){
		*mode_index = 19;
		*char_mode_select = 1;
	}

	if(*char_mode_select == 0){
		*mode_index = atoi(user_string);
	}


	/*
	 * Set mode index according to user input
	 */

	switch (*mode_index) {

	case 1:
		*mode_index = mode_1;
		break;

	case 2:
		*mode_index = mode_2;
		break;

	case 8:
		*mode_index = mode_8;
		*mode_interactive = 1;
		break;

	case 10:
		*mode_index = mode_10;
		*mode_interactive = 1;
		break;

	case 11:
		*mode_index = mode_11;
		*mode_interactive = 1;
		break;

	case 13:
		*mode_index = mode_13;
		*mode_interactive = 1;
		break;

	case 15:
		*mode_index = mode_15;
		*mode_interactive = 1;
		break;

	case 19:
		*mode_index = mode_19;
		*mode_interactive = 1;
		break;

	default:
		*mode_index = mode_1;
		break;
	}

}

/*
 * Configure system based on user selection
 */

void user_configuration(void){

	enable_rotor_actuator_test = 0;
	enable_rotor_actuator_control = 0;
	enable_encoder_test = 0;
	enable_rotor_actuator_high_speed_test = 0;
	enable_motor_actuator_characterization_mode = 0;
	enable_full_sysid = 0;

	enable_rotor_tracking_comb_signal = 0;
	rotor_track_comb_amplitude = 0;
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;


	while (1){
		RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE
				- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

		readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx,
				RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);


		/*
		 * Exit read loop after timeout selecting default Mode 1
		 */

		tick_read_cycle = HAL_GetTick();
		if (((tick_read_cycle - tick_read_cycle_start) > START_DEFAULT_MODE_TIME) && (mode_interactive == 0)) {
			sprintf(msg, "\n\rNo Entry Detected - Now Selecting Default Inverted Pendulum Mode 1......: \n\r");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			enable_state_feedback = 0;
			select_suspended_mode = 0;
			proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
			integral = 			PRIMARY_INTEGRAL_MODE_1;
			derivative = 		PRIMARY_DERIVATIVE_MODE_1;
			rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
			rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
			rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
			max_speed = 		MAX_SPEED_MODE_1;
			min_speed = 		MIN_SPEED_MODE_1;
			enable_rotor_plant_design = 0;
			enable_rotor_plant_gain_design = 0;
			enable_rotor_position_step_response_cycle = 0;
			enable_pendulum_position_impulse_response_cycle = 0;
			enable_rotor_chirp = 0;
			enable_mod_sin_rotor_tracking = 1;
			enable_angle_cal = 1;
			enable_swing_up = 1;
			L6474_SetAnalogValue(0, L6474_TVAL, TORQ_CURRENT_DEFAULT);
			break;
		}

		if (readBytes) // Message found
		{
			RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;

			if (Msg.Len != 1) {
				continue;
			}

			sprintf(msg, "%s", (char*)Msg.Data);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			get_user_mode_index((char*)Msg.Data, &char_mode_select, &mode_index, &mode_interactive);

			/*
			 * Configure Motor Speed Profile and PID Controller Gains
			 */

			enable_rotor_position_step_response_cycle = 0;
			enable_mod_sin_rotor_tracking = 0;
			enable_rotor_chirp = 0;
			enable_pendulum_position_impulse_response_cycle = 0;
			enable_pendulum_sysid_test = 0;
			enable_full_sysid = 0;
			enable_rotor_tracking_comb_signal = 0;
			enable_disturbance_rejection_step = 0;
			enable_noise_rejection_step = 0;
			enable_sensitivity_fnc_step = 0;
			enable_rotor_plant_design = 0;
			enable_rotor_plant_gain_design = 0;


			switch (mode_index) {

			/* Mode 1 selection */

			case 1:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				enable_state_feedback = 0;
				select_suspended_mode = 0;
				proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
				integral = 			PRIMARY_INTEGRAL_MODE_1;
				derivative = 		PRIMARY_DERIVATIVE_MODE_1;
				rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
				rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
				rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
				max_speed = 		MAX_SPEED_MODE_1;
				min_speed = 		MIN_SPEED_MODE_1;
				enable_rotor_plant_design = 0;
				enable_rotor_plant_gain_design = 0;

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart..... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				enable_mod_sin_rotor_tracking = 0;
				enable_rotor_position_step_response_cycle = 0;
				enable_pendulum_position_impulse_response_cycle = 0;
				enable_rotor_chirp = 0;

				sprintf(msg, "\n\rMode %i Configured\n\r", mode_index);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg), HAL_MAX_DELAY);

				enable_angle_cal = 0;
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable.....................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				enable_swing_up = ENABLE_SWING_UP;
				enable_swing_up_resp = 0;
				sprintf(msg, "\n\rSwing Up Enabled - Enter 1 to Disable.......................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_swing_up_resp);
				if (enable_swing_up_resp == 1){
					enable_swing_up = 0;
				}
				sprintf(msg, "%i", enable_swing_up_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_swing_up_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable...........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable..................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					if (enable_rotor_tracking_comb_signal == 1){
						rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}
				}

				if (enable_rotor_position_step_response_cycle == 1) {
					enable_pendulum_position_impulse_response_cycle = 0;
				}

				if (enable_mod_sin_rotor_tracking == 1) {
					enable_pendulum_position_impulse_response_cycle = 0;
				}

				if (enable_rotor_chirp == 1) {
					enable_rotor_position_step_response_cycle = 0;
					enable_pendulum_position_impulse_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 0;
				}

				if (enable_rotor_tracking_comb_signal == 1) {
					enable_rotor_position_step_response_cycle = 0;
					enable_pendulum_position_impulse_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 0;
				}

				break;

				/* Mode 3 selection */

			case 3:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				enable_state_feedback = 0;
				select_suspended_mode = 0;
				proportional = 		PRIMARY_PROPORTIONAL_MODE_2;
				integral = 			PRIMARY_INTEGRAL_MODE_2;
				derivative = 		PRIMARY_DERIVATIVE_MODE_2;
				rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_2;
				rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_2;
				rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_2;
				max_speed = 		MAX_SPEED_MODE_2;
				min_speed = 		MIN_SPEED_MODE_2;
				enable_rotor_plant_design = 0;
				enable_rotor_plant_gain_design = 0;

				enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
				enable_rotor_position_step_response_cycle = 0;
				sprintf(msg, "\n\rMode %i Configured", mode_index);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart..... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				enable_angle_cal = 0;
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable.....................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}



				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable...........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}
				if ( enable_mod_sin_rotor_tracking < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable..................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					if (enable_rotor_tracking_comb_signal == 1){
						rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}
				}

				if (enable_rotor_position_step_response_cycle == 1) {
					sprintf(msg, "\n\rRotor Step Drive enabled ");
					enable_pendulum_position_impulse_response_cycle = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				if (enable_mod_sin_rotor_tracking == 1) {
					sprintf(msg, "\n\rRotor Sine Drive enabled ");
					enable_pendulum_position_impulse_response_cycle = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				if (enable_rotor_chirp == 1) {
					enable_rotor_position_step_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 0;
					enable_pendulum_position_impulse_response_cycle = 0;
					sprintf(msg, "\n\rRotor Chirp Drive enabled ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}
				break;

				/* Mode 4 selection */

			case 4:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				enable_state_feedback = 0;
				select_suspended_mode = 0;
				proportional = 		PRIMARY_PROPORTIONAL_MODE_3;
				integral = 			PRIMARY_INTEGRAL_MODE_3;
				derivative = 		PRIMARY_DERIVATIVE_MODE_3;
				rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_3;
				rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_3;
				rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_3;
				max_speed = 		MAX_SPEED_MODE_3;
				min_speed = 		MIN_SPEED_MODE_3;
				enable_rotor_plant_design = 0;
				enable_rotor_plant_gain_design = 0;

				enable_mod_sin_rotor_tracking = 0;
				enable_rotor_position_step_response_cycle = 0;
				sprintf(msg, "\n\rMode %i Configured", mode_index);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart..... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				enable_angle_cal = 0;
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable.................. ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				enable_swing_up = ENABLE_SWING_UP;
				enable_swing_up_resp = 0;
				sprintf(msg, "\n\rSwing Up Enabled - Enter 1 to Disable.......................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_swing_up_resp);
				if (enable_swing_up_resp == 1){
					enable_swing_up = 0;
				}
				sprintf(msg, "%i", enable_swing_up_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_swing_up_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable...........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable..................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					if (enable_rotor_tracking_comb_signal == 1){
						rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}
				}

				if (enable_rotor_position_step_response_cycle == 1) {
					enable_pendulum_position_impulse_response_cycle = 0;
				}

				if (enable_mod_sin_rotor_tracking == 1) {
					enable_pendulum_position_impulse_response_cycle = 0;
				}

				if (enable_rotor_chirp == 1) {
					enable_rotor_position_step_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 0;
					enable_pendulum_position_impulse_response_cycle = 0;
				}
				break;

				/* Mode 2 Suspended Mode selection */
			case 2:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				enable_state_feedback = 0;
				select_suspended_mode = 1;
				proportional = 		PRIMARY_PROPORTIONAL_MODE_4;
				integral = 			PRIMARY_INTEGRAL_MODE_4;
				derivative = 		PRIMARY_DERIVATIVE_MODE_4;
				rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_4;
				rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_4;
				rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_4;
				max_speed = 		MAX_SPEED_MODE_1;
				min_speed = 		MIN_SPEED_MODE_1;
				enable_rotor_plant_design = 0;
				enable_rotor_plant_gain_design = 0;

				enable_mod_sin_rotor_tracking = 0;
				enable_rotor_position_step_response_cycle = 0;
				sprintf(msg, "\n\rMode %i Configured", mode_index);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart..... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				enable_angle_cal = 0;
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable.....................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}


				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable...........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable..................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					if (enable_rotor_tracking_comb_signal == 1){
						rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}
				}

				if (enable_rotor_position_step_response_cycle == 1) {
					sprintf(msg, "\n\rRotor Step Drive enabled ");
					enable_pendulum_position_impulse_response_cycle = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				if (enable_mod_sin_rotor_tracking == 1) {
					sprintf(msg, "\n\rRotor Sine Drive enabled ");
					enable_pendulum_position_impulse_response_cycle = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				if (enable_rotor_chirp == 1) {
					enable_rotor_position_step_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 0;
					enable_pendulum_position_impulse_response_cycle = 0;
					sprintf(msg, "\n\rRotor Chirp Drive enabled ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}
				break;


				/* Adaptive Mode selection - currently disabled */
			case 7:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				break;

				/* General mode selection requiring user specification of all configurations */
			case 8:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart..... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\rEnter 0 for Dual PID - Enter 1 for State Feedback...........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_state_feedback);
				sprintf(msg, "%i", enable_state_feedback);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_state_feedback < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_state_feedback == 1){

					/*
					 * State feedback includes only proportional and derivative gains
					 * State feedback also includes optional integral compensator gain
					 */

					integral = 0;
					rotor_i_gain = 0;
					feedforward_gain = 1;

					sprintf(msg, "\n\rEnter Pendulum Angle Gain...................................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &proportional);
					sprintf(msg, "%0.2f", proportional);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( proportional < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Pendulum Angle Derivative Gain........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &derivative);
					sprintf(msg, "%0.2f", derivative);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( derivative < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor Angle Gain......................................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_p_gain);
					sprintf(msg, "%0.2f", rotor_p_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_p_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor Angle Derivative Gain...........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_d_gain);
					sprintf(msg, "%0.2f", rotor_d_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_d_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Integral Compensator Gain (zero to disable)...........................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &integral_compensator_gain);
					sprintf(msg, "%0.2f", integral_compensator_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( integral_compensator_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Feedforward Gain (return or zero to set to unity).....................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &feedforward_gain);
					if (feedforward_gain == 0){
						feedforward_gain = 1;
					}
					sprintf(msg, "%0.2f", feedforward_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

					proportional = proportional * FULL_STATE_FEEDBACK_SCALE;
					derivative = derivative * FULL_STATE_FEEDBACK_SCALE;
					rotor_p_gain = rotor_p_gain * FULL_STATE_FEEDBACK_SCALE;
					rotor_d_gain = rotor_d_gain * FULL_STATE_FEEDBACK_SCALE;
					integral_compensator_gain = integral_compensator_gain * FULL_STATE_FEEDBACK_SCALE;
					feedforward_gain = feedforward_gain * FULL_STATE_FEEDBACK_SCALE;
				}

				if (enable_state_feedback == 0){

					sprintf(msg, "\n\rEnter Pendulum PID Proportional Gain........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);

					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &proportional);
					sprintf(msg, "%0.2f", proportional);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( proportional < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Pendulum PID Integral Gain............................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &integral);
					sprintf(msg, "%0.2f", integral);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( integral < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Pendulum PID Differential Gain........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &derivative);
					sprintf(msg, "%0.2f", derivative);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( derivative < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor PID Proportional Gain...........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);

					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_p_gain);
					sprintf(msg, "%0.2f", rotor_p_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_p_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor PID Integral Gain:..............................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_i_gain);
					sprintf(msg, "%0.2f", rotor_i_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_i_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor PID Differential Gain...........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_d_gain);
					sprintf(msg, "%0.2f", rotor_d_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_d_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}


				}

				select_suspended_mode = 0;

				sprintf(msg, "\n\rEnter 0 for Inverted Mode - Enter 1 for Suspended Mode......................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_suspended_mode);
				sprintf(msg, "%i", select_suspended_mode);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( select_suspended_mode < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if ( select_suspended_mode == 1 ){
					enable_angle_cal = 0;
				}

				enable_pendulum_position_impulse_response_cycle = 0;
				enable_rotor_position_step_response_cycle = 0;
				enable_mod_sin_rotor_tracking = 0;
				enable_rotor_chirp = 0;

				enable_angle_cal = 0;
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable.....................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if ( select_suspended_mode == 0 ){

					enable_swing_up = ENABLE_SWING_UP;
					enable_swing_up_resp = 0;
					sprintf(msg, "\n\rSwing Up Enabled - Enter 1 to Disable:......................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_swing_up_resp);
					if (enable_swing_up_resp == 1){
						enable_swing_up = 0;
					}
					sprintf(msg, "%i", enable_swing_up_resp);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_swing_up_resp < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable...........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable..................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable..................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					if (enable_rotor_tracking_comb_signal == 1){
						rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
					}
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0 && enable_rotor_tracking_comb_signal == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Pendulum Impulse Signal; 0 to Disable.....................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_pendulum_position_impulse_response_cycle);
					sprintf(msg, "%i", enable_pendulum_position_impulse_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_pendulum_position_impulse_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				if (enable_pendulum_position_impulse_response_cycle == 1) {
				}

				if (enable_rotor_position_step_response_cycle == 1) {
				}

				if (enable_mod_sin_rotor_tracking == 1) {
				}

				if (enable_rotor_chirp == 1) {
					enable_rotor_position_step_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 0;
				}

				if (enable_rotor_tracking_comb_signal == 1) {
					enable_rotor_position_step_response_cycle = 0;
					enable_pendulum_position_impulse_response_cycle = 0;
					enable_mod_sin_rotor_tracking = 0;
				}

				enable_disturbance_rejection_step = 0;
				enable_noise_rejection_step = 0;
				enable_sensitivity_fnc_step = 0;

				sprintf(msg, "\n\rEnter 1 to Enable Disturbance Rejection Sensitivity; 0 to Disable...........: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_disturbance_rejection_step);
				sprintf(msg, "%i", enable_disturbance_rejection_step);
				if ( enable_disturbance_rejection_step < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if (enable_disturbance_rejection_step == 1) {
					enable_sensitivity_fnc_step = 0;
					enable_rotor_position_step_response_cycle = 1;
					enable_noise_rejection_step = 0;
				}



				if (enable_disturbance_rejection_step == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Noise Rejection Sensitivity; 0 to Disable.................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_noise_rejection_step);
					sprintf(msg, "%i", enable_noise_rejection_step);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_noise_rejection_step < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
					if (enable_noise_rejection_step == 1) {
						enable_sensitivity_fnc_step = 0;
						enable_rotor_position_step_response_cycle = 1;
						enable_disturbance_rejection_step = 0;
					}
				}

				if (enable_noise_rejection_step == 0 && enable_disturbance_rejection_step == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Sensitivity Function; 0 to Disable........................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_sensitivity_fnc_step);
					sprintf(msg, "%i", enable_sensitivity_fnc_step);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_sensitivity_fnc_step < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
					if (enable_sensitivity_fnc_step == 1) {
						enable_rotor_position_step_response_cycle = 1;
						enable_disturbance_rejection_step = 0;
						enable_noise_rejection_step = 0;
					}
				}

				/*
				 * Reverse polarity of gain values to account for suspended mode angle configuration
				 */

				if(select_suspended_mode == 1){
					proportional = 	-proportional;
					integral = 		-integral;
					derivative = 	-derivative;
					rotor_p_gain = 	-rotor_p_gain;
					rotor_i_gain = 	-rotor_i_gain;
					rotor_d_gain = 	-rotor_d_gain;
					integral_compensator_gain = -integral_compensator_gain;
				}

				/*
				 * Rotor Plant Design is enabled for selection of Full State
				 * Feedback and Integral Action
				 */


				select_rotor_plant_design = 0;
				enable_rotor_plant_design = 0;
				enable_rotor_plant_gain_design = 0;

				/*
				 * Optional addition of second order rotor plant gain specification
				 */

				/*

				if (enable_rotor_plant_gain_design == 0 && enable_state_feedback == 1 && abs(integral_compensator_gain) > 0 ){
					sprintf(msg, "\n\rEnter 1 for Rotor Plant Design Grotor = Wn^2/(s^2 + 2D*s + Wn^2) ...........: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_rotor_plant_design);
					if (select_rotor_plant_design == 1) {
						enable_rotor_plant_design = 1;
					}
					if (enable_rotor_plant_design == 1) {
						if (select_suspended_mode == 1){
							sprintf(msg, "\n\rEnter Natural Frequency (rad/sec) of Minimum 0.5 and Maximum 2 .............: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_natural_frequency);
							if (rotor_natural_frequency > 2){
								rotor_natural_frequency = 2;
							}
							if (rotor_natural_frequency < 0.5){
								rotor_natural_frequency = 0.5;
							}

							sprintf(msg, "%0.2f", rotor_natural_frequency);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

							sprintf(msg, "\n\rEnter Rotor Damping Coefficient of Minimum 0.1 and Maximum 5................: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_damping_coefficient);
							if (rotor_damping_coefficient > 5){
								rotor_damping_coefficient = 5;
							}

							if (rotor_damping_coefficient < 0.1){
								rotor_damping_coefficient = 0.1;
							}

							sprintf(msg, "%0.2f", rotor_damping_coefficient);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

							rotor_plant_gain = 1;

						}

						if (select_suspended_mode == 0 ){
							sprintf(msg, "\n\rEnter Natural Frequency (rad/sec) of Minimum 0.5 and Maximum 2..............: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_natural_frequency);
							if (rotor_natural_frequency > 5){
								rotor_natural_frequency = 5;
							}
							if (rotor_natural_frequency <= 0.5){
								rotor_natural_frequency = 0.5;
							}

							sprintf(msg, "%0.2f", rotor_natural_frequency);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

							sprintf(msg, "\n\rEnter Rotor Damping Coefficient of Minimum 0.5 and Maximum 5 .............. : ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_damping_coefficient);
							if (rotor_damping_coefficient > 5){
								rotor_damping_coefficient = 5;
							}

							if (rotor_damping_coefficient < 0.5){
								rotor_damping_coefficient = 0.5;
							}
							sprintf(msg, "%0.2f", rotor_damping_coefficient);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

							rotor_plant_gain = 1;
						}
					}
				}

				*/



				if (enable_rotor_plant_gain_design == 0 && enable_rotor_plant_design == 0 && enable_state_feedback == 1){

					/* Optional addition of transfer function design
					sprintf(msg, "\n\rEnter 1 for Rotor Plant Design Grotor = Wn/(s^3 + Wn*s^2): ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_rotor_plant_design);
					sprintf(msg, "%i", select_rotor_plant_design);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if (select_rotor_plant_design == 1) {
						enable_rotor_plant_design = 3;
					}
					 */

					sprintf(msg, "\n\rEnter 1 for Rotor Plant Design Grotor = 1/(s^2 + Wn*s) .....................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_rotor_plant_design);
					sprintf(msg, "%i", select_rotor_plant_design);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( select_rotor_plant_design < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
					/* Configure trotor plant design Grotor = 1/(s^2 + Wn*s) with identifier 2 */
					if (select_rotor_plant_design == 1) {
						enable_rotor_plant_design = 2;
					}


					if (enable_rotor_plant_design == 2 || enable_rotor_plant_design == 3) {
						if (select_suspended_mode == 1){
							sprintf(msg, "\n\rEnter Wn Frequency (rad/sec) of Minimum 0 and Maximum 10 ...................: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &Wo_r);
							if ( Wo_r < 0 ){
								sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
								HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
								HAL_Delay(3000);
								NVIC_SystemReset();
							}
							if (Wo_r > 10){
								Wo_r = 10;
							}
							if (Wo_r < 0.0){
								Wo_r = 0.0;
							}

							rotor_plant_gain = 1;
							sprintf(msg, "%0.2f", Wo_r);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}


						if (select_suspended_mode == 0){
							sprintf(msg, "\n\rEnter Wn Frequency (rad/sec) of Minimum 0 and Maximum 10 ...................: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &Wo_r);
							if ( Wo_r < 0 ){
								sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
								HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
								HAL_Delay(3000);
								NVIC_SystemReset();
							}
							if (Wo_r > 10){
								Wo_r = 10;
							}
							if (Wo_r < 0){
								Wo_r = 0;
							}
							rotor_plant_gain = 1;
							sprintf(msg, "%0.2f", Wo_r);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}
					}
				}



				if (enable_rotor_plant_gain_design == 0 && enable_state_feedback == 0){

					/* Optional addition of transfer function design
					sprintf(msg, "\n\rEnter 1 for Rotor Plant Design Grotor = Wn/(s^3 + Wn*s^2): ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_rotor_plant_design);
					sprintf(msg, "%i", select_rotor_plant_design);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					enable_rotor_plant_design = 3;
					 */

					sprintf(msg, "\n\rEnter 1 for Rotor Plant Design Grotor = 1/(s^2 + Wn*s) .....................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_rotor_plant_design);
					sprintf(msg, "%i", select_rotor_plant_design);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					/* Configure trotor plant design Grotor = 1/(s^2 + Wn*s) with identifier 2 */
					if (select_rotor_plant_design == 1) {
						enable_rotor_plant_design = 2;
					}
					if ( select_rotor_plant_design < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					if (enable_rotor_plant_design == 2 || enable_rotor_plant_design == 3) {
						if (select_suspended_mode == 1){
							sprintf(msg, "\n\rEnter Wn Frequency (rad/sec) of Minimum 0 and Maximum 10 ...................: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &Wo_r);
							if ( Wo_r < 0 ){
								sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
								HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
								HAL_Delay(3000);
								NVIC_SystemReset();
							}
							if (Wo_r > 10){
								Wo_r = 10;
							}
							if (Wo_r < 0){
								Wo_r = 0;
							}
							rotor_plant_gain = 1;
							sprintf(msg, "%0.2f", Wo_r);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}


						if (select_suspended_mode == 0){
							sprintf(msg, "\n\rEnter Wn Frequency (rad/sec) of Minimum 0 and Maximum 10 ...................: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &Wo_r);
							if ( Wo_r < 0 ){
								sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
								HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
								HAL_Delay(3000);
								NVIC_SystemReset();
							}
							if (Wo_r > 10){
								Wo_r = 10;
							}
							if (Wo_r < 0){
								Wo_r = 0;
							}
							rotor_plant_gain = 1;
							sprintf(msg, "%0.2f", Wo_r);
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						}
					}
				}


				torq_current_val = MAX_TORQUE_CONFIG;

				if (ENABLE_TORQUE_CURRENT_ENTRY == 1){
					sprintf(msg, "\n\rEnter Torque Current mA (default is %i)..................................: ", (int)MAX_TORQUE_CONFIG);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &torq_current_val);
					if (torq_current_val == 0){
						torq_current_val = MAX_TORQUE_CONFIG;
					}

					sprintf(msg, "%0.2f", torq_current_val);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				sprintf(msg, "\n\rPendulum PID Gains: \tP: %.02f; I: %.02f; D: %.02f", proportional, integral, derivative);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
				sprintf(msg, "\n\rRotor PID Gains: \tP: %.02f; I: %.02f; D: %.02f", rotor_p_gain, rotor_i_gain, rotor_d_gain);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
				if (select_suspended_mode == 1){
					sprintf(msg, "\n\rSuspended Mode gain values must be negative or zero");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
				}

				max_speed = 		MAX_SPEED_MODE_1;
				min_speed = 		MIN_SPEED_MODE_1;


				break;

				/* Interactive entry of Pendulum Controller gains for Single PID Inverted Mode */

			case 10:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				enable_state_feedback = 0;

				sprintf(msg, "\n\r *** Starting Single PID Configuration Mode ***\n\r ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg),
						HAL_MAX_DELAY);

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart..... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\rEnter Pendulum PID Proportional Gain ...................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg),
						HAL_MAX_DELAY);

				read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &proportional);
				sprintf(msg, "%0.2f", proportional);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( proportional < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				sprintf(msg, "\n\rEnter Pendulum PID Integral Gain .......................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg),
						HAL_MAX_DELAY);
				read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &integral);
				sprintf(msg, "%0.2f", integral);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( integral < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				sprintf(msg, "\n\rEnter Pendulum PID Differential Gain ...................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg),
						HAL_MAX_DELAY);
				read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &derivative);
				sprintf(msg, "%0.2f", derivative);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( derivative < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				/*
				 * Rotor Controller gains for Single PID Mode
				 */
				rotor_p_gain = ROTOR_PID_PROPORTIONAL_GAIN_SINGLE_PID_MODE;
				rotor_i_gain = ROTOR_PID_INTEGRAL_GAIN_SINGLE_PID_MODE;
				rotor_d_gain = ROTOR_PID_DIFFERENTIAL_GAIN_SINGLE_PID_MODE;

				/*
				 * Only inverted mode is supported in Single PID Mode
				 */

				select_suspended_mode = 0;

				enable_rotor_position_step_response_cycle = 0;
				enable_mod_sin_rotor_tracking = 0;
				enable_rotor_chirp = 0;

				enable_rotor_plant_design = 0;
				enable_rotor_plant_gain_design = 0;
				enable_angle_cal = 0;

				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable ................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}


				enable_swing_up = ENABLE_SWING_UP;
				enable_swing_up_resp = 0;
				sprintf(msg, "\n\rSwing Up Enabled - Enter 1 to Disable ..................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_swing_up_resp);
				if (enable_swing_up_resp == 1){
					enable_swing_up = 0;
				}
				sprintf(msg, "%i", enable_swing_up_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_swing_up_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}


				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable ......................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable .............................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable .............................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				sprintf(msg, "\n\rEnter 1 to Enable Pendulum Impulse; 0 to Disable ...........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_pendulum_position_impulse_response_cycle);
				sprintf(msg, "%i", enable_pendulum_position_impulse_response_cycle);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_pendulum_position_impulse_response_cycle < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart*****************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_position_step_response_cycle == 1) {
					enable_pendulum_position_impulse_response_cycle = 0;
				}

				if (enable_mod_sin_rotor_tracking == 1) {
					enable_pendulum_position_impulse_response_cycle = 0;
				}

				if (enable_pendulum_position_impulse_response_cycle == 1) {
				}

				/*
				 * Reverse polarity of gain values to account for suspended mode angle configuration
				 */
				if(select_suspended_mode == 1){
					proportional = 	-proportional;
					integral = 		-integral;
					derivative = 	-derivative;
					rotor_p_gain = 	-rotor_p_gain;
					rotor_i_gain = 	-rotor_i_gain;
					rotor_d_gain = 	-rotor_d_gain;
				}

				max_speed = 		MAX_SPEED_MODE_1;
				min_speed = 		MIN_SPEED_MODE_1;

				sprintf(msg, "\n\rPendulum PID Gains: \tP: %.02f; I: %.02f; D: %.02f", proportional, integral, derivative);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
				sprintf(msg, "\n\rRotor PID Gains: \tP: %.02f; I: %.02f; D: %.02f", rotor_p_gain, rotor_i_gain, rotor_d_gain);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
				if (select_suspended_mode == 1){
					sprintf(msg, "\n\rSuspended Mode gains must be negative");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg),HAL_MAX_DELAY);
				}

				break;

				/* Rotor actuator and encoder test mode */

			case 11:
				enable_rotor_actuator_test = 1;
				enable_encoder_test = 1;
				sprintf(msg, "\n\rTest Mode Configured");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg), HAL_MAX_DELAY);
				break;

				/* Rotor actuator characterization test mode */
			case 13:
				enable_motor_actuator_characterization_mode = 1;
				sprintf(msg, "\n\rMotor Characterization Mode Configured");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);

				rotor_test_speed_min = 200;
				rotor_test_speed_max = 1000;
				rotor_test_acceleration_max = 3000;
				swing_deceleration_max = 3000;
				torq_current_val = MAX_TORQUE_CONFIG;
				rotor_chirp_amplitude = 5;
				rotor_chirp_start_freq = 0.05;
				rotor_chirp_end_freq = 5;
				rotor_chirp_period = 40;

				break;

				/* Pendulum system identification mode */
			case 14:
				enable_pendulum_sysid_test = 1;
				sprintf(msg, "\n\rPendulum System Identification Test Mode Configured");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				break;

				/* Rotor actuator control mode */
			case 15:
				enable_rotor_actuator_control = 1;
				sprintf(msg, "\n\rRotor Actuator Control Mode Configured");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg), HAL_MAX_DELAY);
				break;

				/* Rotor tracking comb signal */
			case 16:
				enable_rotor_tracking_comb_signal = 1;
				rotor_track_comb_amplitude = ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
				sprintf(msg, "\n\rLoad Disturbance Sensitivity Spectrum Analyzer Enabled");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg), HAL_MAX_DELAY);
				break;

				/* Full system identification mode */
			case 19:
				enable_full_sysid = 1;

				select_suspended_mode = 1;
				proportional = 0;
				integral = 0;
				derivative = 0;
				rotor_p_gain = 0;
				rotor_i_gain = 0;
				rotor_d_gain = 0;
				enable_mod_sin_rotor_tracking = 0;
				enable_rotor_position_step_response_cycle = 0;
				enable_rotor_chirp = 0;
				enable_rotor_tracking_comb_signal = 0;

				full_sysid_max_vel_amplitude_deg_per_s = 0;
				full_sysid_min_freq_hz = 0;
				full_sysid_max_freq_hz = 0;
				full_sysid_num_freqs = 0;

				sprintf(msg, "\n\r *** Starting Suspended Mode System Identification (using frequency \"comb\") ***\n\r ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\rEnter maximum velocity amplitude in deg/s (default 150 Hz maximum 150 deg/s): ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);
				read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &full_sysid_max_vel_amplitude_deg_per_s);
				if (full_sysid_max_vel_amplitude_deg_per_s == 0) {
					full_sysid_max_vel_amplitude_deg_per_s = 150;
				}
				if (full_sysid_max_vel_amplitude_deg_per_s >= 150) {
					full_sysid_max_vel_amplitude_deg_per_s = 150;
				}
				sprintf(msg, "%.02f", full_sysid_max_vel_amplitude_deg_per_s);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\rEnter minimum frequency for input signal in Hz (default 0.2 Hz maximum 20 Hz): ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &full_sysid_min_freq_hz);
				if (full_sysid_min_freq_hz == 0) {
					full_sysid_min_freq_hz = 0.2;
				}
				if (full_sysid_min_freq_hz > 20) {
					full_sysid_min_freq_hz = 20;
				}
				sprintf(msg, "%.02f", full_sysid_min_freq_hz);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\rEnter maximum frequency for input signal in Hz (default 5 Hz maximum 20 Hz): ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &full_sysid_max_freq_hz);
				if (full_sysid_max_freq_hz == 0) {
					full_sysid_max_freq_hz = 5;
				}
				if (full_sysid_max_freq_hz > 20) {
					full_sysid_max_freq_hz = 20;
				}
				sprintf(msg, "%.02f", full_sysid_max_freq_hz);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\rEnter the number of frequency steps (default 11 maximum 20): ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &full_sysid_num_freqs);
				if (full_sysid_num_freqs == 0) {
					full_sysid_num_freqs = 11;
				}
				if (full_sysid_num_freqs > 20) {
					full_sysid_num_freqs = 20;
				}
				sprintf(msg, "%i", full_sysid_num_freqs);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				full_sysid_freq_log_step = powf(full_sysid_max_freq_hz/full_sysid_min_freq_hz, 1.0f/(full_sysid_num_freqs-1));

				sprintf(msg, "\n\rGenerating a comb with %d frequencies, using log step multiplier %f.\n\r", full_sysid_num_freqs, full_sysid_freq_log_step);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				sprintf(msg, "List of frequencies (Hz):\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				float f = full_sysid_min_freq_hz;
				for (int i = 0; i < full_sysid_num_freqs; i++) {
					sprintf(msg, "%.02f\t", f);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					f *= full_sysid_freq_log_step;
				}
				sprintf(msg, "\n\rList of frequencies (rad/s):\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				f = full_sysid_min_freq_hz * M_TWOPI;
				for (int i = 0; i < full_sysid_num_freqs; i++) {
					sprintf(msg, "%.02f\t", f);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					f *= full_sysid_freq_log_step;
				}

				sprintf(msg, "\n\rThe input signal will start when \"Log Data\" is activated from the Real-Time Workbench. "
						"Alternatively, enter '>' and hit return once system starts.\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);
				sprintf(msg, "\n\rPress enter to proceed");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);
				read_char(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, NULL);
				sprintf(msg, "\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,strlen(msg), HAL_MAX_DELAY);

				break;

				/* Default start mode */
			default:

				enable_state_feedback = 0;
				select_suspended_mode = 0;
				proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
				integral = 			PRIMARY_INTEGRAL_MODE_1;
				derivative = 		PRIMARY_DERIVATIVE_MODE_1;
				rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
				rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
				rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
				max_speed = 		MAX_SPEED_MODE_1;
				min_speed = 		MIN_SPEED_MODE_1;
				enable_rotor_position_step_response_cycle = 0;
				enable_mod_sin_rotor_tracking = 0;
				enable_angle_cal = 1;
				L6474_SetAnalogValue(0, L6474_TVAL, TORQ_CURRENT_DEFAULT);
				sprintf(msg, "\n\rDefault Mode 1 Configured");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg), HAL_MAX_DELAY);
				break;
			}
			return;
		}
	}
	return;
}

/*
 * Rotor and encoder test mode
 */

void rotor_encoder_test(void){
	/*
	 * Set Motor Speed Profile
	 */

	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_MODE_1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_MODE_1);

	sprintf(msg, "\n\rMotor Profile Speeds Min %u Max %u",
			rotor_test_speed_min, rotor_test_speed_max);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

	BSP_MotorControl_SetAcceleration(0,(uint16_t)(MAX_ACCEL));
	BSP_MotorControl_SetDeceleration(0,(uint16_t)(MAX_DECEL));

	sprintf(msg, "\n\rMotor Profile Acceleration Max %u Deceleration Max %u",
			BSP_MotorControl_GetAcceleration(0), BSP_MotorControl_GetDeceleration(0));
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

	j = 0;

	while (j < ROTOR_ACTUATOR_TEST_CYCLES) {

		j++;

		sprintf(msg, "\r\n\r\n********  Starting Rotor Motor Control Test  ********\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg, "Motor Position at Zero Angle: %.2f\r\n",
				(float) ((rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

		sprintf(msg, "Next Test in 3s\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
		HAL_Delay(3000);

		rotor_position_command_deg = -45;
		BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
		BSP_MotorControl_WaitWhileActive(0);

		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg, "Motor Position Test to -45 Degree Angle: %.2f\r\n",
				(float) ((rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

		sprintf(msg, "Correct motion shows rotor rotating to left\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "Next Test in 3s\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		HAL_Delay(3000);

		rotor_position_command_deg = 0;
		BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
		BSP_MotorControl_WaitWhileActive(0);

		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg, "Motor Position Test to Zero Angle: %.2f\r\n",
				(float) ((rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

		sprintf(msg, "Correct motion shows rotor returning to zero angle\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

		sprintf(msg, "Next Test in 3s\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
		HAL_Delay(3000);

		rotor_position_command_deg = 90;
		BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
		BSP_MotorControl_WaitWhileActive(0);

		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg, "Motor Position at 90 Degree Angle: %.2f\r\n",
				(float) ((rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg, "Correct motion shows rotor rotating to right\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

		sprintf(msg, "Next Test in 3s\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
		HAL_Delay(3000);

		rotor_position_command_deg = 0;
		BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
		BSP_MotorControl_WaitWhileActive(0);

		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg, "Motor Position at Zero Angle: %.2f\r\n",
				(float) ((rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

		sprintf(msg, "Correct motion shows rotor rotating to zero angle\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

		sprintf(msg, "Rotor Actuator Test Cycle Complete, Next Test in 3s\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
		HAL_Delay(3000);
	}
	/*
	 * 	Encoder Test Sequence will execute at each cycle of operation if enable_encoder_test is set to 1
	 */

	if (enable_encoder_test == 1) {
		sprintf(msg, "\r\n*************  Starting Encoder Test  ***************\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		sprintf(msg, "Permit Pendulum to Stabilize in Vertical Down\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
		HAL_Delay(1000);
		sprintf(msg, "Angle will be measured in 3 seconds\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
		HAL_Delay(3000);

		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_down = encoder_position;
		sprintf(msg, "Encoder Angle is: %.2f \r\n(Correct value should lie between -0.5 and 0.5 degrees))\r\n\r\n",
				(float) (encoder_position_down / angle_scale));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		sprintf(msg,
				"Manually Rotate Pendulum in Clock Wise Direction One Full 360 Degree Turn and Stabilize Down\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
		sprintf(msg, "Angle will be measured in 10 seconds\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
		HAL_Delay(10000);

		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		sprintf(msg, "Encoder Angle is: %.2f\r\n(Correct value should lie between -359.5 and -360.5 degrees)\r\n\r\n",
				(float) ((encoder_position_steps - encoder_position_down)
						/ angle_scale));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		sprintf(msg,
				"Manually Rotate Pendulum in Counter Clock Wise Direction One Full 360 Degree Turn and Stabilize Down\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
		sprintf(msg, "Angle will be measured in 10 seconds\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
		HAL_Delay(10000);

		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		sprintf(msg, "Encoder Angle is: %.2f \r\n(Correct value should lie between -0.5 and 0.5 degrees) \r\n\r\n",
				(float) ((encoder_position_steps - encoder_position_down)
						/ angle_scale));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		L6474_CmdDisable(0);
		while(1){
			sprintf(msg, "Test Operation Complete, System in Standby, Press Reset Button to Restart\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
			HAL_Delay(5000);
		}
	}
}

/*
 * Rotor actuator characterization mode
 */

void motor_actuator_characterization_mode(void){
	/*
	 * Set Motor Speed Profile
	 */

	BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
	BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);

	sprintf(msg, "\n\rMotor Profile Speeds Min %u Max %u",
			rotor_test_speed_min, rotor_test_speed_max);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);

	BSP_MotorControl_SetAcceleration(0,
			(uint16_t) (rotor_test_acceleration_max));
	BSP_MotorControl_SetDeceleration(0,
			(uint16_t) (swing_deceleration_max));

	sprintf(msg,
			"\n\rMotor Profile Acceleration Max %u Deceleration Max %u",
			BSP_MotorControl_GetAcceleration(0),
			BSP_MotorControl_GetDeceleration(0));
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);

	/*
	 * Set Rotor Position Zero
	 */

	rotor_position_set();
	test_time = HAL_GetTick() - tick_cycle_start;

	rotor_chirp_step_period = (int) (rotor_chirp_period * 240.0);
	tick_cycle_start = HAL_GetTick();
	mode_index_command = 1;
	mode_index = 1;

	while (1) {
		i = 0;
		while (i < rotor_chirp_step_period) {
			RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE
					- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx,
					RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

			if (readBytes == 2 && Msg.Len == 1 && i % 10 == 0) {
				RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes)
								% UART_RX_BUFFER_SIZE;
				mode_transition_state = 1;
				if (strcmp((char *) Msg.Data, mode_string_stop) == 0) {
					mode_index_command = mode_quit;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_accel)
						== 0) {
					mode_index_command = 17;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_accel)
						== 0) {
					mode_index_command = 16;
				} else if (strcmp((char *) Msg.Data,mode_string_inc_amp)
						== 0) {
					mode_index_command = 18;
				} else if (strcmp((char *) Msg.Data,mode_string_dec_amp)
						== 0) {
					mode_index_command = 19;
				} else if (strcmp((char *) Msg.Data,
						mode_string_mode_motor_characterization_mode)
						== 0) {
					mode_index_command = 1;
				} else {
					mode_index_command = atoi((char*) Msg.Data);
				}
			}



			if (mode_index_command == mode_quit) {
				break;
			}

			if (mode_index_command == 1 && mode_transition_state == 1) {
				mode_index = 1;
				mode_transition_state = 0;
			}

			if (mode_index_command == 2 && mode_transition_state == 1) {
				mode_index = 2;
				mode_transition_state = 0;
			}

			if (mode_index_command == 3 && mode_transition_state == 1) {
				L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
				mode_transition_state = 0;
			}

			if (mode_index_command == 4 && mode_transition_state == 1) {
				L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
				mode_transition_state = 0;
			}
			if (mode_index_command == 5 && mode_transition_state == 1) {
				L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
				mode_transition_state = 0;
			}

			if (mode_index_command == 6 && mode_transition_state == 1) {
				rotor_test_speed_max = rotor_test_speed_max + 100;
				if (rotor_test_speed_max > 1000) {
					rotor_test_speed_max = 1000;
				}
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
				mode_transition_state = 0;
			}

			if (mode_index_command == 7 && mode_transition_state == 1) {
				rotor_test_speed_max = rotor_test_speed_max - 100;
				if (rotor_test_speed_max < 200) {
					rotor_test_speed_max = 200;
				}
				if (rotor_test_speed_min > rotor_test_speed_max) {
					rotor_test_speed_max = rotor_test_speed_min;
				}
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
				mode_transition_state = 0;
			}

			if (mode_index_command == 8 && mode_transition_state == 1) {
				rotor_test_speed_min = rotor_test_speed_min + 100;
				if (rotor_test_speed_min > rotor_test_speed_max) {
					rotor_test_speed_min = rotor_test_speed_max;
				}
				if (rotor_test_speed_min > 1000) {
					rotor_test_speed_min = 1000;
				}
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);
				mode_transition_state = 0;
			}

			if (mode_index_command == 9 && mode_transition_state == 1) {
				rotor_test_speed_min = rotor_test_speed_min - 100;
				if (rotor_test_speed_min < 200) {
					rotor_test_speed_min = 200;
				}
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);
				mode_transition_state = 0;
			}

			if (mode_index_command == 16 && mode_transition_state == 1) {
				rotor_test_acceleration_max = rotor_test_acceleration_max - 500;
				if (rotor_test_acceleration_max < 0) {
					rotor_test_acceleration_max = 0;
				}
				swing_deceleration_max = rotor_test_acceleration_max;
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetAcceleration(0,
						(uint16_t) (rotor_test_acceleration_max));
				BSP_MotorControl_SetDeceleration(0,
						(uint16_t) (swing_deceleration_max));
				mode_transition_state = 0;
			}

			if (mode_index_command == 17 && mode_transition_state == 1) {
				rotor_test_acceleration_max = rotor_test_acceleration_max + 500;
				if (rotor_test_acceleration_max > 10000) {
					rotor_test_acceleration_max = 10000;
				}
				swing_deceleration_max = rotor_test_acceleration_max;
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetAcceleration(0,
						(uint16_t) (rotor_test_acceleration_max));
				BSP_MotorControl_SetDeceleration(0,
						(uint16_t) (swing_deceleration_max));
				mode_transition_state = 0;
			}

			if (mode_index_command == 18 && mode_transition_state == 1) {
				rotor_chirp_amplitude = rotor_chirp_amplitude + 1;
				if (rotor_chirp_amplitude > 10) {
					rotor_chirp_amplitude = 10;
				}
				mode_transition_state = 0;
			}

			if (mode_index_command == 19 && mode_transition_state == 1) {
				rotor_chirp_amplitude = rotor_chirp_amplitude - 1;
				if (rotor_chirp_amplitude < 1) {
					rotor_chirp_amplitude = 1;
				}
				mode_transition_state = 0;
			}

			if (i == 0) {
				cycle_period_start = HAL_GetTick();
				cycle_period_sum = 100 * Tsample * 1000 - 1;
			}
			if (i % 100 == 0) {
				cycle_period_sum = HAL_GetTick() - cycle_period_start;
				cycle_period_start = HAL_GetTick();
			}

			tick_cycle_previous = tick_cycle_current;
			tick_cycle_current = tick;
			chirp_time = (float) (i) / 400;
			rotor_chirp_frequency = rotor_chirp_start_freq
					+ (rotor_chirp_end_freq - rotor_chirp_start_freq)
					* (float) (i) / rotor_chirp_step_period;

			if (mode_index == 1) {
				rotor_position_command_steps =
						rotor_chirp_amplitude
						* (float) (STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)
						* sin(
								2.0 * 3.14159
								* rotor_chirp_frequency
								* chirp_time);
			}

			if (mode_index == 2) {
				if (sin(
						2.0 * 3.14159 * rotor_chirp_frequency
						* chirp_time) < 0) {
					k = -1;
				} else {
					k = 1;
				}
				rotor_position_command_steps = k * rotor_chirp_amplitude
						* STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
			}

			current_speed = BSP_MotorControl_GetCurrentSpeed(0);
			BSP_MotorControl_GoTo(0, (int) (rotor_position_command_steps));

			if (BSP_MotorControl_GetDeviceState(0) == ACCELERATING) {
				motor_state = 1;
			}
			if (BSP_MotorControl_GetDeviceState(0) == DECELERATING) {
				motor_state = -1;
			}
			if (BSP_MotorControl_GetDeviceState(0) == STEADY) {
				motor_state = -2;
			}
			if (BSP_MotorControl_GetDeviceState(0) == INACTIVE) {
				motor_state = 0;
			}
			ret = rotor_position_read(&rotor_position_steps);
			current_speed = BSP_MotorControl_GetCurrentSpeed(0);
			sprintf(msg,
					"%i\t%i\t%i\t%i\t%i\t%f\t%i\t%i\t%i\t%i\t%i\r\n", i,
					cycle_period_sum,
					(int) (tick_cycle_current - tick_cycle_previous),
					current_speed, rotor_position_steps,
					rotor_position_command_steps, motor_state,
					rotor_test_speed_max, rotor_test_speed_min,
					rotor_test_acceleration_max, swing_deceleration_max);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
			i = i + 1;
		}
		if (mode_index_command == mode_quit) {
			break;
		}
		j = j + 1;
	}
	L6474_CmdDisable(0);
	while (1){
		HAL_Delay(5000);
		sprintf(msg, "\r\nMotor Characterization Terminated, Press Reset to Continue");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
	}
}

/*
 * Interactive rotor control
 */

void interactive_rotor_actuator_control(void){
	while (1) {

		/*
		 * Set Motor Speed Profile
		 */

		sprintf(msg, "\r\nEnter Motor Maximum Speed..............................................: ");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
				&rotor_test_speed_max);
		sprintf(msg, "%i", rotor_test_speed_max);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		sprintf(msg, "\r\nEnter Motor Minimum Speed..............................................: ");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
				&rotor_test_speed_min);
		sprintf(msg, "%i", rotor_test_speed_min);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		sprintf(msg, "\r\nEnter Motor Maximum Acceleration.......................................: ");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
				&rotor_test_acceleration_max);
		sprintf(msg, "%i", rotor_test_acceleration_max);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		sprintf(msg, "\r\nEnter Motor Maximum Deceleration.......................................: ");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
				&swing_deceleration_max);
		sprintf(msg, "%i", swing_deceleration_max);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
		BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);

		sprintf(msg, "\n\rMotor Profile Speeds Minimum %u Maximum %u",
				rotor_test_speed_min, rotor_test_speed_max);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

		BSP_MotorControl_SetAcceleration(0, rotor_test_acceleration_max);
		BSP_MotorControl_SetDeceleration(0, swing_deceleration_max);

		sprintf(msg,"\n\rMotor Profile Acceleration Maximum %u Deceleration Maximum %u",
				BSP_MotorControl_GetAcceleration(0), BSP_MotorControl_GetDeceleration(0));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

		j = 1;

		/*
		 * Set initial rotor position
		 */

		while (1) {

			sprintf(msg, "\r\nEnter Motor Position Target in Degrees ................................: ");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &rotor_position_command_deg);
			sprintf(msg, "%0.2f", rotor_position_command_deg);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
			BSP_MotorControl_WaitWhileActive(0);

			ret = rotor_position_read(&rotor_position_steps);
			sprintf(msg, "\n\rMotor Position in Steps %i and Degrees %.2f\r\n",
					rotor_position_steps, (float) ((rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			sprintf(msg, "\r\nEnter 1 to Enter New Motor Configuration, 0 to Continue ...............: ");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
					&j);
			if (j == 1) {
				break;
			}

		}
	}
	L6474_CmdDisable(0);
	while (1){
		HAL_Delay(5000);
		sprintf(msg, "\r\nMotor Characterization Terminated, Press Reset to Continue");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
	}
}
