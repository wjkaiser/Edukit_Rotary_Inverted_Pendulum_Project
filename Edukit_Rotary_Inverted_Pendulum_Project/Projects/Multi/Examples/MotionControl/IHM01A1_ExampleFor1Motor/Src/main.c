
/*
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 *
 *    Acknowledgments to the invaluable development, support and guidance by
 *    Marco De Fazio, Giorgio Mariano, Enrico Poli, and Davide Ghezzi
 *    of STMicroelectronics
 *
 *    Acknowledgements to the innovative development, support and guidance by
 *    Markus Dauberschmidt for development of the Pendulum Swing Up algorithm
 *    Please see https://github.com/OevreFlataeker/steval_edukit_swingup
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
 *
 * @author  William J. Kaiser (UCLA Electrical and Computer Engineering).
 *
 * Application based on development by STMicroelectronics as described below
 *
 * @version V2.0
 * @date    January 13, 2021
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

/*
 * Integrated Rotary Inverted Pendulum System Configuration
 *
 * ********** Primary Components *************************
 * Processor:			Nucleo F401RE
 * Motor Interface: 	IHM01A1 Stepper Motor Controller
 * Motor Power Supply:	12V 2A Supply
 * Encoder:				LPD3806-600BM-G5-24C 600 Pulse Per Revolution Incremental Rotary Encoder
 * Stepper Motor:		NEMA-17 17HS19-2004S  Stepper Motor
 *
 *********** Stepper Lead Assignment *********************
 *
 * Lead Color	IHM01A1 Terminal
 * 	 Blue			-A
 * 	 Red			+A
 * 	 Green			-B
 * 	 Black			+B
 *
 * Stepper Lead Extension Cable (if present) replaces Blue with White
 *
 * Caution: Please note that motors have been receieved from the vendor showing
 * reversal of Motor White and Motor Red.  Check rotor operation after assembly
 * and in initial testing.
 *
 * ********** Encoder Lead Assignment *********************
 *
 * Lead Color	Nucleo F401RE Terminal
 * 	Red 			5V
 * 	Black 			GND
 * 	White 			GPIO Dir3
 * 	Green 			GPIO Dir2
 *
 * Note: Optical Encoder is LPD3806-600BM-G5-24C This encoder requires an open collector pull up resistor.
 * Note: GPIO_PULLUP is set in HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder) of
 * stm32f4xx_hal_msp.c Line 217
 *
 * Initial Motor Speed Profiles, Torque Current, and Overcurrent Thresholds set in l6474_target_config.h
 *
 *
 *
 */




/*
 * System Configuration Parameters
 *
 * ENABLE_SUSPENDED_PENDULUM_CONTROL is set to 0 for Inverted Pendulum and
 * 									 set to 1 for Suspended Pendulum
 * ENABLE_DUAL_PID is set to 1 to enable control action
 *
 * High Speed 2 millisecond Control Loop delay, 500 Hz Cycle System Configuration
 *
 * Motor Speed Profile Configurations are:
 *
 *						MAX_ACCEL: 3000; 	MAX_DECEL 3000
 * High Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 500
 * Medium Speed Mode:	MAX_SPEED: 1000; 	MIN_SPEED 300
 * Low Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 200
 * Suspended Mode: 		MAX_SPEED: 1000; 	MIN_SPEED 200
 *
 */



/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "edukit_system.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>


/*
 * Motor Interface Data Structure
 *
 * Note that this is not required by default since Motor Profile is included
 * from l6474_target_config.h
 *
 * Note: This application is based on usage of l6474_target_config.h header
 * file for initialization of configuration of L6474
 */

L6474_Init_t gL6474InitParams = {
		MAX_ACCEL,           	/// Acceleration rate in step/s2. Range: (0..+inf).
		MAX_DECEL,           	/// Deceleration rate in step/s2. Range: (0..+inf).
		MAX_SPEED,              /// Maximum speed in step/s. Range: (30..10000].
		MIN_SPEED,              /// Minimum speed in step/s. Range: [30..10000).
		MAX_TORQUE_CONFIG, 		/// Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
		OVERCURRENT_THRESHOLD, 	/// Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
		L6474_CONFIG_OC_SD_ENABLE, /// Overcurrent shutwdown (OC_SD field of CONFIG register).
		L6474_CONFIG_EN_TQREG_TVAL_USED, /// Torque regulation method (EN_TQREG field of CONFIG register).
		L6474_STEP_SEL_1_16, 	/// Step selection (STEP_SEL field of STEP_MODE register).
		L6474_SYNC_SEL_1_2, 	/// Sync selection (SYNC_SEL field of STEP_MODE register).
		L6474_FAST_STEP_12us, 	/// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
		L6474_TOFF_FAST_8us, 	/// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
		3,   					/// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
		21, 					/// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
		L6474_CONFIG_TOFF_044us, /// Target Swicthing Period (field TOFF of CONFIG register).
		L6474_CONFIG_SR_320V_us, /// Slew rate (POW_SR field of CONFIG register).
		L6474_CONFIG_INT_16MHZ, /// Clock setting (OSC_CLK_SEL field of CONFIG register).
		(L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN
				| L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE
				| L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD) /// Alarm (ALARM_EN register).
};

 /* CMSIS */
#define ARM_MATH_CM4

/*
 * Apply Swing Up algorithm developed by Markus Dauberschmidt
 */

#define swing_up 1

/*
 * Apply acceleration
 */

#define ACCEL_CONTROL_DATA 0		// Set to 1 for display of timing data
#define ACCEL_CONTROL 1 			// Set to 1 to enable acceleration control. Set to 0 to use position target control.
#define PWM_COUNT_SAFETY_MARGIN 2
#define MAXIMUM_ACCELERATION 131071
#define MAXIMUM_DECELERATION 131071
#define MAXIMUM_SPEED 131071

volatile uint16_t gLastError;
/* Private function prototypes -----------------------------------------------*/
void read_float(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, float *float_return);
void Error_Handler(uint16_t error);
void read_int(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, int * int_return);
void read_char(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, char * char_return);
void select_mode_1(void);
void user_configuration(void);
void Main_StepClockHandler();
void apply_acceleration(float * acc, float* target_velocity_prescaled, float t_sample);

#define delayUS_ASM(us) do {\
		asm volatile (	"MOV R0,%[loops]\n\t"\
				"1: \n\t"\
				"SUB R0, #1\n\t"\
				"CMP R0, #0\n\t"\
				"BNE 1b \n\t" : : [loops] "r" (16*us) : "memory"\
		);\
} while(0)

int Delay_Pulse(){
	return desired_pwm_period == UINT32_MAX;
}

/*
 * PWM pulse (step) interrupt
 */
void Main_StepClockHandler() {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */

	uint32_t desired_pwm_period_local = desired_pwm_period;

	/*
	 * Add time reporting
	 */

	clock_int_time = DWT->CYCCNT;

	if (desired_pwm_period_local != 0) {
		L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
		current_pwm_period = desired_pwm_period_local;
	}
}

void apply_acceleration(float * acc, float * target_velocity_prescaled, float t_sample) {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */

	uint32_t current_pwm_period_local = current_pwm_period;
	uint32_t desired_pwm_period_local = desired_pwm_period;

	/*
	 * Add time reporting
	 */

	apply_acc_start_time = DWT->CYCCNT;


	motorDir_t old_dir = *target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (old_dir == FORWARD) {
		if (*acc > MAXIMUM_ACCELERATION) {
			*acc = MAXIMUM_ACCELERATION;
		} else if (*acc < -MAXIMUM_DECELERATION) {
			*acc = -MAXIMUM_DECELERATION;
		}
	} else {
		if (*acc < -MAXIMUM_ACCELERATION) {
			*acc = -MAXIMUM_ACCELERATION;
		} else if (*acc > MAXIMUM_DECELERATION) {
			*acc = MAXIMUM_DECELERATION;
		}	
	}

	*target_velocity_prescaled += L6474_Board_Pwm1PrescaleFreq(*acc) * t_sample;
	motorDir_t new_dir = *target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (*target_velocity_prescaled > L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
		*target_velocity_prescaled = L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
	} else if (*target_velocity_prescaled < -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
		*target_velocity_prescaled = -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
	}

	float speed_prescaled;
	if (new_dir == FORWARD) {
		speed_prescaled = *target_velocity_prescaled;
	} else {
		speed_prescaled = *target_velocity_prescaled * -1;
		if (speed_prescaled == 0) speed_prescaled = 0; // convert negative 0 to positive 0
	}


	uint32_t effective_pwm_period = desired_pwm_period_local;

	float desired_pwm_period_float = roundf(RCC_SYS_CLOCK_FREQ / speed_prescaled);
	if (!(desired_pwm_period_float < 4294967296.0f)) {
		desired_pwm_period_local = UINT32_MAX;
	} else {
		desired_pwm_period_local = (uint32_t)(desired_pwm_period_float);
	}

	if (old_dir != new_dir) {
		L6474_Board_SetDirectionGpio(0, new_dir);
	}

	if (current_pwm_period_local != 0) {
		uint32_t pwm_count = L6474_Board_Pwm1GetCounter();
		uint32_t pwm_time_left = current_pwm_period_local - pwm_count;
		if (pwm_time_left > PWM_COUNT_SAFETY_MARGIN) {
			if (old_dir != new_dir) {
				// pwm_time_left = effective_pwm_period - pwm_time_left; // One method for assignment of PWM period during switching directions. This has the effect of additional discrete step noise.
				pwm_time_left = effective_pwm_period; // Second method for assignment of PWM period during switching directions. This shows reduced discrete step noise.
			}

			uint32_t new_pwm_time_left = ((uint64_t) pwm_time_left * desired_pwm_period_local) / effective_pwm_period;
			if (new_pwm_time_left != pwm_time_left) {
				if (new_pwm_time_left < PWM_COUNT_SAFETY_MARGIN) {
					new_pwm_time_left = PWM_COUNT_SAFETY_MARGIN;
				}
				current_pwm_period_local = pwm_count + new_pwm_time_left;
				if (current_pwm_period_local < pwm_count) {
					current_pwm_period_local = UINT32_MAX;
				}

				L6474_Board_Pwm1SetPeriod(current_pwm_period_local);
				current_pwm_period = current_pwm_period_local;
			}
		}
	} else {
		L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
		current_pwm_period = desired_pwm_period_local;
	}

	desired_pwm_period = desired_pwm_period_local;

}

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
	uint32_t clk_cycle_start = DWT->CYCCNT;

	/* Go to number of cycles for system */
	microseconds *= (RCC_HCLK_FREQ / 1000000);

	/* Delay till end */
	while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

__STATIC_INLINE void DWT_Delay_until_cycle(volatile uint32_t cycle)
{
	while (DWT->CYCCNT < cycle);
}

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
T_Serial_Msg Msg;

uint8_t RxBuffer[UART_RX_BUFFER_SIZE];
uint16_t Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos,
		uint16_t BufMaxLen, T_Serial_Msg *Msg);

/* Acceleration control system variables */
volatile uint32_t apply_acc_start_time;
volatile uint32_t clock_int_time;
volatile uint32_t clock_int_tick;

/// PWM period variables used by step interrupt
volatile uint32_t desired_pwm_period;
volatile uint32_t current_pwm_period;

float target_velocity_prescaled;
int32_t enable_speed_prescale;

/* System data reporting */
char tmp_string[256];
char msg[192];
char msg_pad[64];
char test_msg[128];

/*
 * Timer 3, UART Transmit, and UART DMA Receive declarations
 */

TIM_HandleTypeDef htim3;

/*
  * Timer 3, UART Transmit, and UART DMA Receive declarations
  */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Control system output signal */
float rotor_control_target_steps;
float rotor_control_target_steps_curr;
float rotor_control_target_steps_prev;

/* Control system variables */
int rotor_position_delta;
int initial_rotor_position;
int cycle_count;
int i, j, k, m;
int ret;

/* PID control system variables */
float windup, rotor_windup;
float *current_error_steps, *current_error_rotor_steps;
float *sample_period, *sample_period_rotor;

/* Loop timing measurement variables */
int cycle_period_start;
int cycle_period_sum;
int enable_cycle_delay_warning;

/* PID control variables */
float *deriv_lp_corner_f;
float *deriv_lp_corner_f_rotor;
float proportional, rotor_p_gain;
float integral, rotor_i_gain;
float derivative, rotor_d_gain;

/* State Feedback variables */
int enable_state_feedback;
float integral_compensator_gain;
float feedforward_gain;
float current_error_rotor_integral;

/* Reference tracking command */
float reference_tracking_command;

/* Pendulum position and tracking command */

/* Rotor position and tracking command */
int rotor_position_steps;
float rotor_position_command_steps;
float rotor_position_command_steps_pf, rotor_position_command_steps_pf_prev;
float rotor_position_command_deg;
float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;
float rotor_position_diff, rotor_position_diff_prev;
float rotor_position_diff_filter, rotor_position_diff_filter_prev;
int rotor_target_in_steps;
int initial_rotor_position;

/* Rotor Plant Design variables */
int select_rotor_plant_design, enable_rotor_plant_design, enable_rotor_plant_gain_design;
int rotor_control_target_steps_int;
float rotor_damping_coefficient, rotor_natural_frequency;
float rotor_plant_gain;
float rotor_control_target_steps_gain;
float rotor_control_target_steps_filter_2, rotor_control_target_steps_filter_prev_2;
float rotor_control_target_steps_prev_prev, rotor_control_target_steps_filter_prev_prev_2;
float c0, c1, c2, c3, c4, ao, Wn2;
float fo_r, Wo_r, IWon_r, iir_0_r, iir_1_r, iir_2_r;

/* Encoder position variables */
uint32_t cnt3;
int range_error;
float encoder_position;
int encoder_position_steps;
int encoder_position_init;
int previous_encoder_position;
int max_encoder_position;
int global_max_encoder_position;
int prev_global_max_encoder_position;
int encoder_position_down;
int encoder_position_curr;
int encoder_position_prev;

/* Angle calibration variables */
float encoder_position_offset;
float encoder_position_offset_zero;
int enable_angle_cal;
int enable_angle_cal_resp;
int offset_end_state;
int offset_start_index;
int angle_index;
int angle_avg_index;
int angle_avg_span;
int offset_angle[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
float encoder_position_offset_avg[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
int angle_cal_end;
int angle_cal_complete;

/* Swing Up system variables */
int enable_swing_up;
int enable_swing_up_resp;
bool peaked;
bool handled_peak;
int zero_crossed;
motorDir_t swing_up_direction;
int swing_up_state, swing_up_state_prev;
int stage_count;
int stage_amp;

/* Initial control state parameter storage */
float init_r_p_gain, init_r_i_gain, init_r_d_gain;
float init_p_p_gain, init_p_i_gain, init_p_d_gain;
int init_enable_state_feedback;
float init_integral_compensator_gain;
float init_feedforward_gain;
int init_enable_state_feedback;
int init_enable_disturbance_rejection_step;
int init_enable_sensitivity_fnc_step;
int init_enable_noise_rejection_step;
int init_enable_rotor_plant_design;
int init_enable_rotor_plant_gain_design;

/* Low pass filter variables */
float fo, Wo, IWon, iir_0, iir_1, iir_2;
float fo_LT, Wo_LT, IWon_LT;
float iir_LT_0, iir_LT_1, iir_LT_2;
float fo_s, Wo_s, IWon_s, iir_0_s, iir_1_s, iir_2_s;

/* Slope correction system variables */
int slope;
int slope_prev;
float encoder_angle_slope_corr_steps;

/* Adaptive control variables */
float adaptive_error, adaptive_threshold_low, adaptive_threshold_high;
float error_sum_prev, error_sum, error_sum_filter_prev, error_sum_filter;
int adaptive_entry_tick, adaptive_dwell_period;
int enable_adaptive_mode, adaptive_state, adaptive_state_change;
float rotor_position_command_steps_prev;

/* Rotor impulse variables */
int rotor_position_step_polarity;
int impulse_start_index;

/* User configuration variables */
int clear_input;
uint32_t enable_control_action;
int max_speed_read, min_speed_read;
int select_suspended_mode;
int motor_response_model;
int enable_rotor_actuator_test, enable_rotor_actuator_control;
int enable_encoder_test;
int enable_rotor_actuator_high_speed_test;
int enable_motor_actuator_characterization_mode;
int motor_state;
float torq_current_val;


/* Rotor chirp system variables */
int enable_rotor_chirp;
int chirp_cycle;
int chirp_dwell_cycle;
float chirp_time;
float rotor_chirp_start_freq;
float rotor_chirp_end_freq;
float rotor_chirp_period ;
float rotor_chirp_frequency;
float rotor_chirp_amplitude;
int rotor_chirp_step_period;

float pendulum_position_command_steps;

/* Modulates sine tracking signal system variables */
int enable_mod_sin_rotor_tracking;
int enable_rotor_position_step_response_cycle;
int disable_mod_sin_rotor_tracking;
int sine_drive_transition;
float mod_sin_amplitude;
float rotor_control_sin_amplitude;
float rotor_sine_drive, rotor_sine_drive_mod;
float rotor_mod_control;
float mod_sin_carrier_frequency;

/* Pendulum impulse system variables */
int enable_pendulum_position_impulse_response_cycle;

/* Rotor high speed test system variables */
int swing_cycles, rotor_test_speed_min, rotor_test_speed_max;
int rotor_test_acceleration_max, swing_deceleration_max;
int start_angle_a[20], end_angle_a[20], motion_dwell_a[20];
int abs_encoder_position_prior, abs_encoder_position_after, abs_encoder_position_max;
uint16_t current_speed;

/*Pendulum system ID variable */
int enable_pendulum_sysid_test;

/* Full system identification variables */
int enable_full_sysid;
float full_sysid_max_vel_amplitude_deg_per_s;
float full_sysid_min_freq_hz;
float full_sysid_max_freq_hz;
int full_sysid_num_freqs;
float full_sysid_freq_log_step;
int full_sysid_start_index;

/* Rotor comb drive system variables */
int enable_rotor_tracking_comb_signal;
float rotor_track_comb_signal_frequency;
float rotor_track_comb_command;
float rotor_track_comb_amplitude;

/* Sensitivity function system variables */
int enable_disturbance_rejection_step;
int enable_noise_rejection_step;
int enable_plant_rejection_step;
int enable_sensitivity_fnc_step;
float load_disturbance_sensitivity_scale;

/* Noise rejection sensitivity function low pass filter */

float noise_rej_signal_filter, noise_rej_signal;
float noise_rej_signal_prev, noise_rej_signal_filter_prev;

/*
 * Real time user input system variables
 */

char config_message[16];
int config_command;
int display_parameter;
int step_size;
float adjust_increment;
int mode_index;

/* Real time data reporting index */
int report_mode;
int speed_scale;
int speed_governor;

/*
 * User selection mode values
 */

int mode_1;				// Enable LQR Motor Model M
int mode_2;				// Enable LRR Motor Model H
int mode_3;				// Enable LQR Motor Model L
int mode_4;				// Enable Suspended Mode Motor Model M
int mode_5;				// Enable sin drive track signal
int mode_adaptive_off;	// Disable adaptive control
int mode_adaptive;		// Enable adaptive control
int mode_8;				// Enable custom configuration entry
int mode_9;				// Disable sin drive track signal
int mode_10;			// Enable Single PID Mode with Motor Model M
int mode_11;			// Enable rotor actuator and encoder test mode
int mode_13;			// Enable rotor control system evaluation
int mode_15;			// Enable interactive control of rotor actuator
int mode_16;			// Enable load disturbance function step mode
int mode_17;			// Enable noise disturbance function step mode
int mode_18;			// Enable sensitivity function step mode
int mode_19;            // Enable full system identification mode
int mode_quit;			// Initiate exit from control loop
int mode_interactive;	// Enable continued terminal interactive user session
int mode_index_prev, mode_index_command;
int mode_transition_tick;
int mode_transition_state;
int transition_to_adaptive_mode;


/*
 * Real time user input characters
 */

char mode_string_stop[UART_RX_BUFFER_SIZE];
char mode_string_mode_1[UART_RX_BUFFER_SIZE];
char mode_string_mode_2[UART_RX_BUFFER_SIZE];
char mode_string_mode_3[UART_RX_BUFFER_SIZE];
char mode_string_mode_4[UART_RX_BUFFER_SIZE];
char mode_string_mode_8[UART_RX_BUFFER_SIZE];
char mode_string_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
char mode_string_mode_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_control[UART_RX_BUFFER_SIZE];
char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_full_sysid[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_p[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_p[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_i[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_i[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_d[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_p[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_p[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_i[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_i[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_d[UART_RX_BUFFER_SIZE];
char mode_string_dec_torq_c[UART_RX_BUFFER_SIZE];
char mode_string_inc_torq_c[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_s[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_s[UART_RX_BUFFER_SIZE];
char mode_string_dec_min_s[UART_RX_BUFFER_SIZE];
char mode_string_inc_min_s[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_a[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_a[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_d[UART_RX_BUFFER_SIZE];
char mode_string_enable_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_step[UART_RX_BUFFER_SIZE];
char mode_string_enable_pendulum_impulse[UART_RX_BUFFER_SIZE];
char mode_string_disable_pendulum_impulse[UART_RX_BUFFER_SIZE];
char mode_string_enable_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_disable_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_enable_noise_rej_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_noise_rej_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
char mode_string_enable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
char mode_string_inc_step_size[UART_RX_BUFFER_SIZE];
char mode_string_dec_step_size[UART_RX_BUFFER_SIZE];
char mode_string_select_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_enable_high_speed_sampling[UART_RX_BUFFER_SIZE];
char mode_string_disable_high_speed_sampling[UART_RX_BUFFER_SIZE];
char mode_string_enable_speed_prescale[UART_RX_BUFFER_SIZE];
char mode_string_disable_speed_prescale[UART_RX_BUFFER_SIZE];
char mode_string_disable_speed_governor[UART_RX_BUFFER_SIZE];
char mode_string_enable_speed_governor[UART_RX_BUFFER_SIZE];
char mode_string_reset_system[UART_RX_BUFFER_SIZE];


int char_mode_select;	// Flag detecting whether character mode select entered


char message_received[UART_RX_BUFFER_SIZE];
char mode_string_mode_1[UART_RX_BUFFER_SIZE];
char mode_string_mode_2[UART_RX_BUFFER_SIZE];
char mode_string_mode_3[UART_RX_BUFFER_SIZE];
char mode_string_mode_4[UART_RX_BUFFER_SIZE];
char mode_string_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_mode_8[UART_RX_BUFFER_SIZE];
char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
char mode_string_mode_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_control[UART_RX_BUFFER_SIZE];
char mode_string_mode_high_speed_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
char mode_string_mode_pendulum_sysid_test[UART_RX_BUFFER_SIZE];
char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_stop[UART_RX_BUFFER_SIZE];

/* CMSIS Variables */
arm_pid_instance_a_f32 PID_Pend, PID_Rotor;
float Deriv_Filt_Pend[2];
float Deriv_Filt_Rotor[2];
float Wo_t, fo_t, IWon_t;

/* System timing variables */

uint32_t tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
tick_read_cycle, tick_read_cycle_start,tick_wait_start,tick_wait;

volatile uint32_t current_cpu_cycle, prev_cpu_cycle, last_cpu_cycle, target_cpu_cycle, prev_target_cpu_cycle;
volatile int current_cpu_cycle_delay_relative_report;

uint32_t t_sample_cpu_cycles;
float Tsample, Tsample_rotor, test_time;
float angle_scale;
int enable_high_speed_sampling;

/* Reset state tracking */
int reset_state;

/* Motor configuration */
uint16_t min_speed, max_speed, max_accel, max_decel;

/* Serial interface variables */
uint32_t RxBuffer_ReadIdx;
uint32_t RxBuffer_WriteIdx;
uint32_t readBytes;


int main(void) {

	/* Initialize reset state indicating that reset has occurred */

	reset_state = 1;

	/* Initialize and enable cycle counter */
	ITM->LAR = 0xC5ACCE55; 	// at address 0xE0001FB0
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // at address 0xE000EDFC, CoreDebug_DEMCR_TRCENA_Msk = 0x01000000
	DWT->CTRL |= 1; 		// at address 0xE0001000
	DWT->CYCCNT = 0; 		// at address 0xE0001004

	/* initialize Integrator Mode time variables */
	apply_acc_start_time = 0;
	clock_int_time = 0;
	clock_int_tick = 0;

	/* Initialize PWM period variables used by step interrupt */
	desired_pwm_period = 0;
	current_pwm_period = 0;
	target_velocity_prescaled = 0;

	/* Initialize default start mode and reporting mode */
	mode_index = 1;
	report_mode = 1;

	/*Initialize serial read variables */
	RxBuffer_ReadIdx = 0;
	RxBuffer_WriteIdx = 0;
	readBytes = 0;

	/*Initialize encoder variables */
	encoder_position = 0;
	encoder_position_down = 0;
	encoder_position_curr = 0;
	encoder_position_prev = 0;
	angle_scale = ENCODER_READ_ANGLE_SCALE;

	/*Initialize rotor control variables */
	rotor_control_target_steps = 0;
	rotor_control_target_steps_curr = 0;
	rotor_control_target_steps_prev = 0;

	/*Initialize rotor plant design transfer function computation variables */
	rotor_control_target_steps_filter_prev_2 = 0.0;
	rotor_control_target_steps_filter_prev_prev_2 = 0.0;
	rotor_control_target_steps_prev_prev = 0.0;

	/* Initialize LQR integral control variables */
	current_error_rotor_integral = 0;

	/*Initialize rotor tracking signal variables */
	enable_rotor_chirp = 0;
	rotor_chirp_start_freq = ROTOR_CHIRP_START_FREQ;
	rotor_chirp_end_freq = ROTOR_CHIRP_END_FREQ;
	rotor_chirp_period = ROTOR_CHIRP_PERIOD;
	enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
	enable_rotor_position_step_response_cycle = ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE;
	disable_mod_sin_rotor_tracking = 0;
	sine_drive_transition = 0;
	mod_sin_amplitude = MOD_SIN_AMPLITUDE;
	rotor_control_sin_amplitude = MOD_SIN_AMPLITUDE;

	/*Initialize sensitivity function selection variables */
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;
	enable_pendulum_position_impulse_response_cycle = 0;

	/*Initialize user adjustment variables */
	step_size = 0;
	adjust_increment = 0.5;

	/*Initialize adaptive mode state variables */
	mode_transition_state = 0;
	transition_to_adaptive_mode = 0;

	/*Initialize user interactive mode */
	char_mode_select = 0;

	/* STM32xx HAL library initialization */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Default select_suspended_mode */
	select_suspended_mode = ENABLE_SUSPENDED_PENDULUM_CONTROL;

	//----- Initialize Motor Control Library
	/* Set the L6474 library to use 1 device */
	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);

	/* When BSP_MotorControl_Init is called with NULL pointer,                  */
	/* the L6474 registers and parameters are set with the predefined values from file   */
	/* l6474_target_config.h, otherwise the registers are set using the   */
	/* L6474_Init_t pointer structure                */
	/* The first call to BSP_MotorControl_Init initializes the first device     */
	/* whose Id is 0.                                                           */
	/* The nth call to BSP_MotorControl_Init initializes the nth device         */
	/* whose Id is n-1.                                                         */
	/* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
	/* device with the structure gL6474InitParams declared in the the main.c file */
	/* and comment the subsequent call having the NULL pointer                   */
	//BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);

	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);

	/* Initialize Timer and UART */
	MX_TIM3_Init();

	/* Initialize Pendulum Angle Encoder offset */
	encoder_position_init = 0;
	HAL_Delay(10);
	/* Initialize UART communication port */
	MX_USART2_UART_Init();

	/* Motor Range Initialization */
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_LOWER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_LOWER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetAcceleration(0, MAX_ACCEL_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetDeceleration(0, MAX_DECEL_UPPER_INIT);
	HAL_Delay(1);

	/* Default Starting Control Configuration */
	max_accel = MAX_ACCEL;
	max_decel = MAX_DECEL;
	max_speed = MAX_SPEED_MODE_1;
	min_speed = MIN_SPEED_MODE_1;
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, max_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, min_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetAcceleration(0, max_accel);
	HAL_Delay(1);
	BSP_MotorControl_SetDeceleration(0, max_decel);
	HAL_Delay(1);

	/* Default torque current */
	torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

	/* Default controller gains */
	proportional = PRIMARY_PROPORTIONAL_MODE_1;
	integral = PRIMARY_INTEGRAL_MODE_1;
	derivative = PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;

	/* Enable State Feedback mode and Integral Action Compensator by default and set
	 * precompensation factor to unity
	 */
	enable_state_feedback = 1;
	integral_compensator_gain = 0;
	feedforward_gain = 1;

	/* Disable adaptive_mode by default */
	enable_adaptive_mode = 0;

	/* DMA Buffer declarations */
	/* Start DMA just once because it's configured in "circular" mode */
	HAL_UART_Receive_DMA(&huart2, RxBuffer, UART_RX_BUFFER_SIZE);

	/* Motor Interface and Encoder initialization */
	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

	/* Attach the function Error_Handler (defined below) to the error Handler*/
	BSP_MotorControl_AttachErrorHandler(Error_Handler);

	/* Encoder initialization */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	/* Assign user interaction mode string values */
	set_mode_strings();

	/* Controller structure and variable allocation */
	current_error_steps = malloc(sizeof(float));
	if (current_error_steps == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}
	current_error_rotor_steps = malloc(sizeof(float));
	if (current_error_rotor_steps == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}
	sample_period = malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}
	deriv_lp_corner_f = malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}
	deriv_lp_corner_f_rotor = malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}
	sample_period_rotor = malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(test_msg, "Memory allocation error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	if (RCC_SYS_CLOCK_FREQ != HAL_RCC_GetSysClockFreq()) {
		sprintf(test_msg, "RCC_SYS_CLOCK_FREQ not equal to HAL_RCC_GetSysClockFreq() (%lu). Exiting.\r\n", HAL_RCC_GetSysClockFreq());
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg), HAL_MAX_DELAY);
	}
	if (RCC_HCLK_FREQ != HAL_RCC_GetHCLKFreq()) {
		sprintf(test_msg, "RCC_HCLK_FREQ not equal to HAL_RCC_GetHCLKFreq() (%lu). Exiting.\r\n", HAL_RCC_GetHCLKFreq());
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg), HAL_MAX_DELAY);
	}

	/* Configure controller filter and sample time parameters */
	*deriv_lp_corner_f = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	*deriv_lp_corner_f_rotor = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	t_sample_cpu_cycles = (uint32_t) round(T_SAMPLE_DEFAULT * RCC_HCLK_FREQ);
	Tsample = (float) t_sample_cpu_cycles / RCC_HCLK_FREQ;
	*sample_period = Tsample;
	Tsample_rotor = Tsample;
	*sample_period_rotor = Tsample_rotor;

	/* PID Derivative Low Pass Filter Coefficients */

	fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (*sample_period));
	Deriv_Filt_Pend[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Pend[1] = Deriv_Filt_Pend[0] * (1 - IWon_t);

	fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (*sample_period));
	Deriv_Filt_Rotor[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Rotor[1] = Deriv_Filt_Rotor[0] * (1 - IWon_t);

	/* Initialize real time clock */
	assert(RCC_SYS_CLOCK_FREQ == HAL_RCC_GetSysClockFreq());
	assert(RCC_HCLK_FREQ == HAL_RCC_GetHCLKFreq());

	/* Configure primary controller parameters */
	windup = PRIMARY_WINDUP_LIMIT;

	/* Configure secondary Rotor controller parameters */
	rotor_windup = SECONDARY_WINDUP_LIMIT;

	/* Compute Low Pass Filter Coefficients for Rotor Position filter and Encoder Angle Slope Correction */
	fo = LP_CORNER_FREQ_ROTOR;
	Wo = 2 * 3.141592654 * fo;
	IWon = 2 / (Wo * Tsample);
	iir_0 = 1 / (1 + IWon);
	iir_1 = iir_0;
	iir_2 = iir_0 * (1 - IWon);
	fo_s = LP_CORNER_FREQ_STEP;
	Wo_s = 2 * 3.141592654 * fo_s;
	IWon_s = 2 / (Wo_s * Tsample);
	iir_0_s = 1 / (1 + IWon_s);
	iir_1_s = iir_0_s;
	iir_2_s = iir_0_s * (1 - IWon_s);
	fo_LT = LP_CORNER_FREQ_LONG_TERM;
	Wo_LT = 2 * 3.141592654 * fo_LT;
	IWon_LT = 2 / (Wo_LT * Tsample);
	iir_LT_0 = 1 / (1 + IWon_LT);
	iir_LT_1 = iir_LT_0;
	iir_LT_2 = iir_LT_0 * (1 - IWon_LT);

	/*
	 * Primary Controller Mode Configuration Loop
	 *
	 * Outer Loop acquires configuration command
	 * Inner Loop includes control system
	 * Inner Loop exits to Outer Loop upon command or exceedance
	 * of rotor or pendulum angles
	 *
	 * Outer Loop provided user-selected system reset option during
	 * data entry by Serial Interface
	 *
	 */


	tick_read_cycle_start = HAL_GetTick();
	sprintf(msg, "\n\rSystem Starting Prepare to Enter Mode Selection... ");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	/*
	 * Request user input for mode configuration
	 */

	enable_adaptive_mode = ENABLE_ADAPTIVE_MODE;
	adaptive_threshold_low = ADAPTIVE_THRESHOLD_LOW;
	adaptive_threshold_high = ADAPTIVE_THRESHOLD_HIGH;
	adaptive_state = ADAPTIVE_STATE;
	adaptive_state_change = 0;
	adaptive_dwell_period = ADAPTIVE_DWELL_PERIOD;

	while (1) {

		mode_interactive = 0;
		user_prompt();

		/*
		 * If user has responded to previous query for configuration, then system remains in interactive mode
		 * and default state is not automatically enabled
		 */


		if (mode_interactive == 0) {
			sprintf(msg, "\n\rEnter Mode Selection Now or System Will Start in Default Mode in 5 Seconds..: ");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}

		/*
		 * If user has responded to query for configuration, then system remains in interactive mode
		 * and default state is not automatically enabled
		 */

		if (mode_interactive == 1) {
			sprintf(msg, "\n\rEnter Mode Selection Now: \n\r");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}

		/* Flush read buffer  */
		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }
		/* Start timer for configuration command read loop */
		tick_read_cycle_start = HAL_GetTick();
		/* Configuration command read loop */
		user_configuration();

		/* Set Motor Speed Profile and torque current */
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
		BSP_MotorControl_SetMaxSpeed(0, max_speed);
		BSP_MotorControl_SetMinSpeed(0, min_speed);
		BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
		BSP_MotorControl_SetDeceleration(0, MAX_DECEL);

		/* Report configuration values */
		if (ACCEL_CONTROL == 0){
		sprintf(msg, "\n\rMotor Profile Speeds Set at Min %u Max %u Steps per Second",
				min_speed, max_speed);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);
		}
			if (select_suspended_mode == 0){
		sprintf(msg, "\n\rInverted Pendulum Mode Selected");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);
			}
			if (select_suspended_mode == 1){
		sprintf(msg, "\n\rSuspended Pendulum Mode Selected");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);
			}

		sprintf(msg, "\n\rMotor Torque Current Set at %0.1f mA",
				torq_current_val);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);

		/* Motor Control Characterization Test*/
		if (enable_motor_actuator_characterization_mode == 1) {
			motor_actuator_characterization_mode();
		}
		/* Interactive digital motor control system */
		if (enable_rotor_actuator_control == 1) {
			interactive_rotor_actuator_control();
		}

		/*
		 * 	Rotor and Encoder Test Sequence will execute by moving rotor and reportin angle values
		 * 	as well as requesting pendulum motion followed by reporting of pendulum angles
		 *
		 * 	Agreement between actions and reported values confirms proper installation of actuator
		 * 	and pendulum encoder.
		 *
		 */

		if (enable_rotor_actuator_test == 1) {
			rotor_encoder_test();
		}

		/*
		 * Configure Primary and Secondary PID controller data structures
		 * Scale by CONTROLLER_GAIN_SCALE set to default value of unity
		 */


		PID_Pend.Kp = proportional * CONTROLLER_GAIN_SCALE;
		PID_Pend.Ki = integral * CONTROLLER_GAIN_SCALE;
		PID_Pend.Kd = derivative * CONTROLLER_GAIN_SCALE;

		PID_Rotor.Kp = rotor_p_gain * CONTROLLER_GAIN_SCALE;
		PID_Rotor.Ki = rotor_i_gain * CONTROLLER_GAIN_SCALE;
		PID_Rotor.Kd = rotor_d_gain * CONTROLLER_GAIN_SCALE;

		PID_Pend.Kp = proportional * CONTROLLER_GAIN_SCALE;
		PID_Pend.Ki = integral * CONTROLLER_GAIN_SCALE;
		PID_Pend.Kd = derivative * CONTROLLER_GAIN_SCALE;

		PID_Rotor.Kp = rotor_p_gain * CONTROLLER_GAIN_SCALE;
		PID_Rotor.Ki = rotor_i_gain * CONTROLLER_GAIN_SCALE;
		PID_Rotor.Kd = rotor_d_gain * CONTROLLER_GAIN_SCALE;

		PID_Pend.state_a[0] = 0;
		PID_Pend.state_a[1] = 0;
		PID_Pend.state_a[2] = 0;
		PID_Pend.state_a[3] = 0;

		PID_Rotor.state_a[0] = 0;
		PID_Rotor.state_a[1] = 0;
		PID_Rotor.state_a[2] = 0;
		PID_Rotor.state_a[3] = 0;

		integral_compensator_gain = integral_compensator_gain * CONTROLLER_GAIN_SCALE;

		/* Assign Rotor Plant Design variable values */


		/* Transfer function model of form 1/(s^2 + 2*Damping_Coefficient*Wn*s + Wn^2) */
		if (rotor_damping_coefficient != 0 || rotor_natural_frequency != 0){
			Wn2 = rotor_natural_frequency * rotor_natural_frequency;
			rotor_plant_gain = rotor_plant_gain * Wn2;
			ao = ((2.0F/Tsample)*(2.0F/Tsample) + (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
					+ rotor_natural_frequency*rotor_natural_frequency);
			c0 = ((2.0F/Tsample)*(2.0F/Tsample)/ao);
			c1 = -2.0F * c0;
			c2 = c0;
			c3 = -(2.0F*rotor_natural_frequency*rotor_natural_frequency - 2.0F*(2.0F/Tsample)*(2.0F/Tsample))/ao;
			c4 = -((2.0F/Tsample)*(2.0F/Tsample) - (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
					+ rotor_natural_frequency*rotor_natural_frequency)/ao;
		}

		/* Transfer function model of form 1/(s^2 + Wn*s) */
		if (enable_rotor_plant_design == 2){
			IWon_r = 2 / (Wo_r * Tsample);
			iir_0_r = 1 - (1 / (1 + IWon_r));
			iir_1_r = -iir_0_r;
			iir_2_r = (1 / (1 + IWon_r)) * (1 - IWon_r);
		}

		/* Optional Transfer function model of form Wn/(s^3 + Wn*s^2)
		if (enable_rotor_plant_design == 3 && enable_state_feedback == 0){
		      IWon_r = 2 / (Wo_r * Tsample);
		      iir_0_r = 1 / (1 + IWon_r);
		      iir_1_r = iir_0_r;
		      iir_2_r = iir_0_r * (1 - IWon_r);
		}
		*/

		/*

		//Optional display coefficients for rotor plant design transfer function
		sprintf(tmp_string, "\n\rEnable Design: %i iir_0 %0.4f iir_1 %0.4f iir_2 %0.4f\n\r", enable_rotor_plant_design, iir_0_r, iir_1_r, iir_2_r);
		HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);


		//Optional display coefficients for rotor plant design transfer function
		sprintf(tmp_string,
				"\n\ra0 %0.4f c0 %0.4f c1 %0.4f c2 %0.4f c3 %0.4f c4 %0.4f\n\r", ao, c0, c1, c2, c3, c4);
		HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

		 */


		/*
		 * *************************************************************************************************
		 *
		 * Control System Initialization Sequence
		 *
		 * *************************************************************************************************
		 */

		/* Setting enable_control_action enables control loop */
		enable_control_action = ENABLE_CONTROL_ACTION;

		/*
		 * Set Motor Position Zero occuring only once after reset and suppressed thereafter
		 * to maintain angle calibration
		 */

		if (reset_state == 1){
			rotor_position_set();
		}
		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg,
				"\r\nPrepare for Control Start - Initial Rotor Position: %i\r\n",
				rotor_position_steps);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);


		/*
		 * Determination of vertical down orientation of the pendulum is required
		 * to establish the reference angle for the vertical upward control setpoint.
		 *
		 * This is measured when the pendulum is determined to be motionless.
		 *
		 * The user is informed to allow the pendulum to remain at rest.
		 *
		 * Motion is detected in the loop below.  Exit from the loop and
		 * initiation of control occurs next.
		 *
		 * Prior to measurement, and due to previous action, the Pendulum may be poised
		 * at upright orientation.
		 *
		 * A small stimulus is applied to ensure Pendulum will fall to Suspended orientation
		 * in the event that it may be finely balanced in the vertical position
		 *
		 */

		BSP_MotorControl_GoTo(0, 3);
		BSP_MotorControl_WaitWhileActive(0);
		HAL_Delay(150);
		BSP_MotorControl_GoTo(0, -3);
		BSP_MotorControl_WaitWhileActive(0);
		HAL_Delay(150);
		BSP_MotorControl_GoTo(0, 3);
		BSP_MotorControl_WaitWhileActive(0);
		HAL_Delay(150);
		BSP_MotorControl_GoTo(0, 0);
		BSP_MotorControl_WaitWhileActive(0);

		sprintf(msg, "Test for Pendulum at Rest - Waiting for Pendulum to Stabilize\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		encoder_position_init = 0;
		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_prev = encoder_position_steps;
		HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_curr = encoder_position_steps;
		while (encoder_position_curr != encoder_position_prev) {
			ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
			encoder_position_prev = encoder_position_steps;
			HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
			ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
			encoder_position_curr = encoder_position_steps;

			/*
			 * Ensure stability reached with final motion test
			 */

			if (encoder_position_prev == encoder_position_curr) {
				HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
				ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
				encoder_position_prev = encoder_position_steps;
				HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
				ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
				encoder_position_curr = encoder_position_steps;
				if (encoder_position_prev == encoder_position_curr) {
					break;
				}
			}
			/* Alert user of undesired motion */
			sprintf(msg, "Pendulum Motion Detected with angle %0.2f - Waiting for Pendulum to Stabilize\r\n",
					(float) ((encoder_position_curr - encoder_position_prev)
							/ ENCODER_READ_ANGLE_SCALE));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}

		sprintf(msg, "Pendulum Now at Rest and Measuring Pendulum Down Angle\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		/* Calibrate down angle */

		/*
		 * Initialize Pendulum Angle Read offset by setting encoder_position_init
		 */

		HAL_Delay(100);
		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_init = encoder_position_steps;

		if (ret == -1) {
			sprintf(msg, "Encoder Position Under Range Error\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}
		if (ret == 1) {
			sprintf(msg, "Encoder Position Over Range Error\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}

		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_down = encoder_position_steps;
		sprintf(msg, "Pendulum Initial Angle %i\r\n", encoder_position_steps);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);


		if (enable_swing_up == 0){
			/*
			 * Alert user with rotor motion prompt to adjust pendulum upright by
			 */
			BSP_MotorControl_GoTo(0, 30);
			BSP_MotorControl_WaitWhileActive(0);
			HAL_Delay(150);
			BSP_MotorControl_GoTo(0, -30);
			BSP_MotorControl_WaitWhileActive(0);
			HAL_Delay(150);
			BSP_MotorControl_GoTo(0, 30);
			BSP_MotorControl_WaitWhileActive(0);
			HAL_Delay(150);
			BSP_MotorControl_GoTo(0, 0);
			BSP_MotorControl_WaitWhileActive(0);

			/* Request user action to bring pendulum upright */
			if(select_suspended_mode == 0){
				sprintf(msg,
						"Adjust Pendulum Upright By Turning CCW Control Will Start When Vertical\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

		}

		/*
		 * Detect Start Condition for Pendulum Angle for Inverted Model
		 *
		 * Detect Pendulum Angle equal to vertical within tolerance of START_ANGLE
		 *
		 * Exit if no vertical orientation action detected and alert user to restart,
		 * then disable control and enable system restart.
		 *
		 * Permitted delay for user action is PENDULUM_ORIENTATION_START_DELAY.
		 *
		 */

		/*
		 * System start option with manual lifting of Pendulum to vertical by user
		 */

		if (enable_swing_up == 0){

			tick_wait_start = HAL_GetTick();
			if (select_suspended_mode == 0) {
				while (1){
					ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
					if (fabs(encoder_position_steps - encoder_position_down - (int) (180 * angle_scale)) < START_ANGLE * angle_scale){
						HAL_Delay(START_ANGLE_DELAY);
						break;
					}
					if (fabs(encoder_position_steps - encoder_position_down + (int)(180 * angle_scale)) < START_ANGLE * angle_scale){
						encoder_position_down = encoder_position_down - 2*(int)(180 * angle_scale);
						HAL_Delay(START_ANGLE_DELAY);
						break;
					}
					tick_wait = HAL_GetTick();

					if ( (tick_wait - tick_wait_start) > PENDULUM_ORIENTATION_START_DELAY){
						sprintf(msg, "Pendulum Upright Action Not Detected - Restarting ...\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						enable_control_action = 0;
						break;
					}
				}
			}
		}


		/*
		 * For case of Suspended Mode Operation, no initial condition check is required
		 * for pendulum down angle.
		 */

		if(select_suspended_mode == 1){
			sprintf(msg, "Suspended Mode Control Will Start in %i Seconds\r\n",
					(int) (CONTROL_START_DELAY / 1000));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}


		/*
		 * Initialize Primary and Secondary PID controllers
		 */

		*current_error_steps = 0;
		*current_error_rotor_steps = 0;
		PID_Pend.state_a[0] = 0;
		PID_Pend.state_a[1] = 0;
		PID_Pend.state_a[2] = 0;
		PID_Pend.state_a[3] = 0;
		PID_Pend.int_term = 0;
		PID_Pend.control_output = 0;
		PID_Rotor.state_a[0] = 0;
		PID_Rotor.state_a[1] = 0;
		PID_Rotor.state_a[2] = 0;
		PID_Rotor.state_a[3] = 0;
		PID_Rotor.int_term = 0;
		PID_Rotor.control_output = 0;

		/* Initialize Pendulum PID control state */
		pid_filter_control_execute(&PID_Pend, current_error_steps, sample_period,
				 Deriv_Filt_Pend);

		/* Initialize Rotor PID control state */
		*current_error_rotor_steps = 0;
		pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps,
				sample_period_rotor, Deriv_Filt_Rotor);

		/* Initialize control system variables */

		cycle_count = CYCLE_LIMIT;
		i = 0;
		rotor_position_steps = 0;
		rotor_position_steps_prev = 0;
		rotor_position_filter_steps = 0;
		rotor_position_filter_steps_prev = 0;
		rotor_position_command_steps = 0;
		rotor_position_diff = 0;
		rotor_position_diff_prev = 0;
		rotor_position_diff_filter = 0;
		rotor_position_diff_filter_prev = 0;
		rotor_position_step_polarity = 1;
		encoder_angle_slope_corr_steps = 0;
		rotor_sine_drive = 0;
		sine_drive_transition = 0;
		rotor_mod_control = 1.0;
		enable_adaptive_mode = 0;
		tick_cycle_start = HAL_GetTick();
		tick_cycle_previous = tick_cycle_start;
		tick_cycle_current =  tick_cycle_start;
		enable_cycle_delay_warning = ENABLE_CYCLE_DELAY_WARNING;
		chirp_cycle = 0;
		chirp_dwell_cycle = 0;
		pendulum_position_command_steps = 0;
		impulse_start_index = 0;
		mode_transition_state = 0;
		transition_to_adaptive_mode = 0;
		error_sum_prev = 0;
		error_sum_filter_prev = 0;
		adaptive_state = 4;
		rotor_control_target_steps_prev = 0;
		rotor_position_command_steps_prev = 0;
		rotor_position_command_steps_pf_prev = 0;
		enable_high_speed_sampling = ENABLE_HIGH_SPEED_SAMPLING_MODE;
		slope_prev = 0;
		rotor_track_comb_command = 0;
		noise_rej_signal_prev = 0;
		noise_rej_signal_filter_prev = 0;
		full_sysid_start_index = -1;
		current_cpu_cycle = 0;
		speed_scale = DATA_REPORT_SPEED_SCALE;
		speed_governor = 0;
		encoder_position_offset = 0;
		encoder_position_offset_zero = 0;

		for (m = 0; m < ANGLE_CAL_OFFSET_STEP_COUNT + 1; m++){
			offset_angle[m] = 0;
		}

		/* Clear read buffer */
		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
			Msg.Data[k] = 0;
		}
		/* Initialize UART receive system */
		__HAL_DMA_RESET_HANDLE_STATE(&hdma_usart2_rx);

		/*
		 * Record user selected operation variable values.  Values will be
		 * restored after Swing Up completion or after Angle Calibration
		 * completion
		 */

		init_r_p_gain = PID_Rotor.Kp;
		init_r_i_gain = PID_Rotor.Ki;
		init_r_d_gain = PID_Rotor.Kd;
		init_p_p_gain = PID_Pend.Kp;
		init_p_i_gain = PID_Pend.Ki;
		init_p_d_gain = PID_Pend.Kd;
		init_enable_state_feedback = enable_state_feedback;
		init_integral_compensator_gain = integral_compensator_gain;
		init_feedforward_gain = feedforward_gain;
		init_enable_state_feedback = enable_state_feedback;
		init_enable_disturbance_rejection_step = enable_disturbance_rejection_step;
		init_enable_sensitivity_fnc_step = enable_sensitivity_fnc_step;
		init_enable_noise_rejection_step = enable_noise_rejection_step;
		init_enable_rotor_plant_design = enable_rotor_plant_design;
		init_enable_rotor_plant_gain_design = enable_rotor_plant_gain_design;

		if(select_suspended_mode == 1){
			load_disturbance_sensitivity_scale = 1.0;
		}
		if(select_suspended_mode == 0){
			load_disturbance_sensitivity_scale = LOAD_DISTURBANCE_SENSITIVITY_SCALE;
		}


		/*
		 * Initiate Pendulum Swing Up with automatic system requiring no user action
		 *
		 * This system was developed by Markus Dauberschmidt see
		 * https://github.com/OevreFlataeker/steval_edukit_swingup
		 *
		 */


		if (enable_swing_up == 1 && select_suspended_mode == 0){

			/*
			 * Apply controller parameters for initial operation at completion of
			 * Swing Up
			 */

			PID_Rotor.Kp = 20;
			PID_Rotor.Ki = 10;
			PID_Rotor.Kd = 10;
			PID_Pend.Kp = 300;
			PID_Pend.Ki = 0.0;
			PID_Pend.Kd = 30.0;
			enable_state_feedback = 0;
			integral_compensator_gain = 0;
			feedforward_gain = 1;
			rotor_position_command_steps = 0;
			enable_state_feedback = 0;
			enable_disturbance_rejection_step = 0;
			enable_sensitivity_fnc_step = 0;
			enable_noise_rejection_step = 0;
			enable_rotor_plant_design = 0;
			enable_rotor_plant_gain_design = 0;

			/* Set Torque Current value to 800 mA (normal operation will revert to 400 mA */
			torq_current_val = MAX_TORQUE_SWING_UP;
			L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

			sprintf(msg, "Pendulum Swing Up Starting\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			/* Initialize position and motion variables */
			max_encoder_position = 0;
			global_max_encoder_position = 0;
			peaked = 0;
			handled_peak = 0;
			swing_up_state = 0;
			swing_up_state_prev = 0;
			zero_crossed = 0;
			stage_count = 0;
			/* Select initial amplitude for rotor impulse */
			stage_amp = STAGE_0_AMP;

			/* Optional encoder state reporting */
			//sprintf(tmp_string,"Current Position %0.2f\r\n", (encoder_position - encoder_position_down)/angle_scale);
			//HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

			//sprintf(tmp_string,"Current Position Down %0.2f\r\n", encoder_position_down/angle_scale);
			//HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

			/* Initiate first swing */
			swing_up_direction = FORWARD;
			BSP_MotorControl_Move(0, swing_up_direction, 150);
			BSP_MotorControl_WaitWhileActive(0);


			/* Enter Swing Up Loop */
			while (1)
			{
				HAL_Delay(2);
				ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
				/* Optional Swing Up progress reporting */
				//sprintf(tmp_string,"Rotor Impulse Amplitude %i Max Angle (degrees) %0.3f\r\n", stage_amp, fabs((float)(global_max_encoder_position)/(ENCODER_READ_ANGLE_SCALE)));
				//HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

				/* Break if pendulum angle relative to vertical meets tolerance (for clockwise or counter clockwise approach */
				if (fabs(encoder_position_steps - encoder_position_down - (int) (180 * angle_scale)) < START_ANGLE * angle_scale){
					break;
				}
				if (fabs(encoder_position_steps - encoder_position_down + (int)(180 * angle_scale)) < START_ANGLE * angle_scale){
					encoder_position_down = encoder_position_down - 2*(int)(180 * angle_scale);
					break;
				}

				if (zero_crossed)
				{
					zero_crossed = 0;
					// Push it aka put some more kinetic energy into the pendulum
					if (swing_up_state == 0){
						BSP_MotorControl_Move(0, swing_up_direction, stage_amp);
						BSP_MotorControl_WaitWhileActive(0);
						stage_count++;

						if (prev_global_max_encoder_position != global_max_encoder_position && stage_count > 4){
						if (abs(global_max_encoder_position) < 600){
							stage_amp = STAGE_0_AMP;
						}
						if (abs(global_max_encoder_position) >= 600 && abs(global_max_encoder_position) < 1000){
							stage_amp = STAGE_1_AMP;
						}
						if (abs(global_max_encoder_position) >= 1000){
							stage_amp = STAGE_2_AMP;
						}
						}
						prev_global_max_encoder_position = global_max_encoder_position;
						global_max_encoder_position = 0;
						ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
					}
				}


				// We have a peak but did not handle it yet
				if (peaked && !handled_peak)
				{
					// Ensure we only enter this branch one per peak
					handled_peak = 1;
					// Reset maximum encoder value to reassess after crossing the bottom
					max_encoder_position = 0;
					// Switch motor direction
					swing_up_direction = swing_up_direction == FORWARD ? BACKWARD : FORWARD;
				}
			}
		}

		/*
		 * *************************************************************************************************
		 *
		 * Control Loop Start
		 *
		 * *************************************************************************************************
		 */

		enable_control_action = 1;

		if (ACCEL_CONTROL == 1) {
			BSP_MotorControl_HardStop(0);
			L6474_CmdEnable(0);
			target_velocity_prescaled = 0;
			L6474_Board_SetDirectionGpio(0, BACKWARD);
		}

		/*
		 * Set Torque Current to value for normal operation
		 */
		torq_current_val = MAX_TORQUE_CONFIG;
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

		target_cpu_cycle = DWT->CYCCNT;
		prev_cpu_cycle = DWT->CYCCNT;

		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		if (select_suspended_mode == 0) {
			encoder_position = encoder_position_steps - encoder_position_down - (int)(180 * angle_scale);
			encoder_position = encoder_position - encoder_position_offset;
		}

		while (enable_control_action == 1) {


			/*
			 *
			 * Restore user selected control parameters after completion of Swing Up and if Angle Calibration
			 * not enabled.  Start up delay permits settling prior to switching to new control parameters
			 *
			 */

			if (enable_swing_up == 1 && i == SWING_UP_CONTROL_CONFIG_DELAY && enable_angle_cal == 0){
				PID_Rotor.Kp = init_r_p_gain;
				PID_Rotor.Ki = init_r_i_gain;
				PID_Rotor.Kd = init_r_d_gain;
				PID_Pend.Kp = init_p_p_gain;
				PID_Pend.Ki = init_p_i_gain;
				PID_Pend.Kd = init_p_d_gain;
				enable_state_feedback = init_enable_state_feedback;
				integral_compensator_gain = init_integral_compensator_gain;
				feedforward_gain = init_feedforward_gain;
				enable_state_feedback = init_enable_state_feedback;
				enable_disturbance_rejection_step = init_enable_disturbance_rejection_step;
				enable_sensitivity_fnc_step = init_enable_sensitivity_fnc_step;
				enable_noise_rejection_step = init_enable_noise_rejection_step;
				enable_rotor_plant_design = init_enable_rotor_plant_design;
				enable_rotor_plant_gain_design = init_enable_rotor_plant_gain_design;
			}

			/*
			 *  Real time user configuration and mode assignment
			 */

			mode_index_prev = mode_index;

			RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE
					- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx,
					RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

			config_command = 0;
			/* Test for NULL message received corresponding to keyboard Carriage Return */
			if (readBytes == 1){
				RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;
				continue;
			}
			/* Test for message received */
			if (readBytes == 2 && Msg.Len == 1 && i % 10 == 0){
				RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;
				mode_transition_state = 1;
				/* Determine user input */
				mode_index_command = mode_index_identification((char *)Msg.Data, config_command, & adjust_increment,
						&PID_Pend, &PID_Rotor);
				strcpy(config_message, (char *) Msg.Data);
				if (strcmp(config_message, ">") == 0){
					// Logging was activated from the Real-Time Workbench
					if (enable_full_sysid && full_sysid_start_index == -1) {
						full_sysid_start_index = i + 50;
					}
				} else if (strcmp(config_message, "q") == 0){
					sprintf(tmp_string,
							"\n\rExit Control Loop Command Received ");
					HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string,
							strlen(tmp_string), HAL_MAX_DELAY);
					break;
				}
			}

			/* Set mode 1 if user request detected */
			if (mode_index_command == 1 && mode_transition_state == 1) {
				mode_index = 1;
				mode_transition_state = 0;
				mode_index_command = 0;
				assign_mode_1(&PID_Pend, &PID_Rotor);
			}
			/* Set mode 3 if user request detected */
			if (mode_index_command == 2 && mode_transition_state == 1) {
				mode_index = 2;
				mode_transition_state = 0;
				mode_index_command = 0;
				assign_mode_2(&PID_Pend, &PID_Rotor);
			}
			/* Set mode 3 if user request detected */
			if (mode_index_command == 3 && mode_transition_state == 1) {
				mode_index = 3;
				mode_transition_state = 0;
				mode_index_command = 0;
				assign_mode_3(&PID_Pend, &PID_Rotor);
			}
			/* End of Real time user configuration and mode assignment read loop */


			/* Exit control if cycle count limit set */

			if (i > cycle_count && ENABLE_CYCLE_INFINITE == 0) {
				break;
			}

			/*
			 * *************************************************************************************************
			 *
			 * Initiate Measurement and Control
			 *
			 * *************************************************************************************************
			 */

			/*
			 * Optional Reset and clear integrator error during initial start of controllers
			 */
			if (i < 1){
				PID_Pend.int_term = 0;
				PID_Rotor.int_term = 0;
			}

			/*
			 * Acquire encoder position and correct for initial angle value of encoder measured at
			 * vertical down position at system start including 180 degree offset corresponding to
			 * vertical upwards orientation.
			 *
			 * For case of Suspended Mode Operation the 180 degree offset is not applied
			 *
			 * The encoder_position_offset variable value is determined by the Automatic Inclination
			 * Angle Calibration system
			 */

			ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
			if (select_suspended_mode == 0) {
				encoder_position = encoder_position_steps - encoder_position_down - (int)(180 * angle_scale);
				encoder_position = encoder_position - encoder_position_offset;
			}
			if (select_suspended_mode == 1) {
				encoder_position = encoder_position_steps - encoder_position_down;
				encoder_position = encoder_position - encoder_position_offset;
			}

			/*  Detect pendulum position excursion exceeding limits and exit */

			if(select_suspended_mode == 0){
				if (((encoder_position)/ ENCODER_READ_ANGLE_SCALE)
						> ENCODER_POSITION_POSITIVE_LIMIT) {
					sprintf(msg, "Error Exit Encoder Position Exceeded: %i\r\n",
							encoder_position_steps);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							HAL_MAX_DELAY);
					break;
				}
				if (((encoder_position)/ ENCODER_READ_ANGLE_SCALE)
						< ENCODER_POSITION_NEGATIVE_LIMIT) {
					sprintf(msg, "Error Exit Encoder Position Exceeded: %i\r\n",
							encoder_position_steps);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							HAL_MAX_DELAY);
					break;
				}

			}

			/* Detect rotor position excursion exceeding limits and exit */

			if (rotor_position_steps
					> (ROTOR_POSITION_POSITIVE_LIMIT
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
				sprintf(msg, "Error Exit Motor Position Exceeded: %i\r\n",
						rotor_position_steps);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						HAL_MAX_DELAY);
				break;
			}

			if (rotor_position_steps
					< (ROTOR_POSITION_NEGATIVE_LIMIT
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
				sprintf(msg, "Error Exit Motor Position Exceeded: %i\r\n",
						rotor_position_steps);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						HAL_MAX_DELAY);
				break;
			}

			/*
			 * Encoder Angle Error Compensation
			 *
			 * Compute Proportional control of pendulum angle compensating for error due to
			 * encoder offset at start time or system platform slope relative to horizontal.
			 *
			 * Apply optional time limit to correct for encoder error or slope.  For cycle count
			 * greater than ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT, encoder angle
			 * slope correction remains constant.
			 *
			 * If ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT = 0, then encoder angle slope
			 * correction continues operation for all time
			 *
			 * Note: This system is *not* required in the event that Automatic Inclination
			 * Angle Calibration is selected.
			 *
			 */

			/* Compute Low Pass Filtered rotor position difference */

			rotor_position_diff_prev = rotor_position_diff;

			if (enable_disturbance_rejection_step == 0){
				rotor_position_diff = rotor_position_filter_steps
						- rotor_position_command_steps;
			}
			if (enable_disturbance_rejection_step == 1){
				rotor_position_diff = rotor_position_filter_steps;
			}



			/* Apply slope correction */
			if (ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION == 1 && i > angle_cal_complete) {
				rotor_position_diff_filter =
						(float) (rotor_position_diff * iir_LT_0)
						+ rotor_position_diff_prev * iir_LT_1
						- rotor_position_diff_filter_prev * iir_LT_2;
				if ((i < ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT) || (ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT == 0)) {
					encoder_angle_slope_corr_steps = rotor_position_diff_filter / ENCODER_ANGLE_SLOPE_CORRECTION_SCALE;
				}
				rotor_position_diff_filter_prev = rotor_position_diff_filter;
			}


			/*
			 *  Compute current_error_steps input for Primary Controller
			 *
			 *  current_error_steps is sum of encoder angle error compensation and encoder position
			 *  in terms of stepper motor steps
			 *
			 *  An Encoder offset may be introduced.  The Encoder offset may remain at all times if
			 *  ENCODER_OFFSET_DELAY == 0 or terminate at a time (in ticks) of ENCODER_OFFSET_DELAY
			 */

			if ((i > ENCODER_START_OFFSET_DELAY) || (ENCODER_START_OFFSET_DELAY == 0)){
				encoder_position = encoder_position - ENCODER_START_OFFSET;
			}

			/*
			 * Compute error between Pendulum Angle and Pendulum Tracking Angle in units of steps
			 * Apply scale factor to match angle to step gain of rotor actuator
			 *
			 */

			*current_error_steps = encoder_angle_slope_corr_steps
			+ ENCODER_ANGLE_POLARITY * (encoder_position / ((float)(ENCODER_READ_ANGLE_SCALE/STEPPER_READ_POSITION_STEPS_PER_DEGREE)));

			/*
			 *
			 * Pendulum Controller execution
			 *
			 * Include addition of Pendulum Angle track signal impulse signal
			 * Compute control signal, rotor position target in step units
			 *
			 * Pendulum tracking command, pendulum_position_command, also supplied in step units
			 *
			 */

			*current_error_steps = *current_error_steps + pendulum_position_command_steps;

			pid_filter_control_execute(&PID_Pend,current_error_steps, sample_period, Deriv_Filt_Pend);

			rotor_control_target_steps = PID_Pend.control_output;

			/* Acquire rotor position and compute low pass filtered rotor position */


			ret = rotor_position_read(&rotor_position_steps);

			/* Optional rotor position filter */

			rotor_position_filter_steps = (float) (rotor_position_steps) * iir_0
					+ rotor_position_steps_prev * iir_1
					- rotor_position_filter_steps_prev * iir_2;
			rotor_position_steps_prev = (float) (rotor_position_steps);
			rotor_position_filter_steps_prev = rotor_position_filter_steps;


			rotor_position_filter_steps = rotor_position_steps;

			/*
			 * 		Compute rotor chirp tracking signal with chirp sweep from rotor_chirp_start_freq
			 * 		to rotor_chirp_end_freq in time period rotor_chirp_period in units of control
			 * 		loop cycle periods.  Each chirp is separated by delay of ROTOR_CHIRP_SWEEP_DELAY.
			 */

			if (enable_rotor_chirp == 1 && enable_mod_sin_rotor_tracking == 0
					&& enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {

				if (i < ROTOR_CHIRP_PERIOD - 1){
					chirp_cycle = 0;
				}
				if (chirp_cycle > ROTOR_CHIRP_PERIOD - 1) {
					chirp_cycle = 0;
					chirp_dwell_cycle = ROTOR_CHIRP_SWEEP_DELAY;
				}
				if (chirp_dwell_cycle > 0){
					chirp_dwell_cycle--;
					chirp_cycle = 0;
				}
				if (chirp_dwell_cycle == 0 && i >= ROTOR_CHIRP_PERIOD - 1){
					chirp_cycle = chirp_cycle + 1;
					chirp_time = (float)((chirp_cycle - 1)/ROTOR_CHIRP_SAMPLE_RATE);
					rotor_chirp_frequency = rotor_chirp_start_freq + (rotor_chirp_end_freq - rotor_chirp_start_freq)*((float)(chirp_cycle/rotor_chirp_period));
					rotor_position_command_steps = ((float)(ROTOR_CHIRP_STEP_AMPLITUDE*STEPPER_READ_POSITION_STEPS_PER_DEGREE))*sin(2.0*3.14159*rotor_chirp_frequency*chirp_time);
				}
			}


			/*  Create rotor track "comb" signal */
			if (enable_rotor_tracking_comb_signal > 0 && i > 1000 && i > angle_cal_complete) {

				chirp_time = ((float)(i - 1))/500.0;
				rotor_track_comb_signal_frequency = 0.01;
				rotor_track_comb_command = ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.017783;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.031623;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.056234;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.1;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.17783;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.31623;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 0.56234;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 1.0;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 1.7783;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 3.1623;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 5.6234;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
				rotor_track_comb_signal_frequency = 10;
				rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
			}

			if (enable_rotor_chirp == 0 && enable_mod_sin_rotor_tracking == 0
					&& enable_rotor_tracking_comb_signal == 1) {
				rotor_position_command_steps = rotor_track_comb_command;
			}

			/*  Create rotor angle reference tracking modulated sine signal */

			rotor_sine_drive = 0;
			if (enable_mod_sin_rotor_tracking == 1 && ENABLE_ROTOR_CHIRP == 0 && i > angle_cal_complete) {

				if (ENABLE_ROTOR_CHIRP == 0){
					mod_sin_carrier_frequency = MOD_SIN_CARRIER_FREQ;
				}

				if (i > MOD_SIN_START_CYCLES && enable_mod_sin_rotor_tracking == 1) {
					rotor_sine_drive =
							(float) (mod_sin_amplitude
									* (1 + sin(-1.5707 + ((i - MOD_SIN_START_CYCLES)/MOD_SIN_SAMPLE_RATE) * (MOD_SIN_MODULATION_FREQ * 6.2832))));
					rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
					rotor_sine_drive = rotor_sine_drive + MOD_SIN_MODULATION_MIN;
					rotor_sine_drive = rotor_sine_drive * rotor_sine_drive_mod * rotor_mod_control;
				}

				if (i > MOD_SIN_START_CYCLES && ENABLE_SIN_MOD == 0) {
					rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
					rotor_sine_drive = rotor_control_sin_amplitude * rotor_sine_drive_mod * rotor_mod_control;
				}

				if ( fabs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 1 && sine_drive_transition == 1){
					rotor_mod_control = 0.0;
					sine_drive_transition = 0;
				}
				if ( fabs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 0 && sine_drive_transition == 1){
					rotor_mod_control = 1.0;
					sine_drive_transition = 0;
				}

				if (enable_rotor_position_step_response_cycle == 0){
					rotor_position_command_steps = rotor_sine_drive;
				}

			}

			/*  Create rotor angle reference tracking impulse signal */

			if (ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE == 1 && i != 0 && i > angle_cal_complete) {
				if ((i % ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
					rotor_position_command_steps =
							(float) (ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
									* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
					impulse_start_index = 0;
				}
				if (impulse_start_index
						> ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
					rotor_position_command_steps = 0;
				}
				impulse_start_index++;
			}

			/*
			 * Create pendulum angle reference tracking impulse signal.  Polarity of impulse alternates
			 */

			if (enable_pendulum_position_impulse_response_cycle == 1 && i != 0 && i > angle_cal_complete) {

				if ((i % PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
					if (select_suspended_mode == 1) {
						pendulum_position_command_steps =
								(float) PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE;
					}
					if (select_suspended_mode == 0) {
						pendulum_position_command_steps =
								(float) (PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
										/PENDULUM_POSITION_IMPULSE_AMPLITUDE_SCALE);
					}
					chirp_cycle = 0;
					impulse_start_index = 0;
				}
				if (impulse_start_index
						> PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
					pendulum_position_command_steps = 0;
				}
				impulse_start_index++;
				chirp_cycle++;
			}

			/*  Create rotor angle reference tracking  step signal */

			if ((i % ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL) == 0 && enable_rotor_position_step_response_cycle == 1 && i > angle_cal_complete) {
				rotor_position_step_polarity = -rotor_position_step_polarity;
				if (rotor_position_step_polarity == 1){
					chirp_cycle = 0;
				}
			}

			if (enable_rotor_position_step_response_cycle == 1 && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {
				if (STEP_RESPONSE_AMP_LIMIT_ENABLE == 1 && abs(rotor_sine_drive) > STEP_RESPONSE_AMP_LIMIT){
					chirp_cycle = chirp_cycle + 1;
				} else {
					if (enable_mod_sin_rotor_tracking == 1){
						rotor_position_command_steps = rotor_sine_drive + (float) ((rotor_position_step_polarity)
								* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
								* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
					}
					if (enable_mod_sin_rotor_tracking == 0){
						rotor_position_command_steps_pf = (float) ((rotor_position_step_polarity)
								* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
								* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
					}
					chirp_cycle = chirp_cycle + 1;
				}
			}

			/*
			 * Rotor tracking reference, rotor_position_command_steps, is low pass filtered to prevent
			 * aliasing of measurement during operation of Real Time Workbench sampling that occurs at
			 * 50 Hz (in support of connected computing platform bandwidth limitations).  This filter
			 * application is not applied during selection of high speed sampling at 500 Hz.
			 */

			if (enable_rotor_position_step_response_cycle == 1 && enable_mod_sin_rotor_tracking == 0 && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete){
				rotor_position_command_steps = rotor_position_command_steps_pf * iir_0_s
						+ rotor_position_command_steps_pf_prev * iir_1_s
						- rotor_position_command_steps_prev * iir_2_s;
				rotor_position_command_steps_pf_prev = rotor_position_command_steps_pf;
			}

			/*
			 *  Automatic Inclination Angle Calibration System
			 *
			 *  The Edukit system may be resting on a surface with a slight incline. This then
			 *  produces a Rotor Angle dependent error between the measurement of Pendulum Angle
			 *  and the angle corresponding to true vertical of the gravitational vector.
			 *
			 *  This system computed true vertical angle relative to the gravity vector for each
			 *  Rotor Step.  This provides an encoder_offset_angle calibration value for all
			 *  orientations of the Rotor.
			 */

			if (enable_angle_cal == 1){

				/*
				 * Angle Calibration system state values applied during Angle Calibration
				 * Period.  User selected system state values restored after Angle Calibration
				 */

				if (i == 1 && select_suspended_mode == 0){
					PID_Rotor.Kp = 21.1;
					PID_Rotor.Ki = 0;
					PID_Rotor.Kd = 17.2;
					PID_Pend.Kp = 419;
					PID_Pend.Ki = 0.0;
					PID_Pend.Kd = 56;
					enable_state_feedback = 1;
					integral_compensator_gain = 10;
					feedforward_gain = 1;
					rotor_position_command_steps = 0;
					current_error_rotor_integral = 0;
				}

				if (i == 1 && select_suspended_mode == 1){
					PID_Rotor.Kp = -23.86;
					PID_Rotor.Ki = 0;
					PID_Rotor.Kd = -19.2;
					PID_Pend.Kp = -293.2;
					PID_Pend.Ki = 0.0;
					PID_Pend.Kd = -41.4;
					enable_state_feedback = 1;
					integral_compensator_gain = -11.45;
					feedforward_gain = 1;
					rotor_position_command_steps = 0;
					current_error_rotor_integral = 0;
				}


				/* Initialize angle calibration variables */

				if (i == 1){
					offset_end_state = 0;
					offset_start_index = 4000;					// initial start index for sweep
					angle_index = ANGLE_CAL_OFFSET_STEP_COUNT;	// Number of angle steps
					angle_cal_end = INT32_MAX;
					angle_cal_complete = INT32_MAX;				// Allowed start time for stimulus signals
					encoder_position_offset_zero = 0;
				}

				if (offset_end_state == 0){
					/* Suspend loop delay warning since computation may lead to control loop cycle delay during
					 * period after measurement and during computation of smoothed offset data array
					 */
					enable_cycle_delay_warning = 0;
					/* Advance to upper angle of 90 degrees*/
					if (i > 1 && i < 4000){
						rotor_position_command_steps = (i/4000.0) * ANGLE_CAL_OFFSET_STEP_COUNT/2;
						offset_start_index = i + 4000;
					}
					/* Delay for time increment to avoid transient response in measurement.
					 * Acquire samples for time-average of offset
					 */
					if (i >= offset_start_index && i < (offset_start_index + 10)){
						//offset_angle[angle_index] = offset_angle[angle_index] + encoder_position;
						//offset_angle[angle_index] = encoder_position;
					}
					/* Compute time-averages offset and advance to next lower angle increment */
					if (i == offset_start_index + 10 && angle_index > 0){
						offset_angle[angle_index] = encoder_position;
						//offset_angle[angle_index] = offset_angle[angle_index]/10;
						angle_index = angle_index - 1;
						offset_start_index = offset_start_index + 10;
						rotor_position_command_steps = rotor_position_command_steps - 1;
					}

					/* Compute average encoder position offset over angle index range from ANGLE_AVG_SPAN to ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN */
					/* Suspend delay warning */

					if (angle_index >= 2*ANGLE_AVG_SPAN && angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1){
						for (angle_avg_index = angle_index - 2*ANGLE_AVG_SPAN; angle_avg_index < (angle_index + 1); angle_avg_index++){
							encoder_position_offset_avg[angle_index] = 0;
									for (angle_avg_index = angle_index - ANGLE_AVG_SPAN; angle_avg_index < (1 + angle_index + ANGLE_AVG_SPAN); angle_avg_index++){
											encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index] + offset_angle[angle_avg_index];
									}
									encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index]/(float)(2*ANGLE_AVG_SPAN + 1);
						}
					}

					/* Restore rotor angle to zero degrees */
					if (angle_index == 0){
						rotor_position_command_steps = rotor_position_command_steps + 0.02*STEPPER_READ_POSITION_STEPS_PER_DEGREE;
					}
					/* Terminate offset measurement and initialize angle_cal_end at time of termination */
					if (rotor_position_command_steps >= 0 && angle_index == 0){
						offset_end_state = 1;
						angle_cal_end = i;
						/* Restore loop delay warning */
						enable_cycle_delay_warning = 1;
					}
				}
			}

			/* Apply offset angle for correction of pendulum angle according to rotor position */

			if (offset_end_state == 1 && i > angle_cal_end){
				/* Compute angle index corresponding to rotor position */
				angle_index = (int)((ANGLE_CAL_OFFSET_STEP_COUNT - 1)/2) + rotor_position_filter_steps;
				if (angle_index < ANGLE_AVG_SPAN ){
					angle_index = ANGLE_AVG_SPAN;
				}
				if (angle_index >  ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN ){
					angle_index =  ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN;
				}
				encoder_position_offset = 2.0 * encoder_position_offset_avg[angle_index];
			}

			/* Measure residual offset at zero rotor position */
			if (offset_end_state == 1 && i > angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING && i < angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL){
				encoder_position_offset_zero = encoder_position_offset_zero + encoder_position;
			}

			/* Correct offset angle array values for any residual offset */
			if (i == (angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL + 1)){
				encoder_position_offset_zero = encoder_position_offset_zero/ANGLE_CAL_ZERO_OFFSET_DWELL;
				for (angle_index = 0; angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1; angle_index++){
					encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index] + encoder_position_offset_zero;
				}
				angle_cal_complete = angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL + 1 + ANGLE_CAL_COMPLETION;
			}

			/* Restore user selected system state configuration */
			if (offset_end_state == 1 && (enable_angle_cal == 1) && i == angle_cal_complete + 1){
				PID_Rotor.Kp = init_r_p_gain;
				PID_Rotor.Ki = init_r_i_gain;
				PID_Rotor.Kd = init_r_d_gain;
				PID_Pend.Kp = init_p_p_gain;
				PID_Pend.Ki = init_p_i_gain;
				PID_Pend.Kd = init_p_d_gain;
				current_error_rotor_integral = 0;
				enable_state_feedback = init_enable_state_feedback;
				integral_compensator_gain = init_integral_compensator_gain;
				feedforward_gain = init_feedforward_gain;
				enable_state_feedback = init_enable_state_feedback;
				enable_disturbance_rejection_step = init_enable_disturbance_rejection_step;
				enable_sensitivity_fnc_step = init_enable_sensitivity_fnc_step;
				enable_noise_rejection_step = init_enable_noise_rejection_step;
				enable_rotor_plant_design = init_enable_rotor_plant_design;
			}


			if (ENABLE_DUAL_PID == 1) {

				/*
				 * Secondary Controller execution including Sensitivity Function computation
				 */

				if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 0){
					*current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
				}
				if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 1 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 0){
					*current_error_rotor_steps = rotor_position_filter_steps;
				}
				if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 1){
					*current_error_rotor_steps = rotor_position_filter_steps + rotor_position_command_steps;
				}

				if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 1 && enable_noise_rejection_step == 0){
					*current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
				}

				/*
				 * Select Reference signal input location at input of controller for Dual PID architecture
				 * for Output Feedback Architecture or at output of controller and plant input for Full State
				 * Feedback Architecture
				 */

				if (enable_state_feedback == 1){
					*current_error_rotor_steps = rotor_position_filter_steps;
				}

				/*
				 * PID input supplied in units of stepper motor steps with tracking error determined
				 * as difference between rotor position tracking command and rotor position in units
				 * of stepper motor steps.
				 */

				pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps,
						sample_period_rotor,  Deriv_Filt_Rotor);

				rotor_control_target_steps = PID_Pend.control_output + PID_Rotor.control_output;


				if (enable_state_feedback == 1 && integral_compensator_gain != 0){
					/*
					 * If integral action is included, state feedback plant input equals
					 * the time integral of difference between reference tracking signal and rotor angle,
					 * current_error_rotor_integral.  This is summed with the controller output, rotor_control_target_steps.
					 * The integral_compensator_gain as input by user includes multiplicative scale factor matching
					 * scaling of controller gain values.
					 */
					current_error_rotor_integral = current_error_rotor_integral + (rotor_position_command_steps*feedforward_gain - rotor_position_filter_steps)*(*sample_period_rotor);
					rotor_control_target_steps = rotor_control_target_steps - integral_compensator_gain*current_error_rotor_integral;
				}

				if (enable_state_feedback == 1 && integral_compensator_gain == 0){
					/*
					 * If integral compensator is not included, full state feedback plant input equals difference
					 * between control output and reference tracking signal with scale factor matching
					 * scaling of controller gain values.
					 *
					 * If Plant Design system is applied, the effects of small numerical error in transfer function
					 * computation is compensated for by removal of average offset error.
					 *
					 */

					rotor_control_target_steps = rotor_control_target_steps - rotor_position_command_steps*feedforward_gain;

				}


				/*
				 * Load Disturbance Sensitivity Function signal introduction with scale factor applied to increase
				 * amplitude of Load Disturbance signal to enhance signal to noise in measurement.  This scale factor
				 * then must be applied after data acquisition to compute proper Load Disturbance Sensitivity Function.
				 * Note that Load Disturbance Sensitivity Function value is typically less than -20 dB
				 *
				 */
				if (enable_disturbance_rejection_step == 1){
					rotor_control_target_steps = rotor_control_target_steps + rotor_position_command_steps * load_disturbance_sensitivity_scale;
				}
			}


			if (full_sysid_start_index != -1 && i >= full_sysid_start_index && i > angle_cal_complete) {
				float total_acc = 0;
				float t = (i - full_sysid_start_index) * Tsample;
				float w = full_sysid_min_freq_hz * M_TWOPI;
				for (int k_step = 0; k_step < full_sysid_num_freqs; k_step++) {
					float wave_value = w * cosf(w * t); // multiply acceleration wave by omega to keep consistent velocity amplitude
					total_acc += wave_value;
					w *= full_sysid_freq_log_step;
				}
				rotor_control_target_steps = ((full_sysid_max_vel_amplitude_deg_per_s/full_sysid_num_freqs) * total_acc * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE);
			}


			/*
			 *
			 * Plant transfer function design based on two stage first order high pass IIR
			 * filter structures applied to rotor_control_target_steps.
			 *
			 * Second order system computed at all cycle times to avoid transient upon switching between
			 * operating modes with and without Rotor Plant Design enabled
			 *
			 */


			if (rotor_damping_coefficient != 0 || rotor_natural_frequency != 0){

					rotor_control_target_steps_filter_2 = c0*rotor_control_target_steps + c1*rotor_control_target_steps_prev
							+ c2*rotor_control_target_steps_prev_prev + c3*rotor_control_target_steps_filter_prev_2
							+ c4*rotor_control_target_steps_filter_prev_prev_2;

					rotor_control_target_steps_prev_prev = rotor_control_target_steps_prev;
					rotor_control_target_steps_filter_prev_prev_2 = rotor_control_target_steps_filter_prev_2;
					rotor_control_target_steps_filter_prev_2 = rotor_control_target_steps_filter_2;
			}

			if ((enable_rotor_plant_design == 2 )){
				rotor_control_target_steps_filter_2 = iir_0_r*rotor_control_target_steps + iir_1_r*rotor_control_target_steps_prev
						- iir_2_r*rotor_control_target_steps_filter_prev_2;
				rotor_control_target_steps_filter_prev_2 = rotor_control_target_steps_filter_2;
			}


			/*
			 * Record current value of rotor_position_command tracking signal
			 * and control signal, rotor_control_target_steps for rotor position
			 * rotor position filters, rotor plant design, performance monitoring and adaptive control
			 */

			rotor_control_target_steps_prev = rotor_control_target_steps;
			rotor_position_command_steps_prev = rotor_position_command_steps;


			if (ACCEL_CONTROL == 1) {
				if (enable_rotor_plant_design != 0){
					rotor_control_target_steps_filter_2 = rotor_plant_gain*rotor_control_target_steps_filter_2;
					apply_acceleration(&rotor_control_target_steps_filter_2, &target_velocity_prescaled, Tsample);
				/* Applies if Rotor Gain defined */
				} else if (enable_rotor_plant_gain_design == 1){
					rotor_control_target_steps_gain = rotor_plant_gain * rotor_control_target_steps;
					apply_acceleration(&rotor_control_target_steps_gain, &target_velocity_prescaled, Tsample);
				/* Applies if no Rotor Design is selected */
				} else {
					apply_acceleration(&rotor_control_target_steps, &target_velocity_prescaled, Tsample);
				}
			} else {
				BSP_MotorControl_GoTo(0, rotor_control_target_steps/2);
			}

			/*
			 * *************************************************************************************************
			 *
			 * Data Report Sequence Start
			 *
			 * *************************************************************************************************
			 */


			if (enable_pendulum_position_impulse_response_cycle == 1) {
				reference_tracking_command = pendulum_position_command_steps;
			} else {
				reference_tracking_command = rotor_position_command_steps;
			}

			/* Compute 100 cycle time average of cycle period for system performance measurement */
			if(i == 1){
				cycle_period_start = HAL_GetTick();
				cycle_period_sum = 100*Tsample*1000 - 1;
			}
			if(i % 100 == 0){
				cycle_period_sum = HAL_GetTick() - cycle_period_start;
				cycle_period_start = HAL_GetTick();
			}
			tick = HAL_GetTick();
			tick_cycle_previous = tick_cycle_current;
			tick_cycle_current = tick;



			/* High speed sampling mode data reporting */
			/* Time reported is equal to ((cycle time - 2000)microseconds/10) */

			if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 1 && enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0){
				sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", cycle_period_sum - 200,
						(int)(roundf(encoder_position)), display_parameter,
						(int)(roundf(rotor_control_target_steps)),(int)(reference_tracking_command));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

			/* High speed sampling mode data reporting without rotor chirp signal and with comb signal */
			if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && enable_rotor_tracking_comb_signal == 1 && ACCEL_CONTROL_DATA == 0){
				sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", current_cpu_cycle_delay_relative_report,
						(int)(roundf(encoder_position)), display_parameter, (int)(roundf(rotor_control_target_steps)), (int)(roundf(100*rotor_position_command_steps)));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

			/* High speed sampling mode data reporting without rotor chirp signal and without comb signal */
			/* Time reported is equal to ((cycle time - sample time)microseconds/10) */
			if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0){

				sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", cycle_period_sum - 200,
						(int)(roundf(encoder_position)), display_parameter,
						(int)(roundf(rotor_control_target_steps)),(int)(reference_tracking_command));
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

			if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && ACCEL_CONTROL_DATA == 1){
				if (enable_pendulum_position_impulse_response_cycle == 1) {
					reference_tracking_command = pendulum_position_command_steps;
				} else {
					reference_tracking_command = rotor_position_command_steps;
				}

				if (Tsample <= 0.00125) { // 1/800Hz = 0.00125s
					/* High speed sampling mode data reporting for 800 Hz mode */
					sprintf(msg, "%i\t%lu\r\n", (int)reference_tracking_command, current_pwm_period);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				} else {
					/* High speed sampling mode data reporting for 500 Hz mode with acceleration contol system data */
					sprintf(msg, "%i\t%i\t%i\t%lu\t%lu\t%lu\r\n",
							(int)reference_tracking_command, (int)(roundf(rotor_control_target_steps/10)),(int)(rotor_position_command_steps),
							current_pwm_period, desired_pwm_period/10000,
							(clock_int_time/100000));
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}
			}

			/* Select display parameter corresponding to requested selection of Sensitivity Functions */
			if (enable_disturbance_rejection_step == 1) { display_parameter = rotor_position_steps/load_disturbance_sensitivity_scale; }
			else if (enable_noise_rejection_step == 1) { noise_rej_signal = rotor_control_target_steps; }
			else if (enable_sensitivity_fnc_step == 1)  { display_parameter = rotor_position_command_steps - rotor_position_steps; }
			else { display_parameter = rotor_position_steps; }


			if (enable_noise_rejection_step == 1){
				display_parameter = noise_rej_signal;
			}

			/*
			 * Normal mode data reporting provides data output each 10th cycle
			 * Time reported is the average of ((cycle periods - desired sample period) microseconds)/10
			 */

			if (enable_high_speed_sampling == 0){

				/*
				 * Provide report each 10th control cycle
				 * Control parameters scaled by 100 to reduce communication bandwidth
				 *
				 * Speed scale may be enabled to reduce communication bandwidth
				 *
				 */


				if (report_mode != 1000 && report_mode != 2000 && speed_governor == 0){
					sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\r\n", (int)2, cycle_period_sum - 200,
							current_cpu_cycle_delay_relative_report,
							(int)(roundf(encoder_position)), display_parameter, (int)(PID_Pend.int_term)/100,
							reference_tracking_command, (int)(roundf(rotor_control_target_steps)),
							(int)(PID_Rotor.control_output)/100);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				if (report_mode != 1000 && report_mode != 2000 && (i % speed_scale) == 0 && speed_governor == 1){
					sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\r\n", (int)2, cycle_period_sum - 200,
							current_cpu_cycle_delay_relative_report,
							(int)(roundf(encoder_position)), display_parameter, (int)(PID_Pend.int_term)/100,
							reference_tracking_command, (int)(roundf(rotor_control_target_steps)),
							(int)(PID_Rotor.control_output)/100);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}

				/* Provide reports each 1000th cycle of system parameters
				 * Speed parameters scaled by 10 to reduce communication bandwidth
				 */

				if (report_mode == 1000){
					sprintf(msg, "%i\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%i\t%i\r\n", (int)0,
							PID_Pend.Kp, PID_Pend.Ki, PID_Pend.Kd,
							PID_Rotor.Kp, PID_Rotor.Ki, PID_Rotor.Kd,
							max_speed/10, min_speed/10);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}
				if (report_mode == 2000){
					sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\r\n", (int)1,
							(int)torq_current_val, max_accel, max_decel, enable_disturbance_rejection_step,
							enable_noise_rejection_step, enable_rotor_position_step_response_cycle,
							(int)(adjust_increment*10), enable_sensitivity_fnc_step);
					report_mode = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}
				report_mode = report_mode + 1;

			}

			/*
			 * Adjust cycle delay to match t_sample_cpu_cycles.
			 */

			prev_target_cpu_cycle = target_cpu_cycle;
			target_cpu_cycle += t_sample_cpu_cycles; // Increment target_cpu_cycle by sample time

			current_cpu_cycle = DWT->CYCCNT;

			// If there is time left until target_cpu_cycle
			if (((int) (target_cpu_cycle - current_cpu_cycle)) > 0) {

				// If DWT->CYCCNT needs to overflow first
				if (current_cpu_cycle > target_cpu_cycle) {
					// Wait for DWT->CYCCNT to overflow
					do {
						last_cpu_cycle = current_cpu_cycle;
						current_cpu_cycle = DWT->CYCCNT;
					} while (current_cpu_cycle >= last_cpu_cycle);
				}

				DWT_Delay_until_cycle(target_cpu_cycle);
			} else {
				// Provide warning and exit if delay exceeds 5 cycles
				if (current_cpu_cycle - target_cpu_cycle > t_sample_cpu_cycles*5 && enable_cycle_delay_warning == 1) {
					sprintf(msg, "Error: control loop lag\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					break;
				}
			}

			/* Record current cpu cycle for delay computation at the end of loop execution */
			current_cpu_cycle = DWT->CYCCNT;

			/* Compute value of relative delay after insertion of delay adjust */
			current_cpu_cycle_delay_relative_report = (int)(t_sample_cpu_cycles - (current_cpu_cycle - prev_cpu_cycle));
			current_cpu_cycle_delay_relative_report = (current_cpu_cycle_delay_relative_report*1000000)/RCC_HCLK_FREQ;

			prev_cpu_cycle = current_cpu_cycle;

			/* Increment cycle counter */

			i++;


		}

		/*
		 * *************************************************************************************************
		 *
		 * Control Loop Exit
		 *
		 * *************************************************************************************************
		 */

		/*
		 * Control System Exit Loop
		 */
		if (ACCEL_CONTROL == 1) {
			desired_pwm_period = 0;
			current_pwm_period = 0;
		}

		/*
		 * Restore rotor position at low speed profile
		 */

		ret = rotor_position_read(&rotor_position_steps);
		BSP_MotorControl_GoTo(0, 0);
		BSP_MotorControl_SoftStop(0);

		/*
		 * Terminate motor control
		 */

		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg,"Exit Control at Rotor Angle, %.2f\r\n",
				(float) ((rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		/*
		 * System software reset
		 */

		NVIC_SystemReset();

	}
}


/**
 * @brief  extract a message from a circular buffer
 * @param  CircularBuff circular buffer with data
 * @param  InitPos first byte of the message
 * @param  LastPos last added byte position
 * @param  BufMaxLen buffer length
 * @param  Msg pointer to the protocol message
 * @retval Number of bytes read if the msg is finished
 */
uint16_t Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos,
		uint16_t BufMaxLen, T_Serial_Msg *Msg) {
	/* Number of bytes to be analyzed */
	uint16_t NumNewByte = 0;
	/* Byte to be analyzed*/
	uint8_t Data;
	/* Circular buffer index */
	uint16_t MsgIdx;
	/* Two index for ByteStuffing process  */
	uint16_t BuffIdx;

	if (LastPos >= StartPos) {
		NumNewByte = LastPos - StartPos;
	} else {
		NumNewByte = BufMaxLen + LastPos - StartPos;
	}
	BuffIdx = StartPos;

	for (MsgIdx = 0; MsgIdx < NumNewByte; MsgIdx++) {
		Data = CircularBuff[BuffIdx];
		BuffIdx++;
		if (BuffIdx >= BufMaxLen) {
			BuffIdx = 0;
		}

		/* If End of message is found, start to recompose the message */
		if (Data == SERIAL_MSG_EOF) {
			Msg->Len = MsgIdx;
			return MsgIdx + 1;
		} else {
			Msg->Data[MsgIdx] = Data;
		}
	}
	return 0;
}

/* TIM3 init function */
	void MX_TIM3_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler(0);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler(0);
	}

}

/* USART2 init function */

	void MX_USART2_UART_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
							;

	huart2.Instance = USART2;
	huart2.Init.BaudRate = SAMPLE_BAUD_RATE;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler(0);
	}

	/* USART2 RX DMA Init */
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
		Error_Handler(0);
	}
	__HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
}

/**
 * @brief  This function is the User handler for the flag interrupt
 * @param  None
 * @retval None
 */
void MyFlagInterruptHandler(void) {
	/* Get the value of the status register via the L6474 command GET_STATUS */
	uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

	/* Check HIZ flag: if set, power brigdes are disabled */
	if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ) {
		// HIZ state
		// Action to be customized
	}

	/* Check direction bit */
	if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR) {
		// Forward direction is set
		// Action to be customized
	} else {
		// Backward direction is set
		// Action to be customized
	}

	/* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
	/* This often occures when a command is sent to the L6474 */
	/* while it is in HIZ state */
	if ((statusRegister & L6474_STATUS_NOTPERF_CMD)
			== L6474_STATUS_NOTPERF_CMD) {
		// Command received by SPI can't be performed
		// Action to be customized
	}

	/* Check WRONG_CMD flag: if set, the command does not exist */
	if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD) {
		//command received by SPI does not exist
		// Action to be customized
	}

	/* Check UVLO flag: if not set, there is an undervoltage lock-out */
	if ((statusRegister & L6474_STATUS_UVLO) == 0) {
		//undervoltage lock-out
		// Action to be customized
	}

	/* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_WRN) == 0) {
		//thermal warning threshold is reached
		// Action to be customized
	}

	/* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_SD) == 0) {
		//thermal shut down threshold is reached
		// Action to be customized
	}

	/* Check OCD  flag: if not set, there is an overcurrent detection */
	if ((statusRegister & L6474_STATUS_OCD) == 0) {
		//overcurrent detection
		// Action to be customized
	}

}


/**
 * @brief  This function is executed in event of error occurrence.
 * @param  error number of the error event
 * @retval None
 */
void Error_Handler(uint16_t error) {
	/* Backup error number */
	gLastError = error;

	/* Infinite loop */
	while (1) {
	}
}


/*
 ******************************************************************************
 *
 * Edukit System Functions Definitions
 *
 ******************************************************************************
 */

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

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				enable_mod_sin_rotor_tracking = 0;
				enable_rotor_position_step_response_cycle = 0;
				enable_pendulum_position_impulse_response_cycle = 0;
				enable_rotor_chirp = 0;

				sprintf(msg, "\n\rMode %i Configured\n\r", mode_index);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg), HAL_MAX_DELAY);

				enable_angle_cal = 0;
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable...................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				enable_swing_up = ENABLE_SWING_UP;
				enable_swing_up_resp = 0;
				sprintf(msg, "\n\rSwing Up Enabled - Enter 1 to Disable.....................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_swing_up_resp);
				if (enable_swing_up_resp == 1){
					enable_swing_up = 0;
				}
				sprintf(msg, "%i", enable_swing_up_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_swing_up_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable.........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				enable_angle_cal = 0;
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable...................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}



				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable.........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				}
				if ( enable_mod_sin_rotor_tracking < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				enable_angle_cal = 0;
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable................ ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				enable_swing_up = ENABLE_SWING_UP;
				enable_swing_up_resp = 0;
				sprintf(msg, "\n\rSwing Up Enabled - Enter 1 to Disable.....................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_swing_up_resp);
				if (enable_swing_up_resp == 1){
					enable_swing_up = 0;
				}
				sprintf(msg, "%i", enable_swing_up_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_swing_up_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable.........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart**************************\n\r\n\r");
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
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				enable_angle_cal = 0;
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable...................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}


				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable.........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\rEnter 0 for Dual PID - Enter 1 for State Feedback.........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_state_feedback);
				sprintf(msg, "%i", enable_state_feedback);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_state_feedback < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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

					sprintf(msg, "\n\rEnter Pendulum Angle Gain.................................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &proportional);
					sprintf(msg, "%0.2f", proportional);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( proportional < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Pendulum Angle Derivative Gain......................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &derivative);
					sprintf(msg, "%0.2f", derivative);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( derivative < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor Angle Gain....................................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_p_gain);
					sprintf(msg, "%0.2f", rotor_p_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_p_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor Angle Derivative Gain.........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_d_gain);
					sprintf(msg, "%0.2f", rotor_d_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_d_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Integral Compensator Gain (zero to disable).........................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &integral_compensator_gain);
					sprintf(msg, "%0.2f", integral_compensator_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( integral_compensator_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Feedforward Gain (return or zero to set to unity)...................: ");
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

					sprintf(msg, "\n\rEnter Pendulum PID Proportional Gain......................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);

					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &proportional);
					sprintf(msg, "%0.2f", proportional);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( proportional < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Pendulum PID Integral Gain..........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &integral);
					sprintf(msg, "%0.2f", integral);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( integral < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Pendulum PID Differential Gain......................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &derivative);
					sprintf(msg, "%0.2f", derivative);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( derivative < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor PID Proportional Gain.........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);

					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_p_gain);
					sprintf(msg, "%0.2f", rotor_p_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_p_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor PID Integral Gain:............................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg,
							strlen(msg),
							HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_i_gain);
					sprintf(msg, "%0.2f", rotor_i_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_i_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter Rotor PID Differential Gain.........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &rotor_d_gain);
					sprintf(msg, "%0.2f", rotor_d_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( rotor_d_gain < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}


				}

				select_suspended_mode = 0;

				sprintf(msg, "\n\rEnter 0 for Inverted Mode - Enter 1 for Suspended Mode....................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_suspended_mode);
				sprintf(msg, "%i", select_suspended_mode);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( select_suspended_mode < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable...................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if ( select_suspended_mode == 0 ){

					enable_swing_up = ENABLE_SWING_UP;
					enable_swing_up_resp = 0;
					sprintf(msg, "\n\rSwing Up Enabled - Enter 1 to Disable:....................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_swing_up_resp);
					if (enable_swing_up_resp == 1){
						enable_swing_up = 0;
					}
					sprintf(msg, "%i", enable_swing_up_resp);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_swing_up_resp < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable.........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				if (enable_rotor_chirp == 0 && enable_rotor_position_step_response_cycle == 0
						&& enable_mod_sin_rotor_tracking == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Rotor Tracking Comb Signal; 0 to Disable................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_tracking_comb_signal);
					sprintf(msg, "%i", enable_rotor_tracking_comb_signal);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_tracking_comb_signal < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
					sprintf(msg, "\n\rEnter 1 to Enable Pendulum Impulse Signal; 0 to Disable...................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_pendulum_position_impulse_response_cycle);
					sprintf(msg, "%i", enable_pendulum_position_impulse_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_pendulum_position_impulse_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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

				sprintf(msg, "\n\rEnter 1 to Enable Disturbance Rejection Sensitivity; 0 to Disable.........: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_disturbance_rejection_step);
				sprintf(msg, "%i", enable_disturbance_rejection_step);
				if ( enable_disturbance_rejection_step < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
					sprintf(msg, "\n\rEnter 1 to Enable Noise Rejection Sensitivity; 0 to Disable...............: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_noise_rejection_step);
					sprintf(msg, "%i", enable_noise_rejection_step);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_noise_rejection_step < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
					sprintf(msg, "\n\rEnter 1 to Enable Sensitivity Function; 0 to Disable......................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_sensitivity_fnc_step);
					sprintf(msg, "%i", enable_sensitivity_fnc_step);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_sensitivity_fnc_step < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
							sprintf(msg, "\n\rEnter Natural Frequency (rad/sec) of Minimum 0.5 and Maximum 2 ...........: ");
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

							sprintf(msg, "\n\rEnter Rotor Damping Coefficient of Minimum 0.1 and Maximum 5..............: ");
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
							sprintf(msg, "\n\rEnter Natural Frequency (rad/sec) of Minimum 0.5 and Maximum 2............: ");
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

							sprintf(msg, "\n\rEnter Rotor Damping Coefficient of Minimum 0.5 and Maximum 5 ............ : ");
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

					sprintf(msg, "\n\rEnter 1 for Rotor Plant Design Grotor = 1/(s^2 + Wn*s) ...................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_rotor_plant_design);
					sprintf(msg, "%i", select_rotor_plant_design);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( select_rotor_plant_design < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
							sprintf(msg, "\n\rEnter Wn Frequency (rad/sec) of Minimum 0 and Maximum 10 .................: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &Wo_r);
							if ( Wo_r < 0 ){
								sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
							sprintf(msg, "\n\rEnter Wn Frequency (rad/sec) of Minimum 0 and Maximum 10 .................: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &Wo_r);
							if ( Wo_r < 0 ){
								sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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

					sprintf(msg, "\n\rEnter 1 for Rotor Plant Design Grotor = 1/(s^2 + Wn*s) ...................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &select_rotor_plant_design);
					sprintf(msg, "%i", select_rotor_plant_design);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					/* Configure trotor plant design Grotor = 1/(s^2 + Wn*s) with identifier 2 */
					if (select_rotor_plant_design == 1) {
						enable_rotor_plant_design = 2;
					}
					if ( select_rotor_plant_design < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					if (enable_rotor_plant_design == 2 || enable_rotor_plant_design == 3) {
						if (select_suspended_mode == 1){
							sprintf(msg, "\n\rEnter Wn Frequency (rad/sec) of Minimum 0 and Maximum 10 .................: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &Wo_r);
							if ( Wo_r < 0 ){
								sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
							sprintf(msg, "\n\rEnter Wn Frequency (rad/sec) of Minimum 0 and Maximum 10 .................: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &Wo_r);
							if ( Wo_r < 0 ){
								sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
					sprintf(msg, "\n\rEnter Torque Current mA (default is %i)................................: ", (int)MAX_TORQUE_CONFIG);
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

				sprintf(msg, "\n\r.....Enter negative value at any prompt to correct entry and Restart... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

				sprintf(msg, "\n\rEnter Pendulum PID Proportional Gain .................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg),
						HAL_MAX_DELAY);

				read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &proportional);
				sprintf(msg, "%0.2f", proportional);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( proportional < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				sprintf(msg, "\n\rEnter Pendulum PID Integral Gain .....................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg),
						HAL_MAX_DELAY);
				read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &integral);
				sprintf(msg, "%0.2f", integral);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( integral < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				sprintf(msg, "\n\rEnter Pendulum PID Differential Gain .................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg,
						strlen(msg),
						HAL_MAX_DELAY);
				read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &derivative);
				sprintf(msg, "%0.2f", derivative);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( derivative < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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

				sprintf(msg, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable ..............: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					enable_angle_cal = 1;
				}
				sprintf(msg, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_angle_cal_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}


				enable_swing_up = ENABLE_SWING_UP;
				enable_swing_up_resp = 0;
				sprintf(msg, "\n\rSwing Up Enabled - Enter 1 to Disable ................................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_swing_up_resp);
				if (enable_swing_up_resp == 1){
					enable_swing_up = 0;
				}
				sprintf(msg, "%i", enable_swing_up_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_swing_up_resp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}


				sprintf(msg, "\n\rEnter 1 to Enable Rotor Chirp Drive; 0 to Disable ....................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_rotor_chirp);
				sprintf(msg, "%i", enable_rotor_chirp);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_rotor_chirp < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (enable_rotor_chirp == 0){
					sprintf(msg, "\n\rEnter 1 to Enable Step Drive; 0 to Disable ...........................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &enable_rotor_position_step_response_cycle);
					sprintf(msg, "%i", enable_rotor_position_step_response_cycle);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_rotor_position_step_response_cycle < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(msg, "\n\rEnter 1 to Enable Sine Drive; 0 to Disable ...........................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_mod_sin_rotor_tracking);
					sprintf(msg, "%i", enable_mod_sin_rotor_tracking);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
					if ( enable_mod_sin_rotor_tracking < 0 ){
						sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				sprintf(msg, "\n\rEnter 1 to Enable Pendulum Impulse; 0 to Disable .........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_pendulum_position_impulse_response_cycle);
				sprintf(msg, "%i", enable_pendulum_position_impulse_response_cycle);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
				if ( enable_pendulum_position_impulse_response_cycle < 0 ){
					sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
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
		sprintf(msg, "\r\nPendulum Encoder Characterization Complete");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
		sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		HAL_Delay(3000);
		NVIC_SystemReset();
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
	sprintf(msg, "\r\nMotor Characterization Complete");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);
	sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(3000);
	NVIC_SystemReset();
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

			sprintf(msg, "\r\nEnter 1 to Enter New Motor Configuration, 0 to Continue, -1 to Exit ...: ");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
					&j);
			if (j == 1) {
				break;
			}
			if (j == -1) {
				break;
			}

		}
		if (j == -1) {
				break;
		}

	}
	L6474_CmdDisable(0);
	sprintf(msg, "\r\nMotor Characterization Terminated");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);
	sprintf(msg, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(3000);
	NVIC_SystemReset();
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/






