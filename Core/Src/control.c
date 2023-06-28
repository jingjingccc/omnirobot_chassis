/*
 * control.c
 *
 *  Created on: Apr 27, 2023
 *      Author: Jing Chen

==============================================================================
                        ##### Brief Introduction #####
==============================================================================
	The driver is mainly used for PID control process.

	The input data is the chassis velocity 底盤速度 (linearvelocity_x, linearvelocity_y, angularvelocity),
	and convert it into wheel angular velocity 輪子角速度 through the function "Forward_Kinematics".

	The output of the function is the goal rps for the motors need to reach.

	For the function "PID_Controller", the input data is current rps, combining goal rps we calculated above, do the PID controll.
	Give the corresponding PWM to each motor.

	The frequency of PID controll is 100hz. (use timer3, with APB1 Timer clocks 84Mhz, prescaler 13 and ARR 60000)
*/

#include <control.h>
#include <math.h>

double radius_error_a;
double radius_error_b;
double radius_error_c;
double radius_error_chassis;

double linearvelocity_x;
double linearvelocity_y;
double angularvelocity;

double limit_integral;
double pwm_arr;
double control_period;

PID_Control WheelA = {0};
PID_Control WheelB = {0};
PID_Control WheelC = {0};

int i;
double sssss[1000];

/**
 * @ brief Include all the initial function
 * @ retval None
 * */
void Control_Init()
{
	Hardware_Info_Init();
	Control_Timer_Init();

#ifdef VNH5019
	Motor_Driver_Init(&WheelA, M1_INA_Pin, M1_INA_GPIO_port, M1_INB_Pin, M1_INB_GPIO_port, M1_Encoder_timer, M1_Encoder_timerchannel, M1_Encoder_dir, M1_PWM_timer, M1_PWM_timerchannel);
	Motor_Driver_Init(&WheelB, M2_INA_Pin, M2_INA_GPIO_port, M2_INB_Pin, M2_INB_GPIO_port, M2_Encoder_timer, M2_Encoder_timerchannel, M2_Encoder_dir, M2_PWM_timer, M2_PWM_timerchannel);
	Motor_Driver_Init(&WheelC, M3_INA_Pin, M3_INA_GPIO_port, M3_INB_Pin, M3_INB_GPIO_port, M3_Encoder_timer, M3_Encoder_timerchannel, M3_Encoder_dir, M3_PWM_timer, M3_PWM_timerchannel);
#endif
#ifdef DRV8874
	Motor_Driver_Init(&WheelA, M1_PHASE_Pin, M1_PHASE_GPIO_port, M1_Encoder_timer, M1_Encoder_timerchannel, M1_Encoder_dir, M1_PWM_timer, M1_PWM_timerchannel);
	Motor_Driver_Init(&WheelB, M2_PHASE_Pin, M2_PHASE_GPIO_port, M2_Encoder_timer, M2_Encoder_timerchannel, M2_Encoder_dir, M2_PWM_timer, M2_PWM_timerchannel);
	Motor_Driver_Init(&WheelC, M3_PHASE_Pin, M3_PHASE_GPIO_port, M3_Encoder_timer, M3_Encoder_timerchannel, M3_Encoder_dir, M3_PWM_timer, M3_PWM_timerchannel);
#endif

	Pid_Param_Init(&WheelA, M1_KP, M1_KI, M1_KD);
	Pid_Param_Init(&WheelB, M2_KP, M2_KI, M2_KD);
	Pid_Param_Init(&WheelC, M3_KP, M3_KI, M3_KD);

	i = 0;
	limit_integral = 0.6;
	pwm_arr = M1_PWM_timer.Init.Period;

	// PCLK1_freq, APB1 timer frequency
	int32_t PCLK1_freq = HAL_RCC_GetPCLK1Freq();
	if((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
	{
		PCLK1_freq *=2;
	}

	int32_t timer_interrupt_freq = (double)PCLK1_freq / (Encoder_Interrupt_timer.Init.Prescaler + 1) / Encoder_Interrupt_timer.Init.Period;
	control_period = (double)(1 / (double)timer_interrupt_freq);

	WheelA.integral = 0.0;
	WheelB.integral = 0.0;
	WheelC.integral = 0.0;

	WheelA.goal = 0.0;
	WheelB.goal = 0.0;
	WheelC.goal = 0.0;

	// stop chassis
	HAL_GPIO_WritePin(WheelA.PHASE_pin_type, WheelA.PHASE_pin_Num, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&(WheelA.pwm_timer), WheelA.pwm_timer_channel, 0);

	HAL_GPIO_WritePin(WheelB.PHASE_pin_type, WheelB.PHASE_pin_Num, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&(WheelB.pwm_timer), WheelB.pwm_timer_channel, 0);

	HAL_GPIO_WritePin(WheelC.PHASE_pin_type, WheelC.PHASE_pin_Num, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&(WheelC.pwm_timer), WheelC.pwm_timer_channel, 0);
}


/**
 * @ brief assign the pid gain value into the PID_Controll object
 * @ retval None
 * */
void Pid_Param_Init(PID_Control *Wheel_, double kp, double ki, double kd)
{
	Wheel_->Kp = kp;
	Wheel_->Ki = ki;
	Wheel_->Kd = kd;
}


/**
 * @ brief assign the required param of each motors into the PID_Controll object
 * @ define choose the board used : VNH5019 or DRV8874, change the define in "control.h"
 * @ param Wheel_ the object declare for each motor (WheelA, WheelB, WheelC)
 * @ param all the other param is define in "control.h"
 * @ retval None
 * */
#ifdef VNH5019
void Motor_Driver_Init(PID_Control *Wheel_,
		GPIO_TypeDef *INA_pin_type_, uint16_t INA_pin_num_,
		GPIO_TypeDef *INB_pin_type_, uint16_t INB_pin_num_,
		TIM_HandleTypeDef encoder_timer_, uint32_t encoder_timer_channel_, int encoder_dir_,
		TIM_HandleTypeDef pwm_timer_, uint32_t pwm_timer_channel_)
{
	Wheel_->INA_pin_type = INA_pin_type_;
	Wheel_->INA_pin_Num = INA_pin_num_;
	Wheel_->INB_pin_type = INB_pin_type_;
	Wheel_->INB_pin_Num = INB_pin_num_;
	Wheel_->encoder_timer = encoder_timer_;
	Wheel_->encoder_timer_channel = encoder_timer_channel_;
	Wheel_->encoder_dir = encoder_dir_;
	Wheel_->pwm_timer = pwm_timer_;
	Wheel_->pwm_timer_channel = pwm_timer_channel_;
}
#endif
#ifdef DRV8874
void Motor_Driver_Init(PID_Control *Wheel_,
		GPIO_TypeDef *PHASE_pin_type_, uint16_t PHASE_pin_num_,
		TIM_HandleTypeDef encoder_timer_, uint32_t encoder_timer_channel_, int encoder_dir_,
		TIM_HandleTypeDef pwm_timer_, uint32_t pwm_timer_channel_)
{
	Wheel_->PHASE_pin_type = PHASE_pin_type_;
	Wheel_->PHASE_pin_Num = PHASE_pin_num_;
	Wheel_->encoder_timer = encoder_timer_;
	Wheel_->encoder_timer_channel = encoder_timer_channel_;
	Wheel_->encoder_dir = encoder_dir_;
	Wheel_->pwm_timer = pwm_timer_;
	Wheel_->pwm_timer_channel = pwm_timer_channel_;
}
#endif


/**
 * @ brief initialize the timers that encoder and pwm used
 * @ all the definition in the function are in "control.h"
 * @ retval None
 * */
void Control_Timer_Init()
{
	HAL_TIM_Encoder_Start(&M1_Encoder_timer, M1_Encoder_timerchannel);
	HAL_TIM_PWM_Start(&M1_PWM_timer, M1_PWM_timerchannel);
	HAL_TIM_Encoder_Start(&M2_Encoder_timer, M2_Encoder_timerchannel);
	HAL_TIM_PWM_Start(&M2_PWM_timer, M2_PWM_timerchannel);
	HAL_TIM_Encoder_Start(&M3_Encoder_timer, M3_Encoder_timerchannel);
	HAL_TIM_PWM_Start(&M3_PWM_timer, M3_PWM_timerchannel);
}


/**
 * @ brief assign the hardware value for motor and chassis radius error
 * @ retval None
 * */
void Hardware_Info_Init()
{
	radius_error_a = 1.0;
	radius_error_b = 1.0;
	radius_error_c = 1.0;
	radius_error_chassis = 1.0;
}


/**
 * @ brief PID control for the motor
 * @ param Wheel_ the object declare for each motor (WheelA, WheelB, WheelC)
 * @ retval None
 * */
void PID_Controller(PID_Control *Wheel_)
{
	Wheel_->CountNum = __HAL_TIM_GetCounter(&Wheel_->encoder_timer)* Wheel_->encoder_dir;
	Wheel_->rps = (double)Wheel_->CountNum / ((double)4 * encoder_resolution * speed_reduction_ratio * control_period);
	__HAL_TIM_SetCounter(&Wheel_->encoder_timer ,0);

//		if (i<500)
//		{
//			sssss[i] = Wheel_->rps;
//			i++;
//		}

	Wheel_->err = Wheel_->goal - Wheel_->rps;
	Wheel_->propotional = (double)Wheel_->err * Wheel_->Kp;
	Wheel_->integral += (double)Wheel_->err * Wheel_->Ki * control_period;
	Wheel_->integral = (Wheel_->integral > limit_integral)? limit_integral : Wheel_->integral;
	Wheel_->integral = (Wheel_->integral < (double)(-1) * limit_integral)? (double)(-1) * limit_integral : Wheel_->integral;
	Wheel_->differential = (double) Wheel_->Kd * (-1) * (Wheel_->rps - Wheel_->rps_before) / control_period;


	Wheel_->duty = Wheel_->propotional + Wheel_->integral + Wheel_->differential;
	Wheel_->duty = (Wheel_->duty > 1)? 1 : Wheel_->duty;
	Wheel_->duty = (Wheel_->duty < -1)? -1 : Wheel_->duty;

//	Wheel_->duty = 1.0;

#ifdef VNH5019
	if(Wheel_->duty >= 0)
	{
		HAL_GPIO_WritePin(Wheel_->INA_pin_type, Wheel_->INA_pin_Num, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Wheel_->INB_pin_type, Wheel_->INB_pin_Num, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
	else
	{
		HAL_GPIO_WritePin(Wheel_->INA_pin_type, Wheel_->INA_pin_Num, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Wheel_->INB_pin_type, Wheel_->INB_pin_Num, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
#endif
#ifdef DRV8874
	if(Wheel_->duty >= 0)
	{
		HAL_GPIO_WritePin(Wheel_->PHASE_pin_type, Wheel_->PHASE_pin_Num, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
	else
	{
		HAL_GPIO_WritePin(Wheel_->PHASE_pin_type, Wheel_->PHASE_pin_Num, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
#endif


	Wheel_->rps_before = Wheel_->rps;
}


/**
 * @ brief turn the chassis velocity into wheel angular velocity
 * @ param x linear velocity for the chassis in x-direction
 * @ param y linear velocity for the chassis in y-direction
 * @ param w angular velocity for the chassis (counter-clockwise is positive )
 *
 *          A                   x (+)
 *         / \                  ↑
 *        /   \                 |
 *       /     \        y(+) ←---
 *      B-------C
 * @ retval None
 * */
void Forward_Kinematics(double x, double y, double w)
{
	double omega_a = (y + w * chassis_radius * radius_error_chassis)/(wheel_radius * radius_error_a);
	double omega_b = ((-sqrt(3)) * x * 0.5 - (0.5) * y + w * chassis_radius * radius_error_chassis)/(wheel_radius * radius_error_b);
	double omega_c = ((sqrt(3)) * x * 0.5 - (0.5) * y + w * chassis_radius * radius_error_chassis)/(wheel_radius * radius_error_c);

	WheelA.goal = omega_a / (2 * M_PI);
	WheelB.goal = omega_b / (2 * M_PI);
	WheelC.goal = omega_c / (2 * M_PI);
}


/**
 * @ brief set the input speed zero
 * @ retval None
 * */
void Stop_Chasis()
{
	linearvelocity_x = 0;
	linearvelocity_y = 0;
	angularvelocity = 0;
}

