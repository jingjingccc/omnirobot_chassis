/*
 * control.h
 *
 *  Created on: Apr 27, 2023
 *      Author: Jing Chen
 */
/*
 * [IMPORTANT：what param in this header file you can change]
 *
 * 1. motor driver pin (INA, INB, PWM for VNH5019)(PHASE, PWM for DRV8874)
 *
 * 2. encoder and pwm timer channel
 * (timer9 is toxic)
 *
 * 3. PID gain (KP, KI, KD)
 * */
#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"

// [!!! IMPORTANT !!!] choose the board used
//#define VNH5019
#define DRV8874

/*motor driver pin*/
#ifdef VNH5019
#define M1_INA_Pin GPIOB
#define M1_INA_GPIO_port GPIO_PIN_1
#define M1_INB_Pin GPIOB
#define M1_INB_GPIO_port GPIO_PIN_2

#define M2_INA_Pin GPIOC
#define M2_INA_GPIO_port GPIO_PIN_2
#define M2_INB_Pin GPIOC
#define M2_INB_GPIO_port GPIO_PIN_3

#define M3_INA_Pin GPIOA
#define M3_INA_GPIO_port GPIO_PIN_10
#define M3_INB_Pin GPIOA
#define M3_INB_GPIO_port GPIO_PIN_11
#endif

#ifdef DRV8874
#define M1_PHASE_Pin GPIOB
#define M1_PHASE_GPIO_port GPIO_PIN_1

#define M2_PHASE_Pin GPIOC
#define M2_PHASE_GPIO_port GPIO_PIN_2

#define M3_PHASE_Pin GPIOA
#define M3_PHASE_GPIO_port GPIO_PIN_10
#endif

/*encoder and pwm timer/channel define*/
#define M1_Encoder_timer htim2
#define M1_Encoder_timerchannel TIM_CHANNEL_1 | TIM_CHANNEL_2
#define M1_Encoder_dir 1
#define M1_PWM_timer htim12
#define M1_PWM_timerchannel TIM_CHANNEL_1


#define M2_Encoder_timer htim5
#define M2_Encoder_timerchannel TIM_CHANNEL_1 | TIM_CHANNEL_2
#define M2_Encoder_dir 1
#define M2_PWM_timer htim12
#define M2_PWM_timerchannel TIM_CHANNEL_2


#define M3_Encoder_timer htim4
#define M3_Encoder_timerchannel TIM_CHANNEL_1 | TIM_CHANNEL_2
#define M3_Encoder_dir 1
#define M3_PWM_timer htim8
#define M3_PWM_timerchannel TIM_CHANNEL_4


#define Encoder_Interrupt_timer htim3
//#define control_period 0.01 // frequency = 84M/(13+1)/(60000) = 100HZ
extern double control_period;

/* Hardware Info */
#define encoder_resolution 52
#define speed_reduction_ratio 75.4

#define wheel_radius 58 / 2 / 1000
#define chassis_radius 60 / 2 / 1000

extern double radius_error_a;
extern double radius_error_b;
extern double radius_error_c;
extern double radius_error_chassis;

extern double linearvelocity_x;
extern double linearvelocity_y;
extern double angularvelocity;

/* PID gain param : only PI control*/
#define M1_KP 1.96301378719129
#define M1_KI 85.4955423434734
#define M1_KD 0.0

#define M2_KP 1.5090598598976
#define M2_KI 66.6951211309224
#define M2_KD 0.0

#define M3_KP 5.63320164765365
#define M3_KI 204.791478607115
#define M3_KD 0.0

/* PID control param define */
extern double limit_integral;
extern double pwm_arr;

typedef struct
{
		double Kp, Ki, Kd;

		int16_t CountNum;
		double rps, rps_before;
		double goal, err;
		double propotional, integral, differential;
		double duty;

#ifdef VNH5019
		GPIO_TypeDef *INA_pin_type;
		uint16_t INA_pin_Num;
		GPIO_TypeDef *INB_pin_type;
		uint16_t INB_pin_Num;
#endif
#ifdef DRV8874
		GPIO_TypeDef *PHASE_pin_type;
		uint16_t PHASE_pin_Num;
#endif

		TIM_HandleTypeDef encoder_timer;
		uint32_t encoder_timer_channel;
		int encoder_dir;
		TIM_HandleTypeDef pwm_timer;
		uint32_t pwm_timer_channel;

}PID_Controll;

extern PID_Controll WheelA;
extern PID_Controll WheelB;
extern PID_Controll WheelC;

extern int i;
extern double sssss[1000];

void Controll_Init();
void Pid_Param_Init(PID_Controll *Wheel_, double kp, double ki, double kd);
void Hardware_Info_Init();
void Controll_Timer_Init();
void PID_Controller(PID_Controll *Wheel_);
void Forward_Kinematics(int x, int y, int w);
void Stop_Chasis();

#ifdef VNH5019
void Motor_Driver_Init(PID_Controll *Wheel_,
		GPIO_TypeDef *INA_pin_type_, uint16_t INA_pin_num_,
		GPIO_TypeDef *INB_pin_type_, uint16_t INB_pin_num_,
		TIM_HandleTypeDef encoder_timer_, uint32_t encoder_timer_channel_, int encoder_dir_,
		TIM_HandleTypeDef pwm_timer_, uint32_t pwm_timer_channel_);
#endif
#ifdef DRV8874
void Motor_Driver_Init(PID_Controll *Wheel_,
		GPIO_TypeDef *PHASE_pin_type_, uint16_t PHASE_pin_num_,
		TIM_HandleTypeDef encoder_timer_, uint32_t encoder_timer_channel_, int encoder_dir_,
		TIM_HandleTypeDef pwm_timer_, uint32_t pwm_timer_channel_);
#endif



#endif /* INC_CONTROL_H_ */
