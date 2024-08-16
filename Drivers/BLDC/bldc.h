#ifndef __BLDC_H
#define __BLDC_H

#include "tim.h"
#include <stdint.h>

/*	---------------------- Reference Parameters ---------------------- */

#define VIN_REF						(float)(16.8)


/*	---------------------- PWM Timer Defines ---------------------- */

#define PWM_TIM         			htim1
#define PHA_CHANNEL     			TIM_CHANNEL_1
#define PHB_CHANNEL     			TIM_CHANNEL_2
#define PHC_CHANNEL     			TIM_CHANNEL_3


/*	---------------------- Motor Align & Ramp Parameters ---------------------- */

#define DELAY_TIMER     			htim17		// Used for Creating a rampup delay - Remove
#define ALIGN_STEPS					(uint8_t)30
#define ALIGN_MAX_PWM				(uint8_t)50	// Percentage
#define ALIGN_START_PWM				(uint8_t)5	// Percentage
#define ALIGN_PWM_INCREMENT			(uint8_t)((ALIGN_MAX_PWM - ALIGN_START_PWM) / ALIGN_STEPS)
#define RAMP_STARTING_PWM			(uint8_t)10

/*	---------------------- BEMF Parameters ---------------------- */

#define BEMF_VALUE_THRESHOLD  		(uint16_t)5
#define BEMF_DETECTED_THRESHOLD		(uint8_t)30




/*	---------------------- Enums Declarations ---------------------- */

typedef enum {
    IDLE,
	ALIGN,
    RAMP,
	AUTO_COMMUTATION
} BldcState_e;

typedef enum{
	INIT,
	BEMF_COUNTING,
	BEMF_SENSING,
	OVER_CURRENT
} BldcEvent_e;


/*	---------------------- Typedefs Declarations ---------------------- */


typedef uint8_t bool_t;
#define false 0
#define true 1

typedef struct BldcParamsConfig_t BldcParamsConfig_t;

typedef struct BldcCommutation_t{
	uint16_t 	vA;
	uint16_t 	vB;
	uint16_t 	vC;

	uint16_t 	prev_vA;
	uint16_t 	prev_vB;
	uint16_t	prev_vC;

	uint16_t	iA;
	uint16_t 	iB;
	uint16_t 	iC;

	uint16_t 	vinRef;

	uint8_t 	current_step;
	uint8_t 	next_step;
	uint8_t 	prev_step;

	uint16_t 	bemf_counter;
    uint16_t	currentPwmDutyCycle;

    float 		rpmValue;

}BldcCommutation_t;


typedef struct BldcEvents_t{

	BldcState_e state;
	BldcEvent_e event;
	bool_t 	bemf_flag;

}BldcEvents_t;


typedef struct BldcHandler_t{

	BldcCommutation_t 	commutation;
	BldcEvents_t		motorStatus;
	BldcParamsConfig_t*	config;			// private members
    TIM_HandleTypeDef* 	pwmTimer;

}BldcHandler_t;




/*	---------------------- Variables Declaration ---------------------- */

extern volatile BldcHandler_t bldc;

//extern volatile BldcHandler_t* pDriver;
//extern volatile Bldc_Handler_t* pHandler;
//extern volatile BldcParamsConfig_t* pConfig;
extern volatile bool_t bemf_flag;
extern volatile uint16_t bemf_counter;
extern uint16_t align_steps_temp;

/*	---------------------- Initialization Methods ---------------------- */

void bldc_init(volatile BldcHandler_t* bldc);
BldcParamsConfig_t* bldc_initConfig();


/*	---------------------- Commutation Stages Methods ---------------------- */



void bldc_ramp					(volatile BldcHandler_t* pDriver);
void bldc_trapezoidal_commute	(volatile BldcHandler_t* pDriver);
void bldc_commutation_step		(volatile BldcHandler_t* pDriver, uint8_t step);
void bldc_align_motor			(volatile BldcHandler_t* pDriver);
void bldc_bemf_sensing			(volatile BldcHandler_t* pDriver);


/*	---------------------- Phases ON/OFF Methods ---------------------- */

void bldc_phaseA_H_ON();
void bldc_phaseA_L_ON();
void bldc_phaseA_off();

void bldc_phaseB_H_ON();
void bldc_phaseB_L_ON();
void bldc_phaseB_off();

void bldc_phaseC_H_ON();
void bldc_phaseC_L_ON();
void bldc_phaseC_off();

void bldc_all_phases_off();

/* ------------------- Setters -----------------*/

void bldc_set_pwm(volatile BldcHandler_t* pDriver, uint8_t duty_cycle);
void bldc_calculate_rpm	(volatile BldcHandler_t* pDriver, uint32_t start_time, uint32_t end_time);
void bldc_increment_bemf_counter(volatile BldcHandler_t* pDriver);



/* ------------------- Getters -----------------*/

uint16_t bldc_get_align_steps (volatile BldcParamsConfig_t* pDriverConfig);

uint16_t bldc_get_motorKV (volatile BldcParamsConfig_t* pDriverConfig);

/* ------------------- Conversions -----------------*/

float bldc_rpm_to_pwm_conversion(volatile BldcHandler_t* pDriver, volatile BldcParamsConfig_t* pDriverConfig);
float bldc_pwm_to_rpm_conversion(volatile BldcHandler_t* pDriver, volatile BldcParamsConfig_t* pDriverConfig);


void delayMicro(uint16_t delay);

#endif	// __BLDC_H
