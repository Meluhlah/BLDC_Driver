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

#define ALIGN_STEP_DELAY			(uint16_t)30	// Multiplier for 1ms Callback Interrupt
#define ALIGN_STEPS					(uint16_t)30	// How Many Steps of increment the Align function preforms
#define ALIGN_PWM_MAX				(uint16_t)15	// Percentage
#define ALIGN_PWM_START				(uint16_t)2		// Percentage
#define ALIGN_PWM_INC				(uint16_t)1		// PWM Value Increment

#define RAMP_STEP_DELAY				(uint16_t)350
#define RAMP_PWM_START				(uint8_t)20
#define RAMP_PWM_STEP				(uint8_t)1
#define RAMP_PWM_MAX				(uint8_t)40

/*	---------------------- BEMF Parameters ---------------------- */

#define BEMF_VALUE_THRESHOLD  		(uint16_t)5
#define BEMF_DETECTED_THRESHOLD		(uint8_t)30




/*	---------------------- Enums Declarations ---------------------- */

typedef enum {
    MOTOR_IDLE,

	MOTOR_ALIGN_INIT,
	MOTOR_ALIGN_PHASE_ON,
	MOTOR_ALIGN_PHASE_OFF,
	MOTOR_ALIGN_DONE,

    MOTOR_RAMP_START,
	MOTOR_RAMP_IN_PROGRESS,
	MOTOR_RAMP_DONE,

	MOTOR_AUTO_COMMUTATION
} BldcState_e;

typedef enum{
	DEFAULT,
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
	uint16_t vA;
	uint16_t vB;
	uint16_t vC;

	uint16_t prev_vA;
	uint16_t prev_vB;
	uint16_t prev_vC;

	uint16_t iA;
	uint16_t iB;
	uint16_t iC;

	uint16_t tempA;
	uint16_t tempB;
	uint16_t tempC;

	uint16_t vinRef;

	uint8_t current_step;
	uint8_t next_step;
	uint8_t prev_step;

	uint16_t bemf_counter;
	uint16_t align_current_step;
    uint16_t pwm_dutyCycle;

    uint16_t potValue;
    float 	 rpmValue;

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
extern volatile bool_t bemf_flag;
extern volatile uint16_t bemf_counter;
extern uint16_t bldcAlignCounter;
extern uint16_t bldcRampCounter;

/*	---------------------- Initialization Methods ---------------------- */

void bldc_init(volatile BldcHandler_t* bldc);
BldcParamsConfig_t* bldc_initConfig();


/*	---------------------- Commutation Stages Methods ---------------------- */



void bldc_ramp_start			(volatile BldcHandler_t* pDriver);
void bldc_ramp_step				(volatile BldcHandler_t* pDriver);
void bldc_trapezoidal_commute	(volatile BldcHandler_t* pDriver);
void bldc_commutation_step		(volatile BldcHandler_t* pDriver, uint8_t step);
void bldc_align_motor_start		(volatile BldcHandler_t* pDriver);
void bldc_align_motor_step		(volatile BldcHandler_t* pDriver);
void bldc_bemf_sensing			(volatile BldcHandler_t* pDriver);
void bldc_motor_status_init		(volatile BldcHandler_t* pDriver);

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

/*	---------------------- BldcParamsConfig_t Setters Methods ---------------------- */

void bldc_set_bemf_threshold	(volatile BldcParamsConfig_t* pDriverConfig, uint16_t bemf_threshold);
void bldc_set_align_steps 		(volatile BldcParamsConfig_t* pDriverConfig, uint16_t align_steps);
void bldc_set_pole_pairs  		(volatile BldcParamsConfig_t* pDriverConfig, uint8_t  pole_pairs);
void bldc_set_motorKV 	  		(volatile BldcParamsConfig_t* pDriverConfig, uint16_t motorKV);

/* ----------------------------------------------------------------------------------*/



void bldc_set_pwm				(volatile BldcHandler_t* pDriver, uint8_t duty_cycle);
void bldc_calculate_rpm			(volatile BldcHandler_t* pDriver, uint32_t start_time, uint32_t end_time);


/*	---------------------- BldcParamsConfig_t Getters Methods ---------------------- */

uint16_t bldc_get_bemf_threshold	(volatile BldcParamsConfig_t* pDriverConfig);
uint16_t bldc_get_align_steps 		(volatile BldcParamsConfig_t* pDriverConfig);
uint16_t bldc_get_pole_pairs  		(volatile BldcParamsConfig_t* pDriverConfig);
uint16_t bldc_get_motorKV 	  		(volatile BldcParamsConfig_t* pDriverConfig);


/* ------------------- Conversions -----------------*/

float bldc_rpm_to_pwm_conversion(volatile BldcHandler_t* pDriver, volatile BldcParamsConfig_t* pDriverConfig);
float bldc_pwm_to_rpm_conversion(volatile BldcHandler_t* pDriver, volatile BldcParamsConfig_t* pDriverConfig);


void delayMicro(uint16_t delay);

#endif	// __BLDC_H
