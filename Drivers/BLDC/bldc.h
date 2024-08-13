#ifndef __BLDC_H
#define __BLDC_H

#include "tim.h"
#include <stdint.h>

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

#define BEMF_THRESHOLD  			(uint16_t)20
#define BEMF_DETECTED_THRESHOLD		(uint8_t)30


/*	---------------------- Typedefs Declarations ---------------------- */

typedef struct Bldc_t Bldc_t;

typedef struct Bldc_Handler_t Bldc_Handler_t;

typedef struct BldcParamsConfig_t BldcParamsConfig_t;

typedef uint8_t bool_t;
#define false 0
#define true 1

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

/*	---------------------- Variables Declaration ---------------------- */

extern volatile Bldc_t* pDriver;
extern volatile Bldc_Handler_t* pHandler;
extern volatile BldcParamsConfig_t* pConfig;
extern volatile bool_t bemf_flag;
extern volatile uint16_t bemf_counter;
extern uint16_t align_steps_temp;

/*	---------------------- Initialization Methods ---------------------- */

Bldc_t* bldc_init();
Bldc_Handler_t* bldc_init_handler();
BldcParamsConfig_t* bldc_init_config();


/*	---------------------- Commutation Stages Methods ---------------------- */

void bldc_ramp					(volatile Bldc_t* pDriver, volatile BldcParamsConfig_t* pConfig , volatile Bldc_Handler_t* pHandler);
void bldc_trapezoidal_commute	(volatile Bldc_t* pDriver, volatile BldcParamsConfig_t *pConfig , volatile Bldc_Handler_t* pHandler);
void bldc_commutation_step		(volatile Bldc_t* pDriver, uint8_t step);
void bldc_align_motor			(volatile Bldc_t* pDriver);
bool_t bldc_bemf_sensing		(volatile Bldc_t* pDriver);


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

void bldc_set_vA		(volatile Bldc_t* pDriver, uint16_t value);
void bldc_set_vB		(volatile Bldc_t* pDriver, uint16_t value);
void bldc_set_vC		(volatile Bldc_t* pDriver, uint16_t value);

void bldc_set_prev_vA	(volatile Bldc_t* pDriver, uint16_t value);
void bldc_set_prev_vB	(volatile Bldc_t* pDriver, uint16_t value);
void bldc_set_prev_vC	(volatile Bldc_t* pDriver, uint16_t value);

void bldc_set_iA		(volatile Bldc_t* pDriver, uint16_t value);
void bldc_set_iB		(volatile Bldc_t* pDriver, uint16_t value);
void bldc_set_iC		(volatile Bldc_t* pDriver, uint16_t value);

void bldc_set_vinRef	(volatile Bldc_t* pDriver, uint16_t value);

void bldc_set_pwm		(volatile Bldc_t* pDriver, uint8_t duty_cycle);
void bldc_set_rpm		(volatile Bldc_t* pDriver, float rpmValue);
void bldc_set_commute_step		(volatile Bldc_t *pDriver, uint8_t step);
void bldc_set_next_commute_step	(volatile Bldc_t *pDriver, uint8_t step);

void bldc_set_state  (volatile Bldc_Handler_t* pHandler, BldcState_e state);
void bldc_set_event  (volatile Bldc_Handler_t* pHandler, BldcEvent_e event);

void bldc_calculate_rpm	(volatile Bldc_t* pDriver, volatile BldcParamsConfig_t* pConfig, uint32_t start_time, uint32_t end_time);

void bldc_increment_bemf_counter(volatile Bldc_t* pDriver);

/* ------------------- Getters -----------------*/
uint16_t bldc_get_vA		(volatile Bldc_t* pDriver);
uint16_t bldc_get_vB		(volatile Bldc_t* pDriver);
uint16_t bldc_get_vC		(volatile Bldc_t* pDriver);

uint16_t bldc_get_prev_vA	(volatile Bldc_t* pDriver);
uint16_t bldc_get_prev_vB	(volatile Bldc_t* pDriver);
uint16_t bldc_get_prev_vC	(volatile Bldc_t* pDriver);

uint16_t bldc_get_iA		(volatile Bldc_t* pDriver);
uint16_t bldc_get_iB		(volatile Bldc_t* pDriver);
uint16_t bldc_get_iC		(volatile Bldc_t* pDriver);

uint16_t bldc_get_vinRef	(volatile Bldc_t* pDriver);

uint8_t bldc_get_pwm		(volatile Bldc_t* pDriver);


BldcState_e bldc_get_state  (volatile Bldc_Handler_t* pHandler);
BldcEvent_e bldc_get_event  (volatile Bldc_Handler_t* pHandler);

uint16_t bldc_get_align_steps (volatile BldcParamsConfig_t* pDriverConfig);


void delayMicro(uint16_t delay);

#endif	// __BLDC_H
