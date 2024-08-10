#include "bldc.h"
#include "tim.h"
#include <stdlib.h>

/*	---------------------- Variables Initialization ---------------------- */

volatile uint16_t bemf_counter;
volatile bool_t bemf_flag = false;

BldcState_e motor_state = IDLE;
BldcEvent_e motor_event = INIT;
uint16_t align_steps_temp = 0;


/*	---------------------- Typedefs Initialization ---------------------- */

struct Bldc_t{
    uint16_t vA;
    uint16_t vB;
    uint16_t vC;

    uint16_t prev_vA;
    uint16_t prev_vB;
    uint16_t prev_vC;

    uint16_t iA;
    uint16_t iB;
    uint16_t iC;

    uint16_t vinRef;

    uint8_t current_step;
    uint8_t next_step;
    uint8_t prev_step;

    TIM_HandleTypeDef* pwmTimer;
    uint16_t currentPwmDutyCycle;
    float rpmValue;

};

struct BldcParamsConfig_t{
	/* Will be used for EEPROM Parameters	*/

    uint16_t bemf_threshold;
    uint16_t align_steps;
    uint8_t pole_pairs;

};

struct Bldc_Handler_t{

	BldcState_e state;
	BldcEvent_e event;
	bool_t bemf_flag;

};

/*	---------------------- Initialization Methods ---------------------- */

Bldc_t* bldc_init(){

	Bldc_t* pBldc = malloc(sizeof(Bldc_t));
	pBldc->vA = 0;
	pBldc->vB = 0;
	pBldc->vC = 0;
	pBldc->prev_vA = 0;
	pBldc->prev_vB = 0;
	pBldc->prev_vC = 0;
	pBldc->iA = 0;
	pBldc->iB = 0;
	pBldc->iC = 0;
	pBldc->vinRef = 0;
	pBldc->current_step = 0;
	pBldc->next_step = 0;
	pBldc->prev_step = 0;
	pBldc->currentPwmDutyCycle = 0;
	pBldc->rpmValue = 0.0f;
	pBldc->pwmTimer = &PWM_TIM;
	return pBldc;
}

Bldc_Handler_t* bldc_init_handler(){

	Bldc_Handler_t* pHandler = malloc(sizeof(pHandler));
	pHandler->event = INIT;
	pHandler->state = IDLE;
	pHandler->bemf_flag = false;
	return pHandler;

}

BldcParamsConfig_t* bldc_init_config(){

	BldcParamsConfig_t* pParams = malloc(sizeof(pParams));
	pParams->align_steps = 30;
	pParams->bemf_threshold = 20;
	pParams->pole_pairs = 7;
	return pParams;

}

/*	---------------------- Setters Methods ---------------------- */

void bldc_set_vA		(volatile Bldc_t* pDriver, uint16_t value){pDriver->vA = value;}
void bldc_set_vB		(volatile Bldc_t* pDriver, uint16_t value){pDriver->vB = value;}
void bldc_set_vC		(volatile Bldc_t* pDriver, uint16_t value){pDriver->vC = value;}

void bldc_set_prev_vA	(volatile Bldc_t* pDriver, uint16_t value){pDriver->prev_vA = value;}
void bldc_set_prev_vB	(volatile Bldc_t* pDriver, uint16_t value){pDriver->prev_vB = value;}
void bldc_set_prev_vC	(volatile Bldc_t* pDriver, uint16_t value){pDriver->prev_vC = value;}

void bldc_set_iA		(volatile Bldc_t* pDriver, uint16_t value){pDriver->iA = value;}
void bldc_set_iB		(volatile Bldc_t* pDriver, uint16_t value){pDriver->iB = value;}
void bldc_set_iC		(volatile Bldc_t* pDriver, uint16_t value){pDriver->iC = value;}

void bldc_set_vinRef	(volatile Bldc_t* pDriver, uint16_t value){pDriver->vinRef = value;}

void bldc_set_commute_step		(volatile Bldc_t *pDriver, uint8_t step){pDriver->current_step = step;}
void bldc_set_next_commute_step	(volatile Bldc_t *pDriver, uint8_t step){pDriver->next_step = step;}

void bldc_set_state  (volatile Bldc_Handler_t* pHandler, BldcState_e state){pHandler->state = state;}
void bldc_set_event  (volatile Bldc_Handler_t* pHandler, BldcEvent_e event){pHandler->event = event;}

/*	---------------------- Getters Methods ---------------------- */

uint16_t bldc_get_vA		(volatile Bldc_t* pDriver){return pDriver->vA;}
uint16_t bldc_get_vB		(volatile Bldc_t* pDriver){return pDriver->vB;}
uint16_t bldc_get_vC		(volatile Bldc_t* pDriver){return pDriver->vC;}

uint16_t bldc_get_prev_vA	(volatile Bldc_t* pDriver){return pDriver->prev_vA;}
uint16_t bldc_get_prev_vB	(volatile Bldc_t* pDriver){return pDriver->prev_vB;}
uint16_t bldc_get_prev_vC	(volatile Bldc_t* pDriver){return pDriver->prev_vC;}


uint16_t bldc_get_iA		(volatile Bldc_t* pDriver){return pDriver->iA;}
uint16_t bldc_get_iB		(volatile Bldc_t* pDriver){return pDriver->iB;}
uint16_t bldc_get_iC		(volatile Bldc_t* pDriver){return pDriver->iC;}

uint16_t bldc_get_vinRef	(volatile Bldc_t* pDriver)	{return pDriver->vinRef;}

uint8_t bldc_get_pwm		(volatile Bldc_t* pDriver)	{return pDriver->currentPwmDutyCycle;}

BldcState_e bldc_get_state  (volatile Bldc_Handler_t* pHandler){return pHandler->state;}
BldcEvent_e bldc_get_event  (volatile Bldc_Handler_t* pHandler){return pHandler->event;}

uint16_t bldc_get_align_steps (volatile BldcParamsConfig_t* pDriverConfig){return pDriverConfig->align_steps;}


void bldc_commutation_step(volatile Bldc_t* pDriver, uint8_t step){

	switch(step){

	case 0:
	    bldc_set_commute_step(pDriver, step);
	    bldc_set_next_commute_step(pDriver, 1);
	    bldc_phaseC_off();
	    bldc_phaseA_H_ON();
	    bldc_phaseB_L_ON();
	    break;


	case 1:
	    bldc_set_commute_step(pDriver, step);
	    bldc_set_next_commute_step(pDriver, 2);
	    bldc_phaseB_off();
	    bldc_phaseA_H_ON();
	    bldc_phaseC_L_ON();
	    break;


	case 2:
	    bldc_set_commute_step(pDriver, step);
	    bldc_set_next_commute_step(pDriver, 3);
	    bldc_phaseA_off();
	    bldc_phaseB_H_ON();
	    bldc_phaseC_L_ON();
	    break;


	case 3:
		bldc_set_commute_step(pDriver, step);
		bldc_set_next_commute_step(pDriver, 4);
	    bldc_phaseC_off();
	    bldc_phaseA_L_ON();
	    bldc_phaseB_H_ON();
	    break;


	case 4:
	    bldc_set_commute_step(pDriver, step);
	    bldc_set_next_commute_step(pDriver, 5);
	    bldc_phaseB_off();
	    bldc_phaseA_L_ON();
	    bldc_phaseC_H_ON();
	    break;


	case 5:
	    bldc_set_commute_step(pDriver, step);
	    bldc_set_next_commute_step(pDriver, 0);
	    bldc_phaseA_off();
	    bldc_phaseB_L_ON();
	    bldc_phaseC_H_ON();
	    break;
	}


}


void bldc_set_pwm(volatile Bldc_t* pDriver, uint8_t duty_cycle){

	if(duty_cycle < 0)
		duty_cycle = 0;
	else if(duty_cycle > 100)
		duty_cycle = 100;

	pDriver->currentPwmDutyCycle = duty_cycle;

	pDriver->pwmTimer->Instance->CCR1 = duty_cycle;
	pDriver->pwmTimer->Instance->CCR2 = duty_cycle;
	pDriver->pwmTimer->Instance->CCR3 = duty_cycle;


}


void bldc_align_motor(volatile Bldc_t* pDriver){

	bldc_all_phases_off(pDriver);
	pDriver->pwmTimer->Instance->CCR1 = ALIGN_START_PWM;
	pDriver->pwmTimer->Instance->CCR2 = 100;	// Channel 2 LOW Side
	pDriver->pwmTimer->Instance->CCR3 = 100;	// Channel 3 LOW Side
	bldc_phaseA_H_ON();
	bldc_phaseB_L_ON();
	bldc_phaseC_L_ON();
	HAL_TIM_Base_Start_IT(&htim14);

	while(align_steps_temp > 0);	// TIM4 Callback

	HAL_TIM_Base_Stop_IT(&htim14);
	//all_ch_OFF();

	bldc_set_pwm(pDriver, 10);

	bldc_set_commute_step(pDriver, 0);
}


void bldc_rampUp(volatile Bldc_t* pDriver){

	/*	REMOVE NUMBERS FROM FUNCTION, USE DEFINES */
	uint16_t i = 5000;
    motor_state = RAMP;
    uint8_t steps_counter = 0;
    while(i > 100 && motor_event != BEMF_SENSING) // && motor_state != AUTO_COMMUTATION){
    {

    	if(i <= 2000)
    		motor_event = BEMF_COUNTING;

    	if(bemf_counter >= BEMF_DETECTED_THRESHOLD){
    		motor_event = BEMF_SENSING;
    		motor_state = AUTO_COMMUTATION;
    	}
    	delayMicro(i);
        bldc_trapezoidal_commute(pDriver);
        i-=20;
        ++steps_counter;
        if(steps_counter == 25){
        	bldc_set_pwm(pDriver, pDriver->currentPwmDutyCycle + 2);
        	steps_counter = 0;
        }
    }

}


bool_t bldc_bemf_sensing(volatile Bldc_t* pDriver){
/*
    STEP0: A-H      , B-L		, C-FLOAT
    STEP1: A-H      , B-FLOAT	, C-L
    STEP2: A-FLOAT  , B-H		, C-L
    STEP3: A-H      , B-L		, C-FLOAT
    STEP4: A-L      , B-FLOAT	, C-H
    STEP5: A-FLOAT  , B-L		, C-H

*/

    switch(pDriver->current_step){
        case 0:
        case 3:
            if(pDriver->prev_vC < pDriver->vinRef && pDriver->vC > pDriver->vinRef)
            	return true;

            if(pDriver->prev_vC > pDriver->vinRef && pDriver->vC < pDriver->vinRef)
            	return true;
            
            return false;

            break;
        case 2:
        case 5:

            if(pDriver->prev_vA < pDriver->vinRef && pDriver->vA > pDriver->vinRef)
            	return true;

            if(pDriver->prev_vA > pDriver->vinRef && pDriver->vA < pDriver->vinRef)
            	return true;

            return false;

            break;

        case 1:
        case 4:

            if(pDriver->prev_vB < pDriver->vinRef && pDriver->vB > pDriver->vinRef)
            	return true;

            if(pDriver->prev_vB > pDriver->vinRef && pDriver->vB < pDriver->vinRef)
            	return true;

            return false;

            break;

        default:
            break;
    }

    return false;
}


void bldc_trapezoidal_commute(volatile Bldc_t* pDriver){

	static uint32_t start_time = 0;
	static uint32_t end_time = 0;
	float rpmValue = 0.0f;

    switch(pDriver->current_step){
        case 0:
        	if(motor_state == AUTO_COMMUTATION){

        		__HAL_TIM_SET_COUNTER(&htim17, 0);
        		start_time = __HAL_TIM_GET_COUNTER(&htim17);

        	}
            bldc_commutation_step(pDriver, 1);

            break;
        case 1:
        	bldc_commutation_step(pDriver, 2);
            break;
        case 2:
        	bldc_commutation_step(pDriver, 3);
            break;
        case 3:
        	bldc_commutation_step(pDriver, 4);
            break;
        case 4:
        	bldc_commutation_step(pDriver, 5);
            break;
        case 5:

        	if(motor_state == AUTO_COMMUTATION){
        		end_time = __HAL_TIM_GET_COUNTER(&htim17);
        		//bldc_get_rpm(pDriver, start_time, end_time);
        	}

        	bldc_commutation_step(pDriver, 0);
            break;
        default:
            break;

    }
}


//float bldc_get_rpm(volatile Bldc_t* pDriver, uint32_t start_time, uint32_t end_time){
//	uint32_t elapsed_time = end_time - start_time;
//
//	if(elapsed_time > 0){
//
//		float elapsed_time_sec = elapsed_time / 1000000.0f;
//		float electrical_freq = 1.0f / elapsed_time_sec;
//		pDriver->rpmValue = (60.0f * electrical_freq) / driver_conf.pole_pairs;
//
//	}
//
//}

/*	---------------------- Phases ON/OFF Methods ---------------------- */

void bldc_all_phases_off(){
    bldc_phaseA_off();
    bldc_phaseB_off();
    bldc_phaseC_off();
}

void bldc_phaseA_H_ON(){
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHA_CHANNEL);
    HAL_TIM_PWM_Start(&PWM_TIM, PHA_CHANNEL);
}

void bldc_phaseA_L_ON(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHA_CHANNEL);
    HAL_TIMEx_PWMN_Start(&PWM_TIM, PHA_CHANNEL);
}

void bldc_phaseA_off(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHA_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHA_CHANNEL);
}

void bldc_phaseB_H_ON(){
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHB_CHANNEL);
    HAL_TIM_PWM_Start(&PWM_TIM, PHB_CHANNEL);
}

void bldc_phaseB_L_ON(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHB_CHANNEL);
    HAL_TIMEx_PWMN_Start(&PWM_TIM, PHB_CHANNEL);
}

void bldc_phaseB_off(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHB_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHB_CHANNEL);
}

void bldc_phaseC_H_ON(){
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHC_CHANNEL);
    HAL_TIM_PWM_Start(&PWM_TIM, PHC_CHANNEL);
}

void bldc_phaseC_L_ON(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHC_CHANNEL);
    HAL_TIMEx_PWMN_Start(&PWM_TIM, PHC_CHANNEL);
}

void bldc_phaseC_off(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHC_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHC_CHANNEL);
}


void delayMicro(uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim17, 0);
	while(__HAL_TIM_GET_COUNTER(&htim17) < delay);
}
