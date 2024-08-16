#include <bldc.h>
#include <stdlib.h>
#include <stm32g0xx_hal_tim.h>
#include <stm32g0xx_hal_tim_ex.h>
#include <sys/_stdint.h>

/*	---------------------- Variables Initialization ---------------------- */

volatile uint16_t bemf_counter;
volatile bool_t bemf_flag = false;
uint16_t align_steps_temp = 0;


/*	---------------------- Typedefs Initialization ---------------------- */

struct BldcParamsConfig_t{
	/* Will be used for EEPROM Parameters	*/

    uint16_t bemf_threshold;
    uint16_t align_steps;
    uint8_t pole_pairs;
    uint16_t motorKV;

};

/*	---------------------- Initialization Methods ---------------------- */

void bldc_init(volatile BldcHandler_t* bldc){

	/* --------------- Commutation struct init ------------- */
	bldc->commutation.vA = 0;
	bldc->commutation.vB = 0;
	bldc->commutation.vC = 0;
	bldc->commutation.prev_vA = 0;
	bldc->commutation.prev_vB = 0;
	bldc->commutation.prev_vC = 0;
	bldc->commutation.iA = 0;
	bldc->commutation.iB = 0;
	bldc->commutation.iC = 0;
	bldc->commutation.vinRef = 0;
	bldc->commutation.current_step = 0;
	bldc->commutation.next_step = 0;
	bldc->commutation.prev_step = 0;
	bldc->commutation.currentPwmDutyCycle = 0;
	bldc->commutation.rpmValue = 0.0f;
	bldc->commutation.bemf_counter = 0;
	bldc->pwmTimer = &PWM_TIM;

	/* --------------- Event struct init ------------- */
	bldc->motorStatus.bemf_flag = false;
	bldc->motorStatus.event = INIT;
	bldc->motorStatus.state = IDLE;

	/* --------------- Event struct init ------------- */

	bldc->config = bldc_initConfig();

}


BldcParamsConfig_t* bldc_initConfig(){
	BldcParamsConfig_t* pConfig = malloc(sizeof(BldcParamsConfig_t));
	if(pConfig == NULL){
		return NULL;		// Add Error handling
	}

	pConfig->align_steps 	= 30;
	pConfig->bemf_threshold = 20;
	pConfig->motorKV 		= 2450;
	pConfig->pole_pairs 	= 7;

	return pConfig;
}


/*	---------------------- Getters Methods ---------------------- */
//
uint16_t bldc_get_align_steps (volatile BldcParamsConfig_t* pDriverConfig)	{return pDriverConfig->align_steps;}
uint16_t bldc_get_motorKV 	  (volatile BldcParamsConfig_t* pDriverConfig)	{return pDriverConfig->motorKV;}


/*	---------------------- Conversions ---------------------- */

//void bldc_calculate_rpm	(volatile BldcHandler_t* pDriver, uint32_t start_time, uint32_t end_time){
//
//	uint32_t timeDelta = end_time - start_time;
//	float frequency = 1.0f / ((float)timeDelta / 1000000.0f);
//	float rpm = 60 * frequency / pConfig->pole_pairs;
//	pDriver->rpmValue = rpm;
//
//}

//float bldc_pwm_to_rpm_conversion(volatile BldcHandler_t* pDriver, volatile BldcParamsConfig_t* pDriverConfig){
//	uint16_t motorKV = bldc_get_motorKV(pDriverConfig);
//	uint8_t pwm = bldc_get_pwm(pDriver);
//	float voltsToRpm = motorKV * (VIN_REF * pwm/100);
//	return voltsToRpm;
//
//}


//float bldc_rpm_to_pwm_conversion(volatile BldcHandler_t* pDriver, volatile BldcParamsConfig_t* pDriverConfig){
//
//	// CHANGE GET RPM BECAUSE I CANT SEND START & END TIME
//
//
//}

/*	---------------------- Commutation ---------------------- */


void bldc_commutation_step(volatile BldcHandler_t* pDriver, uint8_t step){

	switch(step){

	case 0:
	    pDriver->commutation.current_step = step;
	    pDriver->commutation.next_step = 1;
	    bldc_phaseC_off();
	    bldc_phaseA_H_ON();
	    bldc_phaseB_L_ON();
	    break;


	case 1:
		pDriver->commutation.current_step = step;
		pDriver->commutation.next_step = 2;
	    bldc_phaseB_off();
	    bldc_phaseA_H_ON();
	    bldc_phaseC_L_ON();
	    break;


	case 2:
		pDriver->commutation.current_step = step;
		pDriver->commutation.next_step = 3;
	    bldc_phaseA_off();
	    bldc_phaseB_H_ON();
	    bldc_phaseC_L_ON();
	    break;


	case 3:
		pDriver->commutation.current_step = step;
		pDriver->commutation.next_step = 4;
	    bldc_phaseC_off();
	    bldc_phaseA_L_ON();
	    bldc_phaseB_H_ON();
	    break;


	case 4:
		pDriver->commutation.current_step = step;
		pDriver->commutation.next_step = 5;
	    bldc_phaseB_off();
	    bldc_phaseA_L_ON();
	    bldc_phaseC_H_ON();
	    break;


	case 5:
		pDriver->commutation.current_step = step;
		pDriver->commutation.next_step = 0;
	    bldc_phaseA_off();
	    bldc_phaseB_L_ON();
	    bldc_phaseC_H_ON();
	    break;
	}


}


void bldc_set_pwm(volatile BldcHandler_t* pDriver, uint8_t duty_cycle){

	if(duty_cycle < 0)
		duty_cycle = 0;
	else if(duty_cycle > 100)
		duty_cycle = 100;

	pDriver->commutation.currentPwmDutyCycle = duty_cycle;

	pDriver->pwmTimer->Instance->CCR1 = duty_cycle;
	pDriver->pwmTimer->Instance->CCR2 = duty_cycle;
	pDriver->pwmTimer->Instance->CCR3 = duty_cycle;


}


void bldc_align_motor(volatile BldcHandler_t* pDriver){

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
	pDriver->commutation.current_step = 0;
}


void bldc_ramp(volatile BldcHandler_t* pDriver){

	/*	REMOVE NUMBERS FROM FUNCTION, USE DEFINES */
	bldc_set_pwm(pDriver, RAMP_STARTING_PWM);
	uint16_t i = 5000;
	pDriver->motorStatus.state = RAMP;
    uint8_t steps_counter = 0;
    while((i > 100) && (pDriver->motorStatus.event != BEMF_SENSING)) // && motor_state != AUTO_COMMUTATION){
    {

    	if(i <= 2000)
    		pDriver->motorStatus.event = BEMF_COUNTING;

    	if(pDriver->commutation.bemf_counter >= BEMF_DETECTED_THRESHOLD){
    		pDriver->motorStatus.event = BEMF_SENSING;
    		pDriver->motorStatus.state = AUTO_COMMUTATION;
    	}
    	delayMicro(i);
        bldc_trapezoidal_commute(pDriver);
        i-=20;
        ++steps_counter;
        if(steps_counter == 25){
        	bldc_set_pwm(pDriver, pDriver->commutation.currentPwmDutyCycle + 2);
        	steps_counter = 0;
        }
    }

}

void bldc_bemf_sensing(volatile BldcHandler_t* pDriver){
/*
    STEP0: A-H      , B-L		, C-FLOAT
    STEP1: A-H      , B-FLOAT	, C-L
    STEP2: A-FLOAT  , B-H		, C-L
    STEP3: A-H      , B-L		, C-FLOAT
    STEP4: A-L      , B-FLOAT	, C-H
    STEP5: A-FLOAT  , B-L		, C-H

*/

    switch(pDriver->commutation.current_step){
        case 0:
        case 3:
            if(	(pDriver->commutation.prev_vC < pDriver->commutation.vinRef && pDriver->commutation.vC > pDriver->commutation.vinRef + BEMF_VALUE_THRESHOLD) ||
            	(pDriver->commutation.prev_vC > pDriver->commutation.vinRef && pDriver->commutation.vC < pDriver->commutation.vinRef - BEMF_VALUE_THRESHOLD))
            {
            	pDriver->motorStatus.bemf_flag = true;
            	return;
            }

            break;

        case 2:
        case 5:

            if(	(pDriver->commutation.prev_vA < pDriver->commutation.vinRef && pDriver->commutation.vA > pDriver->commutation.vinRef + BEMF_VALUE_THRESHOLD) ||
            	(pDriver->commutation.prev_vA > pDriver->commutation.vinRef && pDriver->commutation.vA < pDriver->commutation.vinRef - BEMF_VALUE_THRESHOLD))
            {
            	pDriver->motorStatus.bemf_flag = true;
            	return;
            }

            break;

        case 1:
        case 4:

            if(	(pDriver->commutation.prev_vB < pDriver->commutation.vinRef && pDriver->commutation.vB > pDriver->commutation.vinRef + BEMF_VALUE_THRESHOLD) ||
                (pDriver->commutation.prev_vB > pDriver->commutation.vinRef && pDriver->commutation.vB < pDriver->commutation.vinRef - BEMF_VALUE_THRESHOLD))
            {
            	pDriver->motorStatus.bemf_flag = true;
            	return;
            }

            break;

        default:
            break;
    }

    pDriver->motorStatus.bemf_flag = false;
}


void bldc_trapezoidal_commute (volatile BldcHandler_t* pDriver){

	static uint32_t start_time = 0;
	static uint32_t end_time = 0;

    switch(pDriver->commutation.current_step){
        case 0:
        	if(pDriver->motorStatus.state == AUTO_COMMUTATION)
        	{

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

        	if(pDriver->motorStatus.state == AUTO_COMMUTATION)
        	{
        		end_time = __HAL_TIM_GET_COUNTER(&htim17);
        	//	bldc_calculate_rpm(pDriver, start_time, end_time);
        	}

        	bldc_commutation_step(pDriver, 0);
            break;
        default:
            break;

    }
}


//float bldc_get_rpm(volatile BldcHandler_t* pDriver, uint32_t start_time, uint32_t end_time){
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
