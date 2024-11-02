#include <bldc.h>
#include <stdlib.h>
#include <stm32g0xx_hal_tim.h>
#include <stm32g0xx_hal_tim_ex.h>
#include <sys/_stdint.h>

/*	---------------------- Variables Initialization ---------------------- */

volatile uint16_t bemf_counter;
volatile bool_t bemf_flag = false;
uint16_t bldcAlignCounter;
uint16_t bldcRampCounter;


/*	---------------------- Typedefs Initialization ---------------------- */

struct BldcParamsConfig_t{
	/* Will be used for EEPROM Parameters	*/

    uint16_t 	bemf_threshold;
    uint16_t 	align_steps;
    uint8_t 	pole_pairs;
    uint16_t 	motorKV;

};

/*	---------------------- BldcParamsConfig_t Setters Methods ---------------------- */

void bldc_set_bemf_threshold	(volatile BldcParamsConfig_t* pDriverConfig, uint16_t bemf_threshold) { pDriverConfig->bemf_threshold 	= bemf_threshold;}
void bldc_set_align_steps 		(volatile BldcParamsConfig_t* pDriverConfig, uint16_t align_steps)	  { pDriverConfig->align_steps		= align_steps;}
void bldc_set_pole_pairs  		(volatile BldcParamsConfig_t* pDriverConfig, uint8_t  pole_pairs)	  { pDriverConfig->pole_pairs		= pole_pairs;}
void bldc_set_motorKV 	  		(volatile BldcParamsConfig_t* pDriverConfig, uint16_t motorKV)		  { pDriverConfig->motorKV			= motorKV;}

/*	---------------------- BldcParamsConfig_t Getters Methods ---------------------- */

uint16_t bldc_get_bemf_threshold	(volatile BldcParamsConfig_t* pDriverConfig)	{return pDriverConfig->bemf_threshold;}
uint16_t bldc_get_align_steps 		(volatile BldcParamsConfig_t* pDriverConfig)	{return pDriverConfig->align_steps;}
uint16_t bldc_get_pole_pairs  		(volatile BldcParamsConfig_t* pDriverConfig)	{return pDriverConfig->pole_pairs;}
uint16_t bldc_get_motorKV 	  		(volatile BldcParamsConfig_t* pDriverConfig)	{return pDriverConfig->motorKV;}


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
	bldc->commutation.pwm_dutyCycle = 0;
	bldc->commutation.rpmValue = 0.0f;
	bldc->commutation.bemf_counter = 0;
	bldc->pwmTimer = &PWM_TIM;
	bldc->commutation.align_current_step = 0;

	/* --------------- Event struct init ------------- */
	bldc->motorStatus.bemf_flag = false;
	bldc->motorStatus.event = DEFAULT;
	bldc->motorStatus.state = MOTOR_IDLE;

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

void bldc_motor_status_init	(volatile BldcHandler_t* pDriver){
	pDriver->motorStatus.state = MOTOR_IDLE;
	pDriver->motorStatus.event = DEFAULT;

}




/*	---------------------- Conversions ---------------------- */

void bldc_calculate_rpm(volatile BldcHandler_t* pDriver, uint32_t start_time, uint32_t end_time) {
    uint32_t timeDelta = end_time - start_time; // timeDelta in microseconds
    float frequency = 1000000.0f / (float)timeDelta; // frequency in Hz
    float rpm = (60.0f * frequency) / bldc_get_pole_pairs(pDriver->config); // RPM calculation
    pDriver->commutation.rpmValue = rpm;
}

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

	pDriver->commutation.pwm_dutyCycle = duty_cycle;

	pDriver->pwmTimer->Instance->CCR1 = duty_cycle;
	pDriver->pwmTimer->Instance->CCR2 = duty_cycle;
	pDriver->pwmTimer->Instance->CCR3 = duty_cycle;


}


void bldc_align_motor_start(volatile BldcHandler_t* pDriver){

	pDriver->motorStatus.state = MOTOR_ALIGN_INIT;
	bldcAlignCounter = 0;
	bldc_all_phases_off(pDriver);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim14);
}

void bldc_align_motor_step(volatile BldcHandler_t* pDriver){

	/*
	 * bldcAlignCounter: Timer14 interrupts every 1ms, so Each step executing for 1ms * bldcAlignCounter
	 *
	 */
	switch(pDriver->motorStatus.state){

	case MOTOR_ALIGN_INIT:
		pDriver->pwmTimer->Instance->CCR1 = ALIGN_PWM_START;
		pDriver->pwmTimer->Instance->CCR2 = 100;
		pDriver->pwmTimer->Instance->CCR3 = 100;

		bldc_phaseA_H_ON();
		bldc_phaseB_L_ON();
		bldc_phaseC_L_ON();
		pDriver->motorStatus.state = MOTOR_ALIGN_PHASE_ON;  // Move to the next state
		bldcAlignCounter = ALIGN_STEP_DELAY; // Set delay count
		break;

	case MOTOR_ALIGN_PHASE_ON:

		if(bldcAlignCounter > 0 && pDriver->pwmTimer->Instance->CCR1 < ALIGN_PWM_MAX)
			pDriver->pwmTimer->Instance->CCR1 += ALIGN_PWM_STEP;

		else{
			pDriver->motorStatus.state = MOTOR_ALIGN_PHASE_OFF;
			bldcAlignCounter = ALIGN_STEP_DELAY;
		}
		break;

	case MOTOR_ALIGN_PHASE_OFF:
		if(bldcAlignCounter > 0 && pDriver->pwmTimer->Instance->CCR1 > 0)
			pDriver->pwmTimer->Instance->CCR1 -= ALIGN_PWM_STEP;

		else
			pDriver->motorStatus.state = MOTOR_ALIGN_DONE;
		break;

	case MOTOR_ALIGN_DONE:
		bldc_all_phases_off();
		HAL_TIM_Base_Stop_IT(&htim14);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
		pDriver->commutation.current_step = 0;
		break;

	default:
		break;
	}


}

void bldc_ramp_start(volatile BldcHandler_t* pDriver) {
    pDriver->motorStatus.state = MOTOR_RAMP_START;
    bldcRampCounter = RAMP_STEP_DELAY;
    HAL_TIM_Base_Start_IT(&htim17);  // Timer configured for ~1 ms interrupts
}

void bldc_ramp_step(volatile BldcHandler_t* pDriver) {

	/*
	 * delayUs - Controlling how long each commutation step takes, Timer fires every 10us
	 * pwmIncreaseRatio - After how many function calls, the pwm value increases.
	 *
	 */

	static uint16_t delayUs = RAMP_STEP_DELAY;	// Timer Interrupts every 10us
	static uint16_t pwmIncreaseRatio = 0;		// after how many calls of the function the pwm will increase

    switch(pDriver->motorStatus.state) {
        case MOTOR_RAMP_START:
            pDriver->motorStatus.state = MOTOR_RAMP_IN_PROGRESS;
            bldc_set_pwm(pDriver, RAMP_PWM_START);
            break;

        case MOTOR_RAMP_IN_PROGRESS:

        	if(bldcRampCounter == 0){
        		bldc_trapezoidal_commute(pDriver);
        		delayUs -= 5;
        		bldcRampCounter = delayUs;
        		pwmIncreaseRatio++;

        	}

        	if(delayUs == 20 && pDriver->commutation.pwm_dutyCycle < RAMP_PWM_MAX){
        		bldc_set_pwm(pDriver, pDriver->commutation.pwm_dutyCycle += 1);
        		pwmIncreaseRatio = 0;
        	}

        	if(delayUs == 0)
        		pDriver->motorStatus.state = MOTOR_RAMP_DONE;
            break;

        case MOTOR_RAMP_DONE:
            //bldc_all_phases_off();
            //HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_TIM_Base_Stop_IT(&htim17);
            pDriver->motorStatus.event = BEMF_SENSING;
            delayUs = RAMP_STEP_DELAY;
            pwmIncreaseRatio = 0;
            break;

        default:
        	break;
    }
}


/*void bldc_ramp(volatile BldcHandler_t* pDriver){

		REMOVE NUMBERS FROM FUNCTION, USE DEFINES
	bldc_set_pwm(pDriver, RAMP_PWM_START);
	uint16_t i = 5000;
	pDriver->motorStatus.state = MOTOR_RAMP_START;
    uint8_t steps_counter = 0;
    while((i > 100) && (pDriver->motorStatus.event != BEMF_SENSING)) // && motor_state != MOTOR_AUTO_COMMUTATION){
    {

    	if(i <= 2000)
    		pDriver->motorStatus.event = BEMF_COUNTING;

    	if(pDriver->commutation.bemf_counter >= BEMF_DETECTED_THRESHOLD){
    		pDriver->motorStatus.event = BEMF_SENSING;
    		pDriver->motorStatus.state = MOTOR_AUTO_COMMUTATION;
    	}
    	delayMicro(i);
        bldc_trapezoidal_commute(pDriver);
        i-=20;
        ++steps_counter;
        if(steps_counter == 25){
        	bldc_set_pwm(pDriver, pDriver->commutation.pwm_dutyCycle + 2);
        	steps_counter = 0;
        }
    }

}*/

void bldc_bemf_sensing(volatile BldcHandler_t* pDriver){
/*
    STEP0: A-H      , B-L		, C-FLOAT
    STEP1: A-H      , B-FLOAT	, C-L
    STEP2: A-FLOAT  , B-H		, C-L
    STEP3: A-H      , B-L		, C-FLOAT
    STEP4: A-L      , B-FLOAT	, C-H
    STEP5: A-FLOAT  , B-L		, C-H

*/


	// ##################### CHANGE STATE EACH CASE FOR RISING OR FALLING ONLY, TO MITIGATE NOISE ###########################
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


void bldc_trapezoidal_commute(volatile BldcHandler_t* pDriver){

	//delayMicro(2);
	static uint32_t start_time = 0;
	static uint32_t end_time = 0;

    switch(pDriver->commutation.current_step){
        case 0:
        	if(pDriver->motorStatus.state == MOTOR_AUTO_COMMUTATION)
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

        	if(pDriver->motorStatus.state == MOTOR_AUTO_COMMUTATION)
        	{
        		end_time = __HAL_TIM_GET_COUNTER(&htim17);
        		bldc_calculate_rpm(pDriver, start_time, end_time);
        	}

        	bldc_commutation_step(pDriver, 0);
            break;
        default:
            break;

    }
}

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
