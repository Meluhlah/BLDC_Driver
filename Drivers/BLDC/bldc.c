#include "bldc.h"
#include "tim.h"


volatile BLDC driver;
volatile BLDC_CONFIG driver_conf;
volatile bool_t bemf_flag = false;
CONTROL_STATE_e motor_state = IDLE;
MOTOR_EVENT_e motor_event = INIT;
TIM_HandleTypeDef* PWM_TIMER = &PWM_TIM;
volatile uint16_t bemf_counter;
uint16_t alignSteps = 0;


void set_phases_pwm_dc(uint8_t duty_cycle){

	if(duty_cycle < 0)
		duty_cycle = 0;
	else if(duty_cycle > 100)
		duty_cycle = 100;

	driver.currentPwmDutyCycle = duty_cycle;

	PWM_TIMER->Instance->CCR1 = duty_cycle;
	PWM_TIMER->Instance->CCR2 = duty_cycle;
	PWM_TIMER->Instance->CCR3 = duty_cycle;


}

void align_motor(){

	all_ch_OFF();
	PWM_TIMER->Instance->CCR1 = ALIGN_START_PWM;
	PWM_TIMER->Instance->CCR2 = 100;	// Channel 2 LOW Side
	PWM_TIMER->Instance->CCR3 = 100;	// Channel 3 LOW Side
	CH_AH_ON();
	CH_BL_ON();
	CH_CL_ON();
	HAL_TIM_Base_Start_IT(&htim14);

	while(alignSteps > 0);	// TIM4 Callback

	HAL_TIM_Base_Stop_IT(&htim14);
	//all_ch_OFF();

	set_phases_pwm_dc(10);

	driver.current_step = 0;
}

void bldc_init(){
	driver.vA = 0;
	driver.vB = 0;
	driver.vC = 0;

	driver.prev_vA = 0;
	driver.prev_vB = 0;
	driver.prev_vC = 0;

	driver.iA = 0;
	driver.iB = 0;
	driver.iC = 0;

	driver.vinRef = 0;

	driver.current_step = 0;
	driver.next_step = 0;
	driver.prev_step = 0;

	driver.currentPwmDutyCycle = 0;
	driver.rpmValue = 0.0f;


	driver_conf.bemf_threshold = BEMF_THRESHOLD;
	driver_conf.align_steps = ALIGN_STEPS;
	driver_conf.pole_pairs = 7;



	bemf_counter = 0;
	alignSteps = ALIGN_STEPS;
}


/*	REMOVE NUMBERS FROM FUNCTION, USE DEFINES */

void rampUp(){
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
        trapezoidal_commute();
        i-=20;
        ++steps_counter;
        if(steps_counter == 25){
        	set_phases_pwm_dc(driver.currentPwmDutyCycle + 2);
        	steps_counter = 0;
        }
    }

}

bool_t bemf_sensing(){
/*
    STEP0: A-H      , B-L		, C-FLOAT
    STEP1: A-H      , B-FLOAT	, C-L
    STEP2: A-FLOAT  , B-H		, C-L
    STEP3: A-H      , B-L		, C-FLOAT
    STEP4: A-L      , B-FLOAT	, C-H
    STEP5: A-FLOAT  , B-L		, C-H

*/

    switch(driver.current_step){
        case 0:
        case 3:
            if(driver.prev_vC < driver.vinRef && driver.vC > driver.vinRef)
            	return true;

            if(driver.prev_vC > driver.vinRef && driver.vC < driver.vinRef)
            	return true;
            
            return false;

            break;
        case 2:
        case 5:

            if(driver.prev_vA < driver.vinRef && driver.vA > driver.vinRef)
            	return true;

            if(driver.prev_vA > driver.vinRef && driver.vA < driver.vinRef)
            	return true;

            return false;

            break;

        case 1:
        case 4:

            if(driver.prev_vB < driver.vinRef && driver.vB > driver.vinRef)
            	return true;

            if(driver.prev_vB > driver.vinRef && driver.vB < driver.vinRef)
            	return true;

            return false;

            break;

        default:
            break;
    }

    return false;
}

void trapezoidal_commute(){

	static uint32_t start_time = 0;
	static uint32_t end_time = 0;

    switch(driver.current_step){
        case 0:
        	if(motor_state == AUTO_COMMUTATION){

        		__HAL_TIM_SET_COUNTER(&htim17, 0);
        		start_time = __HAL_TIM_GET_COUNTER(&htim17);

        	}
            commutation_step1();

            break;
        case 1:
            commutation_step2();
            break;
        case 2:
            commutation_step3();
            break;
        case 3:
            commutation_step4();
            break;
        case 4:
            commutation_step5();
            break;
        case 5:

        	if(motor_state == AUTO_COMMUTATION){
        		end_time = __HAL_TIM_GET_COUNTER(&htim17);
        		get_rpm(start_time, end_time);
        	}

            commutation_step0();
            break;
        default:
            break;

    }
}


void get_rpm(uint32_t start_time, uint32_t end_time){
	uint32_t elapsed_time = end_time - start_time;

	if(elapsed_time > 0){

		float elapsed_time_sec = elapsed_time / 1000000.0f;
		float electrical_freq = 1.0f / elapsed_time_sec;
		driver.rpmValue = (60.0f * electrical_freq) / driver_conf.pole_pairs;

	}

}

void all_ch_OFF(){
    CH_A_OFF();
    CH_B_OFF();
    CH_C_OFF();
}

void commutation_step0(){
    driver.current_step = 0;
    driver.next_step = 1;
    CH_C_OFF();
    CH_AH_ON();
    CH_BL_ON();
}

void commutation_step1(){
    driver.current_step = 1;
    driver.next_step = 2;
    CH_B_OFF();
    CH_AH_ON();
    CH_CL_ON();
}

void commutation_step2(){
    driver.current_step = 2;
    driver.next_step = 3;
    CH_A_OFF();
    CH_BH_ON();
    CH_CL_ON();
}

void commutation_step3(){
    driver.current_step = 3;
    driver.next_step = 4;
    CH_C_OFF();
    CH_AL_ON();
    CH_BH_ON();
}

void commutation_step4(){
    driver.current_step = 4;
    driver.next_step = 5;
    CH_B_OFF();
    CH_AL_ON();
    CH_CH_ON();
}

void commutation_step5(){
    driver.current_step = 5;
    driver.next_step = 0;
    CH_A_OFF();
    CH_BL_ON();
    CH_CH_ON();
}


void CH_AH_ON(){
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHA_CHANNEL);
    HAL_TIM_PWM_Start(&PWM_TIM, PHA_CHANNEL);
}

void CH_AL_ON(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHA_CHANNEL);
    HAL_TIMEx_PWMN_Start(&PWM_TIM, PHA_CHANNEL);
}

void CH_A_OFF(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHA_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHA_CHANNEL);
}

void CH_BH_ON(){
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHB_CHANNEL);
    HAL_TIM_PWM_Start(&PWM_TIM, PHB_CHANNEL);
}

void CH_BL_ON(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHB_CHANNEL);
    HAL_TIMEx_PWMN_Start(&PWM_TIM, PHB_CHANNEL);
}

void CH_B_OFF(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHB_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHB_CHANNEL);
}

void CH_CH_ON(){
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHC_CHANNEL);
    HAL_TIM_PWM_Start(&PWM_TIM, PHC_CHANNEL);
}

void CH_CL_ON(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHC_CHANNEL);
    HAL_TIMEx_PWMN_Start(&PWM_TIM, PHC_CHANNEL);
}

void CH_C_OFF(){
    HAL_TIM_PWM_Stop(&PWM_TIM, PHC_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&PWM_TIM, PHC_CHANNEL);
}


void delayMicro(uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim17, 0);
	while(__HAL_TIM_GET_COUNTER(&htim17) < delay);
}
