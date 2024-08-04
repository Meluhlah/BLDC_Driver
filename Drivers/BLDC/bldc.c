#include "bldc.h"


volatile bldc driver;
volatile bool_t bemf_flag = false;
CONTROL_STATE_e motor_state = IDLE;

void rampUp(){
    uint16_t i = 5000;
    motor_state = RAMPUP;
    while(i > 100){
        delayMicro(i);
        trapezoidal_commute(driver.current_step);
        i-=20;
    }
    motor_state = BEMF_SENSING;

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

    switch(driver.current_step){
        case 0:
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
            commutation_step0();
            break;
        default:
            break;

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
