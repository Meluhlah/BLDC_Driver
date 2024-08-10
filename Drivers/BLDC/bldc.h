#ifndef __BLDC_H
#define __BLDC_H


#include "app.h"
//#include "tim.h"
//#include "adc.h"

#define PWM_TIM         			htim1
#define PHA_CHANNEL     			TIM_CHANNEL_1
#define PHB_CHANNEL     			TIM_CHANNEL_2
#define PHC_CHANNEL     			TIM_CHANNEL_3


/* ------ MOTOR Alignment macros -------*/

#define ALIGN_STEPS					(uint8_t)30
#define ALIGN_MAX_PWM				(uint8_t)50	// Percentage
#define ALIGN_START_PWM				(uint8_t)5	// Percentage
#define ALIGN_PWM_INCREMENT			(uint8_t)((ALIGN_MAX_PWM - ALIGN_START_PWM) / ALIGN_STEPS)


#define BEMF_THRESHOLD  			(uint16_t)20
#define BEMF_DETECTED_THRESHOLD		(uint8_t)30


#define DELAY_TIMER     			htim17


typedef uint8_t bool_t;


typedef struct BLDC{
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

    uint16_t currentPwmDutyCycle;
    float rpmValue;


}BLDC;


/* Will be used for EEPROM Parameters	*/
typedef struct BLDC_CONFIG{


    uint16_t bemf_threshold;
    uint16_t align_steps;
    uint8_t pole_pairs;

}BLDC_CONFIG;



typedef enum {
    IDLE,
	ALIGN,
    RAMP,
	AUTO_COMMUTATION
} CONTROL_STATE_e;



typedef enum{
	INIT,
	BEMF_COUNTING,
	BEMF_SENSING,
	OVER_CURRENT
} MOTOR_EVENT_e;

extern volatile BLDC driver;
extern volatile BLDC_CONFIG driver_conf;

extern volatile bool_t bemf_flag;
extern TIM_HandleTypeDef* PWM_TIMER;
//extern volatile bool_t tim14_flag;
extern CONTROL_STATE_e motor_state;
extern MOTOR_EVENT_e motor_event;
extern volatile uint16_t bemf_counter;
extern uint16_t alignSteps;



bool_t bemf_sensing();

void bldc_init();

void set_phases_pwm_dc(uint8_t duty_cycle);

void align_motor();
void rampUp();

void trapezoidal_commute();
void get_rpm(uint32_t start_time, uint32_t end_time);


void commutation_step0();
void commutation_step1();
void commutation_step2();
void commutation_step3();
void commutation_step4();
void commutation_step5();


void CH_AH_ON();
void CH_AL_ON();
void CH_A_OFF();

void CH_BH_ON();
void CH_BL_ON();
void CH_B_OFF();

void CH_CH_ON();
void CH_CL_ON();
void CH_C_OFF();

void all_ch_OFF();

void delayMicro(uint16_t delay);

#endif	// __BLDC_H
