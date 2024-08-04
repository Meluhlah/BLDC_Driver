#ifndef __BLDC
#define __BLDC

#include "app.h"
#include "tim.h"
#include "adc.h"

#define PWM_TIM         htim1
#define PHA_CHANNEL     TIM_CHANNEL_1
#define PHB_CHANNEL     TIM_CHANNEL_2
#define PHC_CHANNEL     TIM_CHANNEL_3

#define BEMG_THRESHOLD  (float)

#define DELAY_TIMER     htim17


typedef struct bldc{
    uint16_t vA;
    uint16_t vB;
    uint16_t vC;

    uint16_t prev_vA;
    uint16_t prev_vB;
    uint16_t prev_vC;

    uint16_t i_A;
    uint16_t i_B;
    uint16_t i_C;

    uint16_t vinRef;

    uint8_t current_step;
    uint8_t next_step;
    uint8_t prev_step;

    uint16_t pwmDutyCycle;

}bldc;

extern volatile bldc driver;
extern volatile uint16_t adc_buffer[ADC_CHANNELS];
extern volatile bool_t bemf_flag;


typedef enum{
    IDLE,
    RAMPUP,
    BEMF_SENSING
} CONTROL_STATE_e;

extern CONTROL_STATE_e motor_state;

bool_t bemf_sensing();

void rampUp();

void trapezoidal_commute();

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

#endif
