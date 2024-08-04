#include "app.h"


volatile uint16_t adc_buffer[ADC_CHANNELS];


void run_app(){
	HAL_ADCEx_Calibration_Start(&hadc1);
//	HAL_TIM_Base_Start_IT(&htim14);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);
	TIM1->CCR1 = 300;
	TIM1->CCR2 = 300;
	TIM1->CCR3 = 300;
	HAL_TIM_Base_Start(&htim17);
	rampUp();
	all_ch_OFF();
    while(1){
        uint8_t sw_state = HAL_GPIO_ReadPin(TOGGLE_SW_GPIO_Port, TOGGLE_SW_Pin);
        if(sw_state){
        		TIM1->CCR1 = driver.pwmDutyCycle;
        		TIM1->CCR2 = driver.pwmDutyCycle;
        		TIM1->CCR3 = driver.pwmDutyCycle;//test
        }
    }

}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//
///*	if(htim->Instance == TIM14){
//		HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
//	}*/
//
//}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){


	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);
//	HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);


	/* ADC BUFFER:
	 * [0]: 	V_C
	 * [1]: 	I_C
	 * [2]: 	V_B
	 * [3]: 	I_B
	 * [4]: 	V_A
	 * [5]: 	I_A
	 * [6]: 	V_VIN
	 * [7]: 	TEMP_PHC
	 * [8]: 	TEMP_PHB
	 * [9]: 	TEMP_PHA
	 * [10]:	POTENTIOMETER
	*/

	if(hadc->Instance == ADC1){
        driver.vA = adc_buffer[4];
        driver.vB = adc_buffer[2];
        driver.vC = adc_buffer[0];       
        driver.vinRef = adc_buffer[6] / 2;
        driver.pwmDutyCycle = (uint16_t)(adc_buffer[10] * (1599.0f/4095.0f));
    }

//	driver.pwmDutyCycle = driver.pwmDutyCycle * (1599/4095);
//
//	TIM1->CCR1 = driver.pwmDutyCycle;
//	TIM1->CCR2 = driver.pwmDutyCycle;
//	TIM1->CCR3 = driver.pwmDutyCycle;

    bemf_flag = bemf_sensing();
    
    if(bemf_flag == true && motor_state == BEMF_SENSING)
        trapezoidal_commute();
    
    bemf_flag = false;
    driver.prev_vA = driver.vA;
    driver.prev_vB = driver.vB;
    driver.prev_vC = driver.vC;

}
