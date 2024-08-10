#include "app.h"


volatile uint16_t adc_buffer[ADC_CHANNELS];
TX_BUFFER tx_buffer;
const uint16_t tx_buffer_size = sizeof(tx_buffer);
uint16_t dataToSend[UART_NUM_BYTES];

void run_app(){
	uint8_t sw_state = HAL_GPIO_ReadPin(TOGGLE_SW_GPIO_Port, TOGGLE_SW_Pin);
    if(sw_state)
    		set_phases_pwm_dc(0);

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);
	HAL_TIM_Base_Start(&htim17);
	//HAL_TIM_Base_Start_IT(&htim16);
	bldc_init();
	align_motor();

	rampUp();
	//all_ch_OFF();

    while(1){
    	HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        uint8_t sw_state = HAL_GPIO_ReadPin(TOGGLE_SW_GPIO_Port, TOGGLE_SW_Pin);
        if(sw_state){
        		set_phases_pwm_dc(0);
        }
    }

}

void guiDataTransmit(){

	tx_buffer.phasesV[0] = driver.vA;
	tx_buffer.phasesV[1] = driver.vB;
	tx_buffer.phasesV[2] = driver.vC;
	tx_buffer.phasesI[0] = driver.iA;
	tx_buffer.phasesI[1] = driver.iB;
	tx_buffer.phasesI[2] = driver.iC;
	tx_buffer.vinRef = driver.vinRef;
	tx_buffer.temperature[0] = 0;
	tx_buffer.temperature[1] = 0;
	tx_buffer.temperature[2] = 0;
	tx_buffer.potValue = 0;
	tx_buffer.pwmDutyCycle = PWM_TIMER->Instance->CCR1;
	//uint16_t temp = (eepromRegRD[1] << 8) | eepromRegRD[0];
	//tx_buffer.eepromVal = temp;
	//tx_buffer.errorcode = errorHandler;
	memcpy(dataToSend, &tx_buffer, sizeof(tx_buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)dataToSend, sizeof(dataToSend), HAL_MAX_DELAY);
//
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM14){
		HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
		--alignSteps;
		//PWM_TIMER->Instance->CCR1 += ALIGN_PWM_INCREMENT;
	}

	if(htim->Instance == TIM16){
		guiDataTransmit();

	}

}



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
        //driver_conf.pwmDutyCycle = (uint16_t)(adc_buffer[10] * (1599.0f/4095.0f));
    }

//	driver.pwmDutyCycle = driver.pwmDutyCycle * (1599/4095);
//
//	TIM1->CCR1 = driver.pwmDutyCycle;
//	TIM1->CCR2 = driver.pwmDutyCycle;
//	TIM1->CCR3 = driver.pwmDutyCycle;

    bemf_flag = bemf_sensing();
    
    if(bemf_flag == true && motor_event == BEMF_COUNTING)
    	++bemf_counter;

    if(bemf_flag == true && motor_event == BEMF_SENSING){
        //motor_state = AUTO_COMMUTATION;
    	trapezoidal_commute();

    }
    
    bemf_flag = false;
    driver.prev_vA = driver.vA;
    driver.prev_vB = driver.vB;
    driver.prev_vC = driver.vC;

}
