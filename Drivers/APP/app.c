#include "app.h"


volatile uint16_t adc_buffer[ADC_CHANNELS];
TX_BUFFER tx_buffer;
const uint16_t tx_buffer_size = sizeof(tx_buffer);
uint16_t dataToSend[UART_NUM_BYTES];


volatile BldcHandler_t pDriver;


void run_app(){
	bldc_init(&pDriver);
	PIDController pid;
	pidInit(&pid, 10.0, 10.0, 0.0);

	uint8_t sw_state = HAL_GPIO_ReadPin(TOGGLE_SW_GPIO_Port, TOGGLE_SW_Pin);
    if(sw_state)
    		bldc_set_pwm(&pDriver, 0);

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);
	HAL_TIM_Base_Start(&htim17);
	//HAL_TIM_Base_Start_IT(&htim16);
	bldc_align_motor(&pDriver);
	bldc_ramp(&pDriver);

    while(1){
    	HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        uint8_t sw_state = HAL_GPIO_ReadPin(TOGGLE_SW_GPIO_Port, TOGGLE_SW_Pin);
        if(sw_state){
        	bldc_set_pwm(&pDriver, 0);
        }
    }

}

void guiDataTransmit(volatile BldcHandler_t* pDriver){

	tx_buffer.phasesV[0] = pDriver->commutation.vA;
	tx_buffer.phasesV[1] = pDriver->commutation.vB;
	tx_buffer.phasesV[2] = pDriver->commutation.vC;
	tx_buffer.phasesI[0] = pDriver->commutation.iA;
	tx_buffer.phasesI[1] = pDriver->commutation.iB;
	tx_buffer.phasesI[2] = pDriver->commutation.iC;
	tx_buffer.vinRef 	 = pDriver->commutation.vinRef;
	tx_buffer.temperature[0] = 0;
	tx_buffer.temperature[1] = 0;
	tx_buffer.temperature[2] = 0;
	tx_buffer.potValue = 0;
	tx_buffer.pwmDutyCycle = pDriver->commutation.currentPwmDutyCycle;
	//uint16_t temp = (eepromRegRD[1] << 8) | eepromRegRD[0];
	//tx_buffer.eepromVal = temp;
	//tx_buffer.errorcode = errorHandler;
	memcpy(dataToSend, &tx_buffer, sizeof(tx_buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)dataToSend, sizeof(dataToSend), HAL_MAX_DELAY);
//

}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){


	if(htim->Instance == TIM14){
		static uint16_t align_steps_temp = ALIGN_STEPS;
		HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
		--align_steps_temp;
	}

	if(htim->Instance == TIM16){
		guiDataTransmit(&pDriver);

	}

}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

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
		pDriver.commutation.vA 	= adc_buffer[4];
		pDriver.commutation.vB 	= adc_buffer[2];
		pDriver.commutation.vC 	= adc_buffer[0];
		pDriver.commutation.vinRef = adc_buffer[6] / 2;

        //driver_conf.pwmDutyCycle = (uint16_t)(adc_buffer[10] * (1599.0f/4095.0f));
    }


//	if(AUTO_COMMUTATION == bldc_get_state(pHandler))
//	{
//		pidUpdate(&pid, setpoint, measurement)
//	}

    bldc_bemf_sensing(&pDriver);
    

    if(pDriver.motorStatus.bemf_flag == true && pDriver.motorStatus.event == BEMF_COUNTING)

    {
    	pDriver.commutation.bemf_counter++;

    }

    if(pDriver.motorStatus.bemf_flag == true && pDriver.motorStatus.state == AUTO_COMMUTATION)
    {

    	bldc_trapezoidal_commute(&pDriver);

    }

    pDriver.motorStatus.bemf_flag = false;
    pDriver.commutation.prev_vA = pDriver.commutation.vA;
    pDriver.commutation.prev_vB = pDriver.commutation.vB;
    pDriver.commutation.prev_vC = pDriver.commutation.vC;
    
}
