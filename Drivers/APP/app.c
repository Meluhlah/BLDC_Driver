#include "app.h"


volatile uint16_t adc_buffer[ADC_CHANNELS];
TX_BUFFER tx_buffer;
const uint16_t tx_buffer_size = sizeof(tx_buffer);
uint16_t dataToSend[UART_NUM_BYTES];


volatile Bldc_t* pDriver = NULL;
volatile Bldc_Handler_t* pHandler = NULL;
volatile BldcParamsConfig_t* pConfig = NULL;


void run_app(){
	pDriver = bldc_init();
	pHandler = bldc_init_handler();
	pConfig = bldc_init_config();

	uint8_t sw_state = HAL_GPIO_ReadPin(TOGGLE_SW_GPIO_Port, TOGGLE_SW_Pin);
    if(sw_state)
    		bldc_set_pwm(pDriver, 0);

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);
	HAL_TIM_Base_Start(&htim17);
	//HAL_TIM_Base_Start_IT(&htim16);
	bldc_align_motor(pDriver);
	bldc_ramp(pDriver, pConfig, pHandler);

    while(1){
    	HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        uint8_t sw_state = HAL_GPIO_ReadPin(TOGGLE_SW_GPIO_Port, TOGGLE_SW_Pin);
        if(sw_state){
        	bldc_set_pwm(pDriver, 0);
        }
    }

}

void guiDataTransmit(volatile Bldc_t* pDriver){

	tx_buffer.phasesV[0] = bldc_get_vA(pDriver);
	tx_buffer.phasesV[1] = bldc_get_vB(pDriver);
	tx_buffer.phasesV[2] = bldc_get_vC(pDriver);
	tx_buffer.phasesI[0] = bldc_get_iA(pDriver);
	tx_buffer.phasesI[1] = bldc_get_iB(pDriver);
	tx_buffer.phasesI[2] = bldc_get_iC(pDriver);
	tx_buffer.vinRef = bldc_get_vinRef(pDriver);
	tx_buffer.temperature[0] = 0;
	tx_buffer.temperature[1] = 0;
	tx_buffer.temperature[2] = 0;
	tx_buffer.potValue = 0;
	tx_buffer.pwmDutyCycle = bldc_get_pwm(pDriver);
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
		guiDataTransmit(pDriver);

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

	bool_t bemf_flag = false;

	if(hadc->Instance == ADC1){
		bldc_set_vA(pDriver, adc_buffer[4]);
		bldc_set_vB(pDriver, adc_buffer[2]);
		bldc_set_vC(pDriver, adc_buffer[0]);
		bldc_set_vinRef(pDriver, adc_buffer[6] / 2);
        //driver_conf.pwmDutyCycle = (uint16_t)(adc_buffer[10] * (1599.0f/4095.0f));
    }


    bemf_flag = bldc_bemf_sensing(pDriver);
    

    if(bemf_flag == true && (bldc_get_event(pHandler) == BEMF_COUNTING))

    {
    	bldc_increment_bemf_counter(pDriver);

    }

    if(bemf_flag == true && (bldc_get_event(pHandler) == BEMF_SENSING)){

    	bldc_trapezoidal_commute(pDriver, pConfig, pHandler);

    }
    
    bemf_flag = false;
    bldc_set_prev_vA(pDriver, bldc_get_vA(pDriver));
    bldc_set_prev_vB(pDriver, bldc_get_vB(pDriver));
    bldc_set_prev_vC(pDriver, bldc_get_vC(pDriver));

}
