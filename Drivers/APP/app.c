#include "app.h"


UartBufferTx_t uartBufferTx;
UartBufferRx_t uartBufferRx;

bool_t  uart_flag = false;

volatile uint16_t adc_buffer[ADC_CHANNELS];
volatile BldcHandler_t pDriver;
PIDController pid;


void run_app(){

	// -------------------------- Initi --------------------------------- //

	bldc_init(&pDriver);
	pidInit(&pid, 10.0, 10.0, 0.0);
	HAL_ADCEx_Calibration_Start(&hadc1);
	uint8_t sw_state = HAL_GPIO_ReadPin(TOGGLE_SW_GPIO_Port, TOGGLE_SW_Pin);
    if(sw_state)
    		bldc_set_pwm(&pDriver, 0);

    // ---------------- Start Communication Links ---------------------- //

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);
//	HAL_TIM_Base_Start(&htim17);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&uartBufferRx, sizeof(UartBufferRx_t));



    while(1){


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
		pDriver.commutation.vA 		 = adc_buffer[4];
		pDriver.commutation.vB 		 = adc_buffer[2];
		pDriver.commutation.vC 		 = adc_buffer[0];
		pDriver.commutation.iA		 = adc_buffer[5];
		pDriver.commutation.iB		 = adc_buffer[3];
		pDriver.commutation.iC		 = adc_buffer[1];
		pDriver.commutation.vinRef 	 = (adc_buffer[6]) / 2;
		pDriver.commutation.tempA	 = adc_buffer[9];
		pDriver.commutation.tempB	 = adc_buffer[8];
		pDriver.commutation.tempC	 = adc_buffer[7];
		pDriver.commutation.potValue = adc_buffer[10];
		pDriver.commutation.rpmValue = 0;

    }

    bldc_bemf_sensing(&pDriver);
    
    if(pDriver.motorStatus.bemf_flag == true && pDriver.motorStatus.event == BEMF_SENSING)

    {
    	pDriver.commutation.bemf_counter++;
    	pDriver.motorStatus.state = MOTOR_AUTO_COMMUTATION;

    }

    if(pDriver.motorStatus.bemf_flag == true && pDriver.motorStatus.state == MOTOR_AUTO_COMMUTATION)
    {

    	bldc_trapezoidal_commute(&pDriver);

    }

    pDriver.motorStatus.bemf_flag = false;
    pDriver.commutation.prev_vA = pDriver.commutation.vA;
    pDriver.commutation.prev_vB = pDriver.commutation.vB;
    pDriver.commutation.prev_vC = pDriver.commutation.vC;

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	switch(uartBufferRx.command){

		case CMD_MOTOR_ALIGN:
			bldc_align_motor_start(&pDriver);
			break;

		case CMD_MOTOR_START:
			bldc_ramp_start(&pDriver);
			break;

		case CMD_MOTOR_STOP:
			bldc_all_phases_off();
			bldc_motor_status_init(&pDriver);
			break;

		default:
			HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
		}

	HAL_UART_Receive_IT(&huart1, (uint8_t*)&uartBufferRx, sizeof(UartBufferRx_t));


}


void guiDataTransmit(volatile BldcHandler_t* pDriver){

	uartBufferTx.phasesV[0] 		= pDriver->commutation.vA;
	uartBufferTx.phasesV[1] 		= pDriver->commutation.vB;
	uartBufferTx.phasesV[2] 		= pDriver->commutation.vC;
	uartBufferTx.phasesI[0] 		= pDriver->commutation.iA;
	uartBufferTx.phasesI[1] 		= pDriver->commutation.iB;
	uartBufferTx.phasesI[2] 		= pDriver->commutation.iC;
	uartBufferTx.vinRef 	 		= pDriver->commutation.vinRef;
	uartBufferTx.temperature[0]		= pDriver->commutation.tempA;
	uartBufferTx.temperature[1]		= pDriver->commutation.tempB;
	uartBufferTx.temperature[2] 	= pDriver->commutation.tempC;
	uartBufferTx.potValue 			= pDriver->commutation.potValue;
	uartBufferTx.pwmDutyCycle 		= pDriver->commutation.pwm_dutyCycle;

	// --------------------------------------- Checksum --------------------------------------------- //
	uartBufferTx.checkSum 			=
			uartBufferTx.phasesV[0] 		+	uartBufferTx.phasesV[1] 		+	uartBufferTx.phasesV[2] +
			uartBufferTx.phasesI[0] 		+	uartBufferTx.phasesI[1] 		+	uartBufferTx.phasesI[2] +
			uartBufferTx.vinRef 			+	uartBufferTx.temperature[0] 	+	uartBufferTx.temperature[1] +
			uartBufferTx.temperature[2] 	+	uartBufferTx.potValue 			+	uartBufferTx.pwmDutyCycle;

	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&uartBufferTx, sizeof(uartBufferTx));
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){


	if(htim->Instance == TIM14){	// ----- 1ms Interrupt ----- //

		if(bldcAlignCounter > 0)
			bldcAlignCounter--;

		bldc_align_motor_step(&pDriver);

	}

	if(htim->Instance == TIM17){ 	// ----- 1ms Interrupt ----- //
    	if(bldcRampCounter > 0){
    		bldcRampCounter--;
    	}
    		bldc_ramp_step(&pDriver);

	}


	if(htim->Instance == TIM16){	//	----- 100ms Interrupt ----- //
		guiDataTransmit(&pDriver);

	}



}
