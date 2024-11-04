#include <stm32g030xx.h>
#include "main.h"

#define WREN	0b00000110
#define WRDI	0b00000100
#define RDSR	0b00000101
#define WRSR	0b00000001
#define READ	0b00000011
#define WRITE	0b00000010

#define SPI_HANDLER	 hspi1

// ------------- Variables Address ---------------//
#define NUM_OF_DATA_REGS	(uint8_t)10

#define ALIGN_DC_REG		0x0000
#define ALIGN_STEPS_REG		0x0002

#define RAMPUP_DC_REG		0x0004
#define RAMPUP_STEPS_REG 	0x0006

#define MOTOR_POLES_REG		0x0008 // and 0x0005
#define DUMMY_REG			0xfffe


HAL_StatusTypeDef eeprom_read(uint16_t address, uint8_t* rx_data, uint8_t size){
	uint8_t status = 0;
	uint8_t addressH = (address >> 8) & 0xff;
	uint8_t addressL = address & 0xff; // A15-A14 D'ont Care
	uint8_t readAddress[3] = {READ, addressH, addressL};
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	status += HAL_SPI_Transmit(&SPI_HANDLER, readAddress, sizeof(readAddress), 100);
	status += HAL_SPI_Receive(&SPI_HANDLER, rx_data, sizeof(rx_data), 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	return status;
}


HAL_StatusTypeDef eeprom_write(uint16_t address, uint8_t data[2]){
	uint8_t status = 0;
	uint8_t writeEN = WREN;
	uint8_t addressH = (address >> 8) & 0xff;
	uint8_t addressL = address & 0xff; // A15-A14 D'ont Care
	uint8_t writeData[5] = {WRITE, addressH, addressL, data[0], data[1]};
	// SET WREN Prior to writing data
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	status += HAL_SPI_Transmit(&SPI_HANDLER, &writeEN, 1, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	status += HAL_SPI_Transmit(&SPI_HANDLER, writeData, sizeof(writeData), 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	return status;
}
