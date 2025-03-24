/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Jan 3, 2025
 *      Author: prave
 */

#include<string.h>
#include"stm32f407xx.h"

//command codes
#define COMMANDD_LED_CTRL			0x50
#define COMMANDD_SENSOR_READ		0x50
#define COMMANDD_LED_READ			0x50
#define COMMANDD_PRINT				0x50
#define COMMANDD_ID_READ			0x50

#define LED_ON 		1
#define LED_OFF		0

//arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

//ardunio led
#define LED_PIN		9

void delay (void)
{
	for(uint32_t i=0; i<500000/2;i++);
}
/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode =5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO - disabling these pins so other user can use it
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS - disabling these pins so other user can use it
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIX = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;// generates sclk of 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;// software slave management enable for NSS pin

	SPI_Init(&SPI2handle);
}
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;
	// This is btn gpio configuration
		GPIOBtn.pGPIOx = GPIOA;
		GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
		GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


		GPIO_Init(&GPIOBtn);
}
int main(void)
{
	//uint8_t dummy_byte = 0xff;

	GPIO_ButtonInit();
	//This Function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	SPI_SSIConfig(SPI2,ENABLE);
	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */

	SPI_SSOEConfig(SPI2,ENABLE);
	while(1)
	{
		while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay();//key debouncing
		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);
		//This line will send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		//Wait until SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );
		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

	}

	return 0;
}

