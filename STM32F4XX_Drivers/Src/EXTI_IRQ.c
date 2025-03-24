/*
 * main.c
 *
 *  Created on: Nov 22, 2024
 *      Author: prave
 */

#include "stm32f407xx.h"
void EXTI0_IRQHandler(void);
int main(void)
{
	return 0;
}


void EXTI0_IRQHandler(void)
{
	//handle the interrupt
	GPIO_IRQHandling(0);
}
