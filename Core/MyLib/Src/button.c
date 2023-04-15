#include "main.h"
#include "button.h"
#include "gpio_ctrl.h"
#include "timer.h"

#define GPIO_ODR_A0 (1 << 0)

void buttonInit(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= (0x0 << 0);
	GPIOA->OTYPER |= (0x0 << 0);
	GPIOA->OSPEEDR |= (0x0 << 0);
	GPIOA->PUPDR |= (0x0 << 0);
	NVIC_EnableIRQ(EXTI0_IRQn);
	EXTI->IMR |= (1 << 0);
	EXTI->RTSR |= (1 << 0); // по переднему фронту
	EXTI->FTSR &= ~(1 << 0);// по заднему фронту
}

void EXTI0_IRQHandler(void) {
	// static volatile uint8_t flag = 0;
	gpioSet(GPIOE, 9, 1);
	timerOn(1000);
	timerOff();
	EXTI->PR |= (1 << 0);
}