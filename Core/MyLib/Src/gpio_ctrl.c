#include "gpio_ctrl.h"
#include "main.h"

void gpioInit(GPIO_TypeDef* port, uint32_t line){
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	port->MODER |= (0x1 << 2*line);
	port->OTYPER |= (0x0 << line);
	port->OSPEEDR |= (0x3 << 2*line);
	port->PUPDR |= (0x0 << 2*line);
}

void gpioSet(GPIO_TypeDef* port, uint32_t line, uint8_t state){
	uint32_t state_val;
	state_val = port->IDR;
	if (state == 1){
		port->ODR |= (state << line);
	} else{
		port->ODR &= state_val ^ (1 << line);
	}
}

uint8_t gpioGet(GPIO_TypeDef* port, uint32_t line){
	return port->IDR & (1 << line);
}

void gpioTogle(GPIO_TypeDef* port, uint32_t line){ // not work
	uint8_t state_valer;
	state_valer = gpioGet(port, line);
	gpioSet(port, line, ~(state_valer));
}
