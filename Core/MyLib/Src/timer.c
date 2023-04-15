#include "gpio_ctrl.h"
#include "main.h"
#include "timer.h"

static volatile uint16_t flag = 0;

void timerInit(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->CR1 |= (0<<11)|(0<<7)|(1<<3)|(0<<1)|(0<<0);
	TIM6->CR2 = 0;
	TIM6->EGR = 0;
	TIM6->DIER |= (1<<0); // прерывания отключены
	TIM6->PSC = 9999; // период счетчика 1 мс
	while ((TIM6->CR1 & 0x01) != 0x0){

	}
}

void timerOn(uint32_t p){
	TIM6->CR1 |= (1<<0);
	TIM6->ARR = p;
}

void timerOff(){
	TIM6->CR1 &= (0<<0);
}

void TIM6_DAC_IRQHandler(void)
{
	timerOff();
	TIM6->SR &= ~(0x01); // обнуление флага вызова
}