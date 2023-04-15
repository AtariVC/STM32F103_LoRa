#ifndef MYCODE_GPIO_CTRL_H_
#define MYCODE_GPIO_CTRL_H_
#include "main.h"

void gpioInit(GPIO_TypeDef* port, uint32_t line);
void gpioSet(GPIO_TypeDef* port, uint32_t line, uint8_t state);
uint8_t gpioGet(GPIO_TypeDef* port, uint32_t line);
void gpioTogle(GPIO_TypeDef* port, uint32_t line);

#endif /* MYCODE_TIMER_H_ */