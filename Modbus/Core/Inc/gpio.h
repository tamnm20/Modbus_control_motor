/*
 * gpio.h
 *
 *  Created on: Oct 13, 2025
 *      Author: TAMRD
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f4xx_hal.h"

extern volatile uint8_t x_home_done;
extern volatile uint8_t y_home_done;

void io_init(void);
void exti_init(void);
// ================== GPIO ==================
static inline void gpio_set(GPIO_TypeDef *port, uint8_t pin) { port->BSRR = (1u << pin); }
static inline void gpio_clr(GPIO_TypeDef *port, uint8_t pin) { port->BSRR = (1u << (pin + 16)); }
void delay_ms(uint32_t ms);

#endif /* INC_GPIO_H_ */
