#ifndef __REG_H_
#define __REG_H_

#define __REG_TYPE	volatile uint32_t
#define __REG		__REG_TYPE *

/* RCC Memory Map */
#define RCC		((__REG_TYPE) 0x40021000)
#define RCC_APB2ENR	((__REG) (RCC + 0x18))	// addr of clock enable register 2 
#define RCC_APB1ENR	((__REG) (RCC + 0x1C))	// addr of clock enable register 1

/* GPIO Memory Map */
#define GPIOA		((__REG_TYPE) 0x40010800)	// GPIO Port A 起始位址
#define GPIOA_CRL	((__REG) (GPIOA + 0x00))		// configuration register low
#define GPIOA_CRH	((__REG) (GPIOA + 0x04))		// configuration register high

/* USART2 Memory Map */
#define USART2	((__REG_TYPE) 0x40004400)	// USART 起始位址
#define USART2_SR	((__REG) (USART2 + 0x00))	// USART 狀態暫存器
#define USART2_DR	((__REG) (USART2 + 0x04))	// USART 資料暫存器
#define USART2_CR1	((__REG) (USART2 + 0x0C))	 // USART 控制暫存器 1

#endif
