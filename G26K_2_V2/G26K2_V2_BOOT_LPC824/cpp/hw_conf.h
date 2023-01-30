#ifndef HW_CONF_H__25_11_2022__16_53
#define HW_CONF_H__25_11_2022__16_53

#include <types.h>
#include <core.h>

#define MCK_MHz 25
#define MCK (MCK_MHz*1000000)
#define NS2CLK(x) (((x)*MCK_MHz+500)/1000)
#define US2CLK(x) ((x)*MCK_MHz)
#define MS2CLK(x) ((x)*MCK_MHz*1000)

#define BOOTLOADER

// ++++++++++++++	USIC	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define USART0_USIC_NUM		0
//#define USART1_USIC_NUM	1
//#define USART2_USIC_NUM	2
//#define SPI0_USIC_NUM		3
//#define SPI1_USIC_NUM		4
//#define I2C0_USIC_NUM		5
//#define I2C1_USIC_NUM		6
//#define I2C2_USIC_NUM		7
//#define I2C3_USIC_NUM		8

// ++++++++++++++	USART	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIN_UTX0			25 
#define PIN_URX0			15 
#define PIN_RTS0			24

#define UTX0				(1UL<<PIN_UTX0)
#define URX0				(1UL<<PIN_URX0)
#define RTS0				(1UL<<PIN_RTS0)

// ++++++++++++++	MOTOR	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIN_DHU				8 
#define PIN_DHV				9 
#define PIN_DHW				10
#define PIN_LIN1			18
#define PIN_HIN1			17
#define PIN_LIN2			20
#define PIN_HIN2			19
#define PIN_LIN3			22
#define PIN_HIN3			21
#define PIN_ENABLE			14

#define DHU					(1UL<<PIN_DHU)
#define DHV					(1UL<<PIN_DHV)
#define DHW					(1UL<<PIN_DHW)
#define LIN1				(1UL<<PIN_LIN1)
#define HIN1				(1UL<<PIN_HIN1)
#define LIN2				(1UL<<PIN_LIN2)
#define HIN2				(1UL<<PIN_HIN2)
#define LIN3				(1UL<<PIN_LIN3)
#define HIN3				(1UL<<PIN_HIN3)
#define ENABLE				(1UL<<PIN_ENABLE)

#define PIN_PWM90			27
#define PIN_ROT				26
#define PIN_FB90			13
#define PIN_FB_AUXPWR		23
#define PIN_ISEN			7
#define PIN_ILOW			6

#define PWM90				(1UL<<PIN_PWM90)
#define ROT					(1UL<<PIN_ROT)
#define FB90				(1UL<<PIN_FB90)
#define FB_AUXPWR			(1UL<<PIN_FB_AUXPWR)
#define ISEN				(1UL<<PIN_ISEN)
#define ILOW				(1UL<<PIN_ILOW)


// ++++++++++++++	PIO INIT	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

																							// 1  8 7  4 3  0 9  6 5  2 1  8 7  4 3  0
#define GPIO_INIT_DIR0		PWM90|ROT|RTS0|ENABLE|LIN1|HIN1|LIN2|HIN2|LIN3|HIN3|(1<<12)		// 0000 1101 0111 1110 0101 0000 0000 0000	
#define GPIO_INIT_PIN0		ENABLE															// 0000 0000 0111 1110 0100 0000 0000 0000


#endif // HW_CONF_H__25_11_2022__16_53
