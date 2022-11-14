#ifndef HW_CONF_H__11_11_22__17_22
#define HW_CONF_H__11_11_22__17_22

#define	CORETYPE_BF592

#include "bf592.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define CLKIN_MHz			25
#define CLKIN_DIV			2	// 1, 2

#define PLL_MUL				8	// 5...64
#define SCLK_DIV			1	// 1...15
#define CCLK_CSEL			0	// 0...3
#define CCLK_DIV			(1UL<<CCLK_CSEL)

#define VCO_CLK_MHz 		(CLKIN_MHz*PLL_MUL/CLKIN_DIV)
#define CCLK_MHz			VCO_CLK_MHz/CCLK_DIV
#define SCLK_MHz			VCO_CLK_MHz/SCLK_DIV

#define VRCTL_VALUE         0x0000

#if CLKIN_DIV == 2
#define PLLCTL_VALUE        (SET_MSEL(PLL_MUL)|DF)
#else
#define PLLCTL_VALUE        (SET_MSEL(PLL_MUL))
#endif

#define PLLDIV_VALUE        (SET_SSEL(SCLK_DIV))
#define PLLLOCKCNT_VALUE    0x0000
#define PLLSTAT_VALUE       0x0000
//#define RSICLK_DIV          0x0001

#define SCLK (SCLK_MHz*1000000)
#define CCLK (CCLK_MHz*1000000)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MS2CLK(x) ((u32)(x*1.0*SCLK/1e3+0.5))
#define US2CLK(x) ((u32)(x*1.0*SCLK/1e6+0.5))
#define NS2CLK(x) ((u32)(x*1.0*SCLK/1e9+0.5))

#define MS2CCLK(x) ((u32)(x*1.0*CCLK/1e3+0.5))
#define US2CCLK(x) ((u32)(x*1.0*CCLK/1e6+0.5))
#define NS2CCLK(x) ((u32)(x*1.0*CCLK/1e9+0.5))

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// 5 - PF5 - 
// 6 - PF6 - 
// 7 - PF7 - 
// 8 - PF8 - SpiFlashSelect - Main Loop
// 36 - PG4 - FIRE_PPI_ISR
// 37 - PG5
// 38 - PG6
// 39 - PG7

// Вектора прерываний
// IVG7		- 
// IVG8 	- DMA0 (PPI)
// IVG9 	- PORTF PF4 SYNC
// IVG10 	- GPTIMER0 FIRE
// IVG11 	- GPTIMER2 RTT
// IVG12 	- TWI


// CoreTimer - PPI delay

// TIMER0 	- Fire
// TIMER1 	- PPI CLK
// TIMER2 	- RTT

// UART0	- 
// SPI0		- Boot flash
// SPI1 	- 
// TWI		- 

#define IVG_EMULATION		0
#define IVG_RESET			1
#define IVG_NMI				2
#define IVG_EXEPTIONS		3
#define IVG_HW_ERROR		5
#define IVG_CORETIMER		6
//#define IVG_PORTF_SYNC		7
//#define IVG_PORTF_SHAFT		8
//#define IVG_GPTIMER2_RTT	9
//#define IVG_PPI_DMA0		10
//#define IVG_PORTG_ROT		11
//#define IVG_TWI				12
//#define IVG_GPTIMER0_FIRE	10

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define INIT_PORTF_MUX			0x000F		//  0000 0000 0000 1111
#define INIT_PORTG_MUX			0xFF00		//  1111 1111 0000 0000

#define INIT_PORTF_FER 			0x1E0F		//  0001 1110 0000 1111
#define INIT_PORTG_FER 			0xFF00		//  1111 1111 0000 0000

#define INIT_PORTFIO_DIR 		0x01A0		//  0000 0001 1010 0000
#define INIT_PORTGIO_DIR 		0x00FF		//  0000 0000 1111 1111

#define INIT_PORTFIO_INEN 		0x0000		//  0000 0000 0000 0000
#define INIT_PORTGIO_INEN 		0x0000		//  0000 0000 0000 0000

#define INIT_PORTGIO 			0
#define INIT_PORTFIO 			0

#define INIT_PORTFIO_POLAR		0
#define INIT_PORTFIO_EDGE 		0
#define INIT_PORTFIO_BOTH 		0
#define INIT_PORTFIO_MASKA		0
#define INIT_PORTFIO_MASKB		0

#define INIT_PORTGIO_POLAR		0
#define INIT_PORTGIO_EDGE 		0
#define INIT_PORTGIO_BOTH 		0
#define INIT_PORTGIO_MASKA		0
#define INIT_PORTGIO_MASKB		0

#define INIT_WDOG_CNT			MS2CLK(10)
#define INIT_WDOG_CTL			WDEV_RESET|WDEN

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BOOT_NETADR
#define BOOT_COM_SPEED			6250000
#define BOOT_COM_PARITY			2
#define BOOT_COM_PRETIMEOUT		(~0)
#define BOOT_COM_POSTTIMEOUT	(US2COM(500))

#define BOOT_MAN_REQ_WORD		0xAD00
#define BOOT_MAN_REQ_MASK 		0xFF00

#define PIO_RTS_SET				*pPORTFIO_SET
#define PIO_RTS_CLR				*pPORTFIO_CLEAR
#define PIO_RTS_DIR				*pPORTFIO_DIR
#define PIO_RTS_MUX				*pPORTF_MUX

#define PIN_RTS					5
#define MASK_RTS				(1UL<<PIN_RTS)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MAIN_LOOP_PIN_SET()		{*pPORTFIO_SET = 1<<7;}
#define MAIN_LOOP_PIN_CLR()		{*pPORTFIO_CLEAR = 1<<7;}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // HW_CONF_H__11_11_22__17_22
