#ifndef G26K_9_HW_CONF_H__14_08_2024__11_44
#define G26K_9_HW_CONF_H__14_08_2024__11_44

#pragma once

#include "types.h"
#include "core.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define CLKIN_MHz			25

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __ADSPBF59x__ //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#ifndef BOOTLOADER

		#define CLKIN_DIV			1	// 1, 2

		#define PLL_MUL				16	// 5...64
		#define SCLK_DIV			2	// 1...15
		#define CCLK_CSEL			0	// 0...3
		#define CCLK_DIV			(1UL<<CCLK_CSEL)

	#else

		#define CLKIN_MHz			25
		#define CLKIN_DIV			1	// 1, 2

		#define PLL_MUL				16	// 5...64
		#define SCLK_DIV			2	// 1...15
		#define CCLK_CSEL			0	// 0...3
		#define CCLK_DIV			(1UL<<CCLK_CSEL)

	#endif

	#define VCO_CLK_MHz 		(CLKIN_MHz*PLL_MUL/CLKIN_DIV)
	#define CCLK_MHz			(VCO_CLK_MHz/CCLK_DIV)
	#define SCLK_MHz			(VCO_CLK_MHz/SCLK_DIV)

	#define VRCTL_VALUE         0x0000

	#if CLKIN_DIV == 2
		#define PLLCTL_VALUE        (SET_MSEL(PLL_MUL)|DF)
	#else
		#define PLLCTL_VALUE        (SET_MSEL(PLL_MUL))
	#endif

	#define PLLDIV_VALUE        (SET_SSEL(SCLK_DIV))
	#define PLLLOCKCNT_VALUE    0x0000
	#define PLLSTAT_VALUE       0x0000

	#define SCLK (SCLK_MHz*1000000)
	#define CCLK (CCLK_MHz*1000000)

#elif defined(__ADSPBF70x__) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#ifndef BOOTLOADER

		#define CLKIN_DIV			1	// 1, 2

		#define PLL_MUL				16	// 1...128
		#define CCLK_DIV			1   // 1...32
		#define SCLK_DIV			2	// 1...32
		#define SCLK0_DIV			1	// 1...8
		#define SCLK1_DIV			1	// 1...8
		#define DCLK_DIV			1	// 1...32	DRAM clock
		#define OCLK_DIV			1	// 1...128  Output clock

	#else

		#define CLKIN_DIV			1	// 1, 2

		#define PLL_MUL				16	// 1...128
		#define CCLK_DIV			2   // 1...32
		#define SCLK_DIV			2	// 1...32
		#define SCLK0_DIV			1	// 1...8
		#define SCLK1_DIV			1	// 1...8
		#define DCLK_DIV			1	// 1...32	DRAM clock
		#define OCLK_DIV			1	// 1...128  Output clock

	#endif

	#define VCO_CLK_MHz 		(CLKIN_MHz*PLL_MUL/CLKIN_DIV)
	#define CCLK_MHz			(VCO_CLK_MHz/CCLK_DIV)
	#define SCLK_MHz			(VCO_CLK_MHz/SCLK_DIV)
	#define SCLK0_MHz			(SCLK_MHz/SCLK0_DIV)
	#define SCLK1_MHz			(SCLK_MHz/SCLK1_DIV)

	#if (VCO_CLK_MHz < 231) || (VCO_CLK_MHz > 800)
		#error VCO_CLK_MHz must be 231...800
	#endif

	#if (CCLK_MHz > 400)
		#error CCLK_MHz must be <= 400
	#endif

	#if (SCLK_MHz > 200)
		#error SCLK_MHz must be <= 200
	#endif

	#if CLKIN_DIV == 2
		#define CGUCTL_VALUE        (CGU_CTL_MSEL(PLL_MUL)|CGU_CTL_DF)
	#else
		#define CGUCTL_VALUE        (CGU_CTL_MSEL(PLL_MUL))
	#endif

	#define CCLK (CCLK_MHz*1000000)
	#define SCLK (SCLK_MHz*1000000)
	#define SCLK0 (SCLK0_MHz*1000000)
	#define SCLK1 (SCLK1_MHz*1000000)

#endif //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MS2SCLK(x) ((u64)((x)*SCLK_MHz*1000))
#define US2SCLK(x) ((u64)((x)*SCLK_MHz))
#define NS2SCLK(x) ((u64)(((x)*SCLK_MHz+500)/1000))

#define MS2SCLK0(x) ((u64)((x)*SCLK0_MHz*1000))
#define US2SCLK0(x) ((u64)((x)*SCLK0_MHz))
#define NS2SCLK0(x) ((u64)(((x)*SCLK0_MHz+500)/1000))

#define MS2SCLK1(x) ((u64)((x)*SCLK1_MHz*1000))
#define US2SCLK1(x) ((u64)((x)*SCLK1_MHz))
#define NS2SCLK1(x) ((u64)(((x)*SCLK1_MHz+500)/1000))

#define MS2CCLK(x) ((u64)((x)*CCLK_MHz*1000))
#define US2CCLK(x) ((u64)((x)*CCLK_MHz))
#define NS2CCLK(x) ((u64)(((x)*CCLK_MHz+500)/1000))

#define MS2CLK(x) MS2SCLK(x)
#define US2CLK(x) US2SCLK(x)
#define NS2CLK(x) NS2SCLK(x)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define IVG_EMULATION		0
#define IVG_RESET			1
#define IVG_NMI				2
#define IVG_EXEPTIONS		3
#define IVG_HW_ERROR		5
#define IVG_CORETIMER		6
#define IVG_PORTF_SYNC		7
#define IVG_PORTF_SHAFT		8
#define IVG_GPTIMER2_RTT	9
#define IVG_SPORT0_DMA		10
#define IVG_SPORT1_DMA		11
#define IVG_PORTG_ROT		12
#define IVG_TWI				13

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __ADSPBF59x__ //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// 17 - PF14 - DSP_MISO - DD3-2 
// 34 - PG3  - UpdateMode
// 37 - PG5  - SPORT0_ISR
// 47 - PG12 - SPORT1_ISR
// 48 - PG13 - ProcessSPORT
// 49 - PG14 - Main Loop

#define Pin_UpdateMode_Set()	HW::PIOG->BSET(3)
#define Pin_UpdateMode_Clr()	HW::PIOG->BCLR(3)

#define Pin_SPORT0_ISR_Set()	HW::PIOG->BSET(5)
#define Pin_SPORT0_ISR_Clr()	HW::PIOG->BCLR(5)

#define Pin_SPORT1_ISR_Set()	HW::PIOG->BSET(12)
#define Pin_SPORT1_ISR_Clr()	HW::PIOG->BCLR(12)

#define Pin_ProcessSPORT_Set()	HW::PIOG->BSET(13)
#define Pin_ProcessSPORT_Clr()	HW::PIOG->BCLR(13)



// ++++++++++++++	DMA	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define	PPI_DMA				0
#define	SPORT0_RX_DMA		1
//#define	SPORT0_TX_DMA		2
#define	SPORT1_RX_DMA		3
//#define	SPORT1_TX_DMA		4
#define	SPI0_DMA			5
#define	SPI1_DMA			6
#define	UART_RX_DMA			7
#define	UART_TX_DMA			8

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_FIRE			HW::PIOF

#define PIN_FIRE1			9
#define PIN_FIRE2			10

#define BM_FIRE1			(1UL << PIN_FIRE1)
#define BM_FIRE2			(1UL << PIN_FIRE2)

#define FIRE1_TIMER			HW::TIMER1
#define FIRE2_TIMER			HW::TIMER0

#define FIRE1_TIMEN			TIMEN1
#define FIRE2_TIMEN			TIMEN0

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_DSHAFT			HW::PIOF
#define PIO_SYNC			HW::PIOF
#define PIO_ROT				HW::PIOG
#define PIO_RST_SW_ARR		HW::PIOG

#define PIN_DSHAFT			3
#define PIN_SYNC			4
#define PIN_ROT				15
#define PIN_RST_SW_ARR		4

#define BM_DSHAFT			(1U << PIN_DSHAFT)	
#define BM_SYNC				(1U << PIN_SYNC)
#define BM_ROT				(1U << PIN_ROT)
#define BM_RST_SW_ARR		(1U << PIN_RST_SW_ARR)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_RTS					HW::PIOF
#define PIN_RTS					10
#define MASK_RTS				(1UL<<PIN_RTS)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_MUX_SYNC			HW::PIOG
#define PIO_MUX_RESET			HW::PIOG

#define PIN_MUX_SCK				8
#define PIN_MUX_DIN				9
#define PIN_MUX_RESET			10
#define PIN_MUX_SYNC			11

#define BM_MUX_SCK				(1UL<<PIN_MUX_SCK	)
#define BM_MUX_DIN				(1UL<<PIN_MUX_DIN	)
#define BM_MUX_RESET			(1UL<<PIN_MUX_RESET	)
#define BM_MUX_SYNC				(1UL<<PIN_MUX_SYNC	)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define INIT_PORTF_MUX			0x0000		//  0000 0000 0000 0000
#define INIT_PORTG_MUX			0x0000		//  0000 0000 0000 0000

#define INIT_PORTF_FER 			0x000F		//  0000 0000 0000 1111
#define INIT_PORTG_FER 			0x000F		//  0000 0000 0000 1111

#define INIT_PORTFIO_DIR 		0x05F0		//  0000 0101 1111 0000
#define INIT_PORTGIO_DIR 		0xFFF0		//  1111 1111 1111 0000

#define INIT_PORTFIO_INEN 		0x0000		//  0000 0000 0000 0000
#define INIT_PORTGIO_INEN 		0x0000		//  0000 0000 0000 0000

#define INIT_PORTFIO 			MASK_RTS
#define INIT_PORTGIO 			0

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

#ifndef __DEBUG
#define INIT_WDOG_CNT			MS2CLK(100)
#define INIT_WDOG_CTL			WDEV_RESET|WDEN
#else
#define INIT_WDOG_CNT			MS2CLK(100)
#define INIT_WDOG_CTL			WDEV_RESET|WDDIS
#endif

#define PIO_MAINLOOP			HW::PIOF
#define PIN_MAINLOOP			4

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif defined(__ADSPBF70x__) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define Pin_UpdateMode_Set()	HW::PIOA->BSET(3)
#define Pin_UpdateMode_Clr()	HW::PIOA->BCLR(3)

#define Pin_ProcessSPORT_Set()	HW::PIOA->BSET(13)
#define Pin_ProcessSPORT_Clr()	HW::PIOA->BCLR(13)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_FIRE			HW::PIOA

#define PIN_FIRE1			5
#define PIN_FIRE2			6

#define BM_FIRE1			(1UL << PIN_FIRE1)
#define BM_FIRE2			(1UL << PIN_FIRE2)

#define FIRE1_TIMER			HW::TIMER->TMR[0]
#define FIRE2_TIMER			HW::TIMER->TMR[1]

#define FIRE1_TIMEN			TIMER_TMR0
#define FIRE2_TIMEN			TIMER_TMR1

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_DSHAFT			HW::PIOB
#define PIO_SYNC			HW::PIOA
#define PIO_ROT				HW::PIOC
#define PIO_RST_SW_ARR		HW::PIOA

#define PIN_DSHAFT			7
#define PIN_SYNC			1
#define PIN_ROT				3
#define PIN_RST_SW_ARR		12

#define BM_DSHAFT			(1U << PIN_DSHAFT)	
#define BM_SYNC				(1U << PIN_SYNC)
#define BM_ROT				(1U << PIN_ROT)
#define BM_RST_SW_ARR		(1U << PIN_RST_SW_ARR)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_RTS					HW::PIOC
#define PIN_RTS					2
#define MASK_RTS				(1UL<<PIN_RTS)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIN_DSP_CS				15
#define PIN_DSP_MOSI			12
#define PIN_DSP_MISO			11
#define PIN_DSP_SCK				10

#define BM_DSP_CS				(1UL<<PIN_DSP_CS	)
#define BM_DSP_MOSI				(1UL<<PIN_DSP_MOSI	)
#define BM_DSP_MISO				(1UL<<PIN_DSP_MISO	)
#define BM_DSP_SCK				(1UL<<PIN_DSP_SCK	)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_MUX_SYNC			HW::PIOA
#define PIO_MUX_RESET			HW::PIOA

#define PIN_MUX_SCK				0
#define PIN_MUX_DIN				2
#define PIN_MUX_RESET			15
#define PIN_MUX_SYNC			14

#define BM_MUX_SCK				(1UL<<PIN_MUX_SCK	)
#define BM_MUX_DIN				(1UL<<PIN_MUX_DIN	)
#define BM_MUX_RESET			(1UL<<PIN_MUX_RESET	)
#define BM_MUX_SYNC				(1UL<<PIN_MUX_SYNC	)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define INIT_PORTA_MUX			0x0000						
#define INIT_PORTB_MUX			0x0000						
#define INIT_PORTC_MUX			0x0000						

#define INIT_PORTA_FER 			0x0000						
#define INIT_PORTB_FER 			0x0000						
#define INIT_PORTC_FER 			0x0000						

#define INIT_PORTA_DIR 			(PA13)
#define INIT_PORTB_DIR 			(PB4|PB5)
#define INIT_PORTC_DIR 			(MASK_RTS|PC4|PC5|PC6|PC7|PC8|PC9|PC10)

#define INIT_PORTA_INEN 		0x0000		
#define INIT_PORTB_INEN 		0x0000		
#define INIT_PORTC_INEN 		0x0000		

#define INIT_PORTA_DATA 		0
#define INIT_PORTB_DATA 		0
#define INIT_PORTC_DATA 		0


#ifndef __DEBUG
#define INIT_WDOG_CNT			MS2SCLK0(100)
#define INIT_WDOG_CTL			WDOG_WDEN
#else
#define INIT_WDOG_CNT			MS2SCLK0(100)
#define INIT_WDOG_CTL			WDOG_WDDIS
#endif

#define PIO_MAINLOOP			HW::PIOB
#define PIN_MAINLOOP			5

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MAIN_LOOP_PIN_SET()		{	PIO_MAINLOOP->BSET(PIN_MAINLOOP);	}
#define MAIN_LOOP_PIN_CLR()		{	PIO_MAINLOOP->BCLR(PIN_MAINLOOP);	}
#define MAIN_LOOP_PIN_TGL()		{	PIO_MAINLOOP->BTGL(PIN_MAINLOOP);	}	

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // G26K_9_HW_CONF_H__14_08_2024__11_44
