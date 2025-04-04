#include "hardware.h"
#include "spi.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define FLASHSPI_IMP_V2

#ifdef CPU_BF592
#define BOOT_SGUID DSP_BOOT_SGUID_BF592
#elif defined(CPU_BF706)
#define BOOT_SGUID DSP_BOOT_SGUID_BF706
#endif

#define BOOT_COM
#define BOOT_MAX_NETADR			DSP_BOOT_NET_ADR
#define BOOT_TIMEOUT			(2000)
#define BOOT_MAIN_TIMEOUT		(100000)
#define BOOT_COM_SPEED			DSP_BOOT_COM_BAUDRATE	
#define BOOT_COM_PARITY			DSP_BOOT_COM_PARITY	
#define BOOT_COM_STOPBITS		DSP_BOOT_COM_STOPBITS
#define BOOT_COM_PRETIMEOUT		(~0)
#define BOOT_COM_POSTTIMEOUT	(US2COM(100))
#define BOOT_COM_WRITEDELAY		(US2CTM(500))

#define BOOT_MAN_REQ_WORD		DSP_BOOT_REQ_WORD
#define BOOT_MAN_REQ_MASK 		DSP_BOOT_REQ_MASK

//#define BOOT_HW_UPDATE 			UpdateADC
#define BOOT_HW_INIT 			InitHardware

//#define PIO_RTS					HW::PIOF
//#define PIN_RTS					10
//#define MASK_RTS				(1UL<<PIN_RTS)

#define ADSP_CHECKFLASH
#define ADSP_CRC_PROTECTION
//#define FLASHSPI_WRITESYNC
//#define FLASHSPI_REQUESTUPDATE

//#define	NUM_SMALL_BUF	60      
//#define	NUM_MEDIUM_BUF	1
//#define	NUM_HUGE_BUF	1

//#define FLASH_IS25LP080D
#define FLASH_START_ADR 0x10000 	

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define MAIN_LOOP_PIN_TGL()		{ HW::PIOF->NOT(PF4);}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BAUD_RATE_DIVISOR 	5

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_BF592

static u16 SPI_CS_MASK[] = { PF8 };

static S_SPIM	spi(0, HW::PIOF, SPI_CS_MASK, ArraySize(SPI_CS_MASK), SCLK);

#elif defined(CPU_BF706)

static u16 SPI_CS_MASK[] = { PB15 };

static S_SPIM	spi(2, HW::PIOB, SPI_CS_MASK, ArraySize(SPI_CS_MASK), SCLK);

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u16 GetNetAdr() { return DSP_BOOT_NET_ADR; }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#undef BOOT_TIMEOUT			
//#undef BOOT_MAIN_TIMEOUT	
//#define BOOT_TIMEOUT			(2000)
//#define BOOT_MAIN_TIMEOUT		(100000)

#define Pin_MainLoop_Tgl()			MAIN_LOOP_PIN_TGL()
//#define Pin_VerifyPageError_Set()	VERIFY_PAGE_ERROR_PIN_SET()
//#define Pin_VerifyPageError_Clr()	VERIFY_PAGE_ERROR_PIN_CLR()

#ifdef BOOT_COM

#include "Comport\ComPort_imp.h"

	#ifdef CPU_BF592

		static ComPort com;

	#elif defined(CPU_BF706)

		static ComPort com(1, PIO_RTS, PIN_RTS);

	#endif

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "FLASH\FlashSPI_imp_v2.h"

FlashSPI bootFlash(spi);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "BOOT\boot_com_emac_imp_v2.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

