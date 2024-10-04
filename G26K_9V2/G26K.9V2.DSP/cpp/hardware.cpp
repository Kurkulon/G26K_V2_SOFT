#include "hardware.h"

#include <bfrom.h>
#include <sys\exception.h>
//#include <cdefBF592-A.h>
//#include <ccblkfn.h>

#include "list.h"
#include "DMA\DMA.h"
#include "spi.h"
#include "i2c.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <ADSP\system_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#pragma diag(push)
#pragma diag(suppress: 1970)

#define SPORT_BUF_NUM 5

#ifdef CPU_BF592 //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define Start_SPORT()	{ HW::SPORT0->TCR1 = sp0TCR1; HW::SPORT1->TCR1 = sp1TCR1; }
#define Start_SPORT0()	{ HW::SPORT0->TCR1 = sp0TCR1; }
#define Start_SPORT1()	{ HW::SPORT1->TCR1 = sp1TCR1; }

#define Stop_SPORT0()	{ HW::SPORT0->TCR1 = sp0TCR1 = 0; /*HW::SPORT0->RCR1 = 0;*/}
#define Stop_SPORT1()	{ HW::SPORT1->TCR1 = sp1TCR1 = 0; }

#define __TCR1			(DITFS|LATFS|LTFS|TFSR|ITFS|ITCLK|TSPEN)
#define __TCR2			(SLEN(12))
#define __TCLKDIV		(NS2SCLK(10)-1)
#define __TFSDIV_MIN	(NS2SCLK(150)/(__TCLKDIV+1)-1)

#define __RCR1			(/*RCKFE|LARFS|*/LRFS|RFSR|RSPEN)
#define __RCR2			(SLEN(12))

#define StartFire()		{ HW::TIMER->Enable = FIRE1_TIMEN|FIRE2_TIMEN; }
#define StopFire()		{ HW::TIMER->Disable = FIRE1_TIMEN|FIRE2_TIMEN; }
#define StartFire1()	{ HW::TIMER->Enable = FIRE1_TIMEN; }
#define StartFire2()	{ HW::TIMER->Enable = FIRE2_TIMEN; }

#define GAIN_SPI_MODE	CPHA	

#elif defined(CPU_BF706) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define Start_SPORT()	{/* HW::SPORT0->TCR1 = sp0TCR1; HW::SPORT1->TCR1 = sp1TCR1;*/ }
#define Start_SPORT0()	{ /*HW::SPORT0->TCR1 = sp0TCR1;*/ }
#define Start_SPORT1()	{ /*HW::SPORT1->TCR1 = sp1TCR1;*/ }

#define Stop_SPORT0()	{ /*HW::SPORT0->TCR1 = sp0TCR1 = 0;*/ }
#define Stop_SPORT1()	{ /*HW::SPORT1->TCR1 = sp1TCR1 = 0;*/ }

#define __TCR1			(DITFS|LATFS|LTFS|TFSR|ITFS|ITCLK|TSPEN)
#define __TCR2			(SLEN(12))
#define __TCLKDIV		(NS2SCLK(10)-1)
#define __TFSDIV_MIN	(NS2SCLK(150)/(__TCLKDIV+1)-1)

#define __RCR1			(/*RCKFE|LARFS|*/LRFS|RFSR|RSPEN)
#define __RCR2			(SLEN(12))

#define StartFire()		{ HW::TIMER->RUN_SET = FIRE1_TIMEN|FIRE2_TIMEN; }
#define StopFire()		{ HW::TIMER->RUN_CLR = FIRE1_TIMEN|FIRE2_TIMEN; }
#define StartFire1()	{ HW::TIMER->RUN_SET = FIRE1_TIMEN; }
#define StartFire2()	{ HW::TIMER->RUN_SET = FIRE2_TIMEN; }

#define GAIN_SPI_MODE	SPI_CPHA	

#define LowLevelInit()	{}

#endif //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define DisableSwArr()	{ PIO_RST_SW_ARR->CLR(BM_RST_SW_ARR); }
#define EnableSwArr()	{ PIO_RST_SW_ARR->SET(BM_RST_SW_ARR); }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void LowLevelInit();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//byte bitGain[16] = {GAIN_M0, GAIN_M1, GAIN_M2, GAIN_M3, GAIN_M4, GAIN_M5, GAIN_M6, GAIN_M7, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8 };

static bool cmdGainUpdate = false;
static byte gainDataBuf[2];

static u16 GAIN_CS_MASK[] = { BM_MUX_SYNC };
static S_SPIM	spiGain(1, PIO_MUX_SYNC, GAIN_CS_MASK, ArraySize(GAIN_CS_MASK), SCLK, BM_MUX_SCK|BM_MUX_DIN);
																				  
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_BF592 //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DMA_CH	dmaRxSp0(SPORT0_RX_DMA);
DMA_CH	dmaRxSp1(SPORT1_RX_DMA);

#elif defined(CPU_BF706) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 sp0TCR1 = 0;
static u16 sp1TCR1 = 0;

static DSCSPORT *curDscSPORT0 = 0;
static DSCSPORT *curDscSPORT1 = 0;
//static DSCPPI *lastDscPPI = 0;

//static u16 ppi_buf[PPI_BUF_LEN][PPI_BUF_NUM];

static DSCSPORT sportdsc[SPORT_BUF_NUM];
//static u16 startIndPPI = 0;
//static u16 endIndPPI = 0;g118

//u16 ppiClkDiv = NS2CLK(400);
//u16 ppiLen = 16;

//static u16 ppiOffset = sizeof(RspHdrCM)/2; //19;

//u32 ppiDelay = US2CCLK(10);

u32 mmsec = 0; // 0.1 ms
static u32 prevFireRefTime = 0;


#pragma instantiate List<DSCSPORT>
static List<DSCSPORT> freeSPORT;
static List<DSCSPORT> readySPORT;

static ReqDsp01 dspVars;

u32 shaftCount = 0;
u32 shaftMMSEC = 0;
u32 shaftPrevMMSEC = 0;

u16 motoCount = 0;
u32 rotCount = 0;
u32 rotMMSEC = 0;
u32 rotDeltaMMSEC = 0;

u32 fireSyncCount = 0;
u32 firesPerRound = 16;

static SENS *curSens = &dspVars.sens[0];

struct PPI 
{
	u16 tfsdiv;
	u16 len;
	u32 delay;
	u16 gain;
	u16 sensType;
	u16 chMask;
	u16 st;
	u16 sd;
	u16 fireDiv;
	u16 freq;
};

static PPI sens1_PPI;
static PPI sens2_PPI;
static PPI refPPI;

u16 dstFireVoltage		= 250;
u16 dstFireVoltage10	= 2500;
u16 curFireVoltage = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetFireVoltage(u16 v)
{
	if (v <= 500) dstFireVoltage = v, dstFireVoltage10 = v*10;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16	GetFireVoltage()
{
	return curFireVoltage;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetGain(byte g1, byte g2, byte g3, byte g4) 
{
	gainDataBuf[0] = (g3&15) | ((g4&15)<<4);
	gainDataBuf[1] = (g1&15) | ((g2&15)<<4);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetMux(byte a) 
{
	//*pPORTGIO = (*pPORTGIO & ~A0) | ((a & 1) << PIN_A0);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetPPI(PPI &ppi, SENS &sens, u16 sensType, u16 chMask, bool forced)
{
	bool c = false;

	if (ppi.st != sens.st || forced)
	{
		ppi.st = (sens.st < NS2DSP(300)) ? NS2DSP(300) : sens.st;

		ppi.tfsdiv = (ppi.st * NS2CLK(20) + 2) / 4;

		if (ppi.tfsdiv > 0) ppi.tfsdiv -= 1;

		if (ppi.tfsdiv < __TFSDIV_MIN) ppi.tfsdiv = __TFSDIV_MIN;

		c = true;
	};

	ppi.len = sens.sl;

	if (ppi.len < 16) ppi.len = 16;
	if (ppi.len > WAVE_MAXLEN) ppi.len = WAVE_MAXLEN;

	if (c || ppi.sd != sens.sd)
	{
		ppi.sd = sens.sd;

		if (ppi.sd != 0)
		{
			//u32 d = (u32)ppi.sd + ppi.st/2;
	
			ppi.delay = (ppi.sd + ppi.st/2) / ppi.st;
		}
		else
		{
			ppi.delay = 0;
		};
	};

	ppi.gain = sens.gain;
	ppi.sensType = sensType;
	ppi.chMask = chMask;

	if (chMask == 0)
	{
		ppi.freq = ~0;
		ppi.fireDiv = 0;
	}
	else if (ppi.freq != sens.freq || forced)
	{
		ppi.freq = sens.freq;

		if (sens.freq > 900)
		{
			ppi.fireDiv = sens.freq - 900;
		}
		else if (sens.freq > 0)
		{
			ppi.fireDiv = (US2CLK(500) + sens.freq/2) / sens.freq;
		}
		else
		{
			ppi.fireDiv = US2CLK(1);
		};

		if (ppi.fireDiv == 0) { ppi.fireDiv = 1; };
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetDspVars(const ReqDsp01 *v, bool forced)
{
	dspVars = *v;

	SetPPI(sens1_PPI,	dspVars.sens[0], 0, v->sensMask&1,		forced); 
	SetPPI(sens2_PPI,	dspVars.sens[1], 1, (v->sensMask>>1)&1, forced); 
	SetPPI(refPPI,		dspVars.sens[2], 2, 2,					forced);
	
	firesPerRound = (dspVars.mode == 0) ? dspVars.wavesPerRoundCM : dspVars.wavesPerRoundIM;

	SetGain(sens1_PPI.gain,sens1_PPI.gain, refPPI.gain, sens2_PPI.gain);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_BF592 //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(RTT_ISR)
{
	if (*pTIMER_STATUS & TIMIL2)
	{
		*pTIMER_STATUS = TIMIL2; 

		//*pPORTGIO_TOGGLE = 1<<6;

		mmsec++;

		ssync();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitRTT()
{
	*pTIMER2_CONFIG = PERIOD_CNT|PWM_OUT|OUT_DIS|IRQ_ENA;
	*pTIMER2_PERIOD = US2CLK(100);

	InitIVG(IVG_GPTIMER2_RTT, PID_GP_Timer_2, RTT_ISR);

	*pTIMER_ENABLE = TIMEN2;
}

#elif defined(CPU_BF706) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(RTT_ISR)
{
	mmsec++;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitRTT()
{
	HW::TMR->CNTL	= TMR_PWR;
	HW::TMR->PERIOD = US2CCLK(100);
	HW::TMR->SCALE	= 0;

	InitIVG(IVG_CORETIMER, RTT_ISR);

	HW::TMR->CNTL	= TMR_AUTORLD|TMR_EN|TMR_PWR;
}

#endif //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DSCSPORT* GetDscSPORT()
{
	return readySPORT.Get();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DSCSPORT* AllocDscSPORT()
{
	return freeSPORT.Get();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeDscSPORT(DSCSPORT* dsc)
{
	freeSPORT.Add(dsc);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Read_SPORT0(PPI &ppi)
{
	Stop_SPORT0();

	if (ppi.chMask == 0) return;

	curDscSPORT0 = AllocDscSPORT();

	if (curDscSPORT0 != 0)
	{
		#ifdef CPU_BF592

			curDscSPORT0->busy = false;
			curDscSPORT0->sportDelay = ppi.delay;
			curDscSPORT0->sampleDelay = ppi.delay*ppi.st;
			curDscSPORT0->sampleTime = ppi.st;
			curDscSPORT0->sensType = ppi.sensType;
			curDscSPORT0->gain = ppi.gain;
			curDscSPORT0->len = ppi.len;
			curDscSPORT0->chMask = ppi.chMask;

			FIRE1_TIMER->Width = ppi.fireDiv; 

			dmaRxSp0.Disable();

			u16 n = ((ppi.chMask&2)>>1)+1;

			HW::SPORT0->RCR2 = __RCR2|((ppi.chMask&2)<<7);
			HW::SPORT0->RCR1 = __RCR1;
			sp0TCR1 = __TCR1;
			HW::SPORT0->TCR2 = __TCR2;
			HW::SPORT0->TCLKDIV = __TCLKDIV; 
			HW::SPORT0->TFSDIV = curDscSPORT0->sport_tfsdiv = ppi.tfsdiv; //14;

			if (ppi.delay != 0)
			{
				dmaRxSp0.Read16(curDscSPORT0->data, ppi.delay*n, (ppi.len + WAVE_OVRLEN)*n);
			}
			else
			{
				dmaRxSp0.Read16(curDscSPORT0->data, (ppi.len + WAVE_OVRLEN)*n); 
			};

		#elif defined(CPU_BF706)

		#endif	
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Read_SPORT1(PPI &ppi)
{
	Stop_SPORT1();

	if (ppi.chMask == 0) return;

	curDscSPORT1 = AllocDscSPORT();

	if (curDscSPORT1 != 0)
	{
		#ifdef CPU_BF592

			curDscSPORT1->busy = false;
			curDscSPORT1->sportDelay = ppi.delay;
			curDscSPORT1->sampleDelay = ppi.delay*ppi.st;
			curDscSPORT1->sampleTime = ppi.st;
			curDscSPORT1->sensType = ppi.sensType;
			curDscSPORT1->gain = ppi.gain;
			curDscSPORT1->len = ppi.len;
			curDscSPORT1->chMask = ppi.chMask;

			FIRE2_TIMER->Width = ppi.fireDiv;

			dmaRxSp1.Disable();

			u16 n = ((ppi.chMask&2)>>1)+1;

			HW::SPORT1->RCR2 = __RCR2|((ppi.chMask&2)<<7);
			HW::SPORT1->RCR1 = __RCR1;
			sp1TCR1 = __TCR1;
			HW::SPORT1->TCR2 = __TCR2;
			HW::SPORT1->TCLKDIV = __TCLKDIV; 
			HW::SPORT1->TFSDIV = curDscSPORT1->sport_tfsdiv = ppi.tfsdiv;

			if (ppi.delay != 0)
			{
				dmaRxSp1.Read16(curDscSPORT1->data, ppi.delay*n, (ppi.len + WAVE_OVRLEN)*n);
			}
			else
			{
				dmaRxSp1.Read16(curDscSPORT1->data, (ppi.len + WAVE_OVRLEN)*n); 
			};

		#elif defined(CPU_BF706)

		#endif	
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void Fire()
{
	if (expected_true(curDscSPORT0 != 0))
	{
		if (!curDscSPORT0->busy)
		{
			curDscSPORT0->busy = true;

			curDscSPORT0->fireIndex = fireSyncCount;

			curDscSPORT0->mmsec			= mmsec;
			curDscSPORT0->shaftTime		= shaftMMSEC;
			curDscSPORT0->rotCount		= rotCount;
			curDscSPORT0->motoCount		= motoCount; //dspVars.motoCount;
			curDscSPORT0->shaftCount	= shaftCount;

			curDscSPORT0->ax = dspVars.ax;
			curDscSPORT0->ay = dspVars.ay;
			curDscSPORT0->az = dspVars.az;
			curDscSPORT0->at = dspVars.at;
		};
	}
	else
	{
		Read_SPORT0(sens1_PPI);
	};

	if (expected_true(curDscSPORT1 != 0))
	{
		if (!curDscSPORT1->busy)
		{
			curDscSPORT1->busy = true;

			curDscSPORT1->fireIndex = fireSyncCount;

			curDscSPORT1->mmsec			= mmsec;
			curDscSPORT1->shaftTime		= shaftMMSEC;
			curDscSPORT1->rotCount		= rotCount;
			curDscSPORT1->motoCount		= motoCount; //dspVars.motoCount;
			curDscSPORT1->shaftCount	= shaftCount;

			curDscSPORT1->ax = dspVars.ax;
			curDscSPORT1->ay = dspVars.ay;
			curDscSPORT1->az = dspVars.az;
			curDscSPORT1->at = dspVars.at;
		};
	}
	else
	{
		Read_SPORT1(sens2_PPI);
	};
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void FireRef()
{
	if (expected_true(curDscSPORT1 != 0))
	{
		if (!curDscSPORT1->busy)
		{
			curDscSPORT1->busy = true;

			curDscSPORT1->fireIndex = fireSyncCount;

			curDscSPORT1->mmsec			= mmsec;
			curDscSPORT1->shaftTime		= shaftMMSEC;
			curDscSPORT1->rotCount		= rotCount;
			curDscSPORT1->motoCount		= motoCount; //dspVars.motoCount;
			curDscSPORT1->shaftCount	= shaftCount;

			curDscSPORT1->ax = dspVars.ax;
			curDscSPORT1->ay = dspVars.ay;
			curDscSPORT1->az = dspVars.az;
			curDscSPORT1->at = dspVars.at;
		};
	};
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

#ifdef CPU_BF592

EX_INTERRUPT_HANDLER(SPORT0_ISR)
{
	Pin_SPORT0_ISR_Set();

	if (dmaRxSp0.CheckComplete()/* &&  dmaSp1.CheckComplete()*/)
	{
		Stop_SPORT0();

		dmaRxSp0.Disable(); //dmaSp1.Disable();

		HW::SPORT0->RCR1 = 0;
		FIRE1_TIMER->Width = 1;

		curDscSPORT0->busy = false;
		readySPORT.Add(curDscSPORT0);
		curDscSPORT0 = 0;

		if ((dspVars.sensMask & 2) == 0)
		{
			u32 t = mmsec;

			if ((t - prevFireRefTime) >= 30011)
			{
				prevFireRefTime = t;

				Read_SPORT1(refPPI);
				
				StartFire2(); // Start Fire Pulse
				Start_SPORT1();
				FireRef();
			};
		};

		Read_SPORT0(sens1_PPI);

		//ssync();
	};

	Pin_SPORT0_ISR_Clr();
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SPORT1_ISR)
{
	Pin_SPORT1_ISR_Set();

	if (dmaRxSp1.CheckComplete()/* &&  dmaSp1.CheckComplete()*/)
	{
		Stop_SPORT1();

		dmaRxSp1.Disable();

		HW::SPORT1->RCR1 = 0;
		FIRE2_TIMER->Width = 1;

		curDscSPORT1->busy = false;
		readySPORT.Add(curDscSPORT1);
		curDscSPORT1 = 0;

		u32 t = mmsec;

		if ((t - prevFireRefTime) >= 30011)
		{
			prevFireRefTime = t;

			Read_SPORT1(refPPI);
			
			StartFire2(); // Start Fire Pulse
			Start_SPORT1();
			FireRef();
		}
		else
		{
			Read_SPORT1(sens2_PPI);
		};

		//ssync();
	};

	Pin_SPORT1_ISR_Clr();	
}

#elif defined(CPU_BF706)

#endif	

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//EX_INTERRUPT_HANDLER(TIMER_PPI_ISR)
//{
//	Start_SPORT();
//
//	*pTCNTL = 0;
//
//	//ssync();
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SYNC_ISR)
{
	StartFire(); // Start Fire Pulse
	Start_SPORT();

#ifdef CPU_BF592
	PIO_SYNC->ClearTriggerIRQ(BM_SYNC);
#elif defined(CPU_BF706)

#endif	

	Fire();

	fireSyncCount += 1;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//EX_INTERRUPT_HANDLER(FIRE_ISR)
//{
//	if (*pTIMER_STATUS & TIMIL0)
//	{
//		*pTIMER_STATUS = TIMIL0; 
//
//		ssync();
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitFire()
{
	for (u16 i = 0; i < ArraySize(sportdsc); i++)
	{
		DSCSPORT &dsc = sportdsc[i];

		dsc.busy = false;

		freeSPORT.Add(&dsc);
	};

	#ifdef CPU_BF592

		PIO_FIRE->SetFER(BM_FIRE1|BM_FIRE2);
		PIO_FIRE->ClrMUX(BM_FIRE1|BM_FIRE2);

		PIO_RST_SW_ARR->ClrFER(BM_RST_SW_ARR);
		PIO_RST_SW_ARR->DirSet(BM_RST_SW_ARR);
		PIO_RST_SW_ARR->CLR(BM_RST_SW_ARR);

		StopFire();
	
		FIRE1_TIMER->Config = EMU_RUN|PWM_OUT|PULSE_HI;
		FIRE1_TIMER->Period = ~0;
		FIRE1_TIMER->Width = NS2SCLK(100);

		FIRE2_TIMER->Config = EMU_RUN|PWM_OUT|PULSE_HI;
		FIRE2_TIMER->Period = ~0;
		FIRE2_TIMER->Width = NS2SCLK(200);

		HW::PIOF->SetFER(PF0|PF1|PF2|PF6|PF7);
		HW::PIOF->ClrMUX(PF0|PF1|PF2|PF6|PF7);
		HW::PIOG->SetFER(PG0|PG1|PG2|PG6|PG7);
		HW::PIOG->ClrMUX(PG0|PG1|PG2|PG6|PG7);

		dmaRxSp0.Disable();
		dmaRxSp1.Disable();

		HW::SPORT0->RCR1 = 0;
		HW::SPORT1->RCR1 = 0;

		*pSPORT_GATECLK = SPORT0_GATECLK_EN|SPORT1_GATECLK_EN/*|SPORT0_GATECLK_STATE|SPORT1_GATECLK_STATE*/;

		HW::SPORT0->RCR2 = __RCR2;
		HW::SPORT1->RCR2 = __RCR2;
		HW::SPORT0->RCR1 = __RCR1;
		HW::SPORT1->RCR1 = __RCR1;

		InitIVG(IVG_SPORT0_DMA, PID_DMA1_SPORT0_RX, SPORT0_ISR);
		InitIVG(IVG_SPORT1_DMA, PID_DMA3_SPORT1_RX, SPORT1_ISR);

		InitIVG(IVG_PORTF_SYNC, PID_Port_F_Interrupt_A, SYNC_ISR);

		PIO_SYNC->ClrMaskA(~BM_SYNC);
		PIO_SYNC->EnableIRQA_Rise(BM_SYNC);

	#elif defined(CPU_BF706)

	#endif	

	SetPPI(sens1_PPI,	dspVars.sens[0], 0, 1, true);
	SetPPI(sens2_PPI,	dspVars.sens[1], 1, 1, true);
	SetPPI(refPPI,		dspVars.sens[2], 2, 2, true);

	EnableSwArr();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SHAFT_ISR)
{
	#ifdef CPU_BF592
		//PIO_DSHAFT->ClearTriggerIRQ(BM_DSHAFT);
	#elif defined(CPU_BF706)

	#endif	

	shaftCount++;

	shaftPrevMMSEC = shaftMMSEC;
	
	shaftMMSEC = mmsec;

	fireSyncCount = 0;

	if ((mmsec - rotMMSEC) <= (rotDeltaMMSEC/4)) rotCount = 0;
	
	//ssync();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitShaft()
{
	#ifdef CPU_BF592

		InitIVG(IVG_PORTF_SHAFT, PID_Port_F_Interrupt_B, SHAFT_ISR);

		PIO_DSHAFT->ClrMaskB(~BM_DSHAFT);

	#elif defined(CPU_BF706)

	#endif	
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(ROT_ISR)
{
//	*pPORTGIO_SET = 1<<6;

	#ifdef CPU_BF592
		//PIO_ROT->ClearTriggerIRQ(BM_ROT);
	#elif defined(CPU_BF706)

	#endif	

	motoCount++;

	rotCount++;

	if (rotCount >= 147) rotCount = 0;

	rotDeltaMMSEC = mmsec - rotMMSEC;
	
	rotMMSEC = mmsec;

//	*pPORTGIO_CLEAR = 1<<6;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitRot()
{
	#ifdef CPU_BF592
		InitIVG(IVG_PORTG_ROT, PID_Port_G_Interrupt_A, ROT_ISR);

		PIO_ROT->EnableIRQA_Rise(BM_ROT);
		PIO_ROT->ClrMaskA(~BM_ROT);
	#elif defined(CPU_BF706)

	#endif	

	//*pPORTG_MUX &= ~BM_ROT;
	//*pPORTGIO_DIR &= ~BM_ROT;
	//*pPORTGIO_INEN |= BM_ROT;
	//*pPORTGIO_EDGE |= BM_ROT;
	//*pPORTGIO_BOTH |= BM_ROT;
	//*pPORTGIO_CLEAR = BM_ROT;
	//*pPORTGIO_MASKA = BM_ROT;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitGain()
{
	spiGain.Connect(1000000);

	PIO_MUX_RESET->DirSet(BM_MUX_RESET);
	PIO_MUX_RESET->CLR(BM_MUX_RESET);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateGain()
{
	static CTM32 tm;
	static DSCSPI dsc;

	if (tm.Check(MS2CTM(201)) || cmdGainUpdate)
	{
		cmdGainUpdate = false;

		PIO_MUX_RESET->SET(BM_MUX_RESET);

		dsc.alen = 0;
		dsc.baud = NS2SCLK(250);
		dsc.csnum = 0;
		dsc.mode = GAIN_SPI_MODE;	// CPOL CPHA
		dsc.wdata = gainDataBuf;
		dsc.wlen = sizeof(gainDataBuf);
		dsc.rdata = 0;
		dsc.rlen = 0;

		spiGain.AddRequest(&dsc);
	};

	spiGain.Update();

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef __ADSPBF70x__

static volatile u16 twiWriteCount = 0;
static volatile u16 twiReadCount = 0;
static volatile byte * volatile twiWriteData = 0;
static volatile byte * volatile twiReadData;
static volatile DSCI2C* volatile twi_dsc = 0;
static volatile DSCI2C* volatile twi_lastDsc = 0;

//#pragma optimize_for_speed

SEC_INTERRUPT_HANDLER(TWI_ISR)
//void TWI_ISR()
{
	u16 stat = HW::TWI->ISTAT;
	
	HW::PIOB->DATA_SET = PB4;
	
	if (stat & TWI_RXSERV)
	{
		if (twiReadCount > 0)
		{
			*twiReadData++ = HW::TWI->RXDATA8;
			twiReadCount--;
		};

		if (twiReadCount == 0)
		{

			HW::TWI->MSTRCTL |= TWI_MST_STOP;
			HW::TWI->FIFOCTL  = TWI_TXFLUSH|TWI_RXFLUSH;
		};
	};

	if (stat & TWI_TXSERV)
	{
		if (twiWriteCount == 0 && twi_dsc->wlen2 != 0)
		{
			twiWriteData = (byte*)twi_dsc->wdata2;
			twiWriteCount = twi_dsc->wlen2;
			twi_dsc->wlen2 = 0;
		};

		if (twiWriteCount > 0)
		{
			HW::TWI->TXDATA8 = *twiWriteData++;
			twiWriteCount--;

		};
	};

	if (stat & (TWI_MCOMP|TWI_MERR))
	{
		twi_dsc->ack = ((stat & TWI_MERR) == 0);

		if (twi_dsc->ack && twiReadCount > 0)
		{
			HW::TWI->IMSK		= TWI_RXSERV|TWI_MCOMP|TWI_MERR;
			HW::TWI->MSTRCTL	= TWI_MST_DCNT(twiReadCount)|TWI_MST_DIR|TWI_MST_FAST|TWI_MST_EN;
		}
		else
		{
			twi_dsc->ready = true;
			twi_dsc->readedLen = twi_dsc->rlen - twiReadCount;
			//twi_dsc->master_stat = *pTWI_MASTER_STAT;

			DSCI2C *ndsc = twi_dsc->next;

			if (ndsc != 0)
			{
				twi_dsc->next = 0;
				twi_dsc = ndsc;

				twi_dsc->ready = false;
				twi_dsc->ack = false;
				twi_dsc->readedLen = 0;

				if (twi_dsc->wdata2 == 0) twi_dsc->wlen2 = 0;

				twiWriteData = (byte*)twi_dsc->wdata;
				twiWriteCount = twi_dsc->wlen;
				twiReadData = (byte*)twi_dsc->rdata;
				twiReadCount = twi_dsc->rlen;

				u16 len = twiWriteCount + twi_dsc->wlen2;

				HW::TWI->MSTRSTAT	= ~0;
				HW::TWI->FIFOCTL	= 0;

				HW::TWI->MSTRADDR = twi_dsc->adr;

				if (len != 0)
				{
					HW::TWI->TXDATA8	= *twiWriteData++; twiWriteCount--;
					HW::TWI->IMSK		= TWI_TXSERV|TWI_MCOMP|TWI_MERR;
					HW::TWI->MSTRCTL	= TWI_MST_DCNT(len)|TWI_MST_FAST|TWI_MST_EN;
				}
				else
				{
					HW::TWI->IMSK		= TWI_RXSERV|TWI_MCOMP|TWI_MERR;
					HW::TWI->MSTRCTL	= TWI_MST_DCNT(twiReadCount)|TWI_MST_DIR|TWI_MST_FAST|TWI_MST_EN;
				};
			}
			else
			{
				HW::TWI->MSTRCTL	= 0;
				HW::TWI->MSTRSTAT	= ~0;
				HW::TWI->FIFOCTL	= TWI_TXFLUSH|TWI_RXFLUSH;
				HW::TWI->IMSK		= 0;

				twi_lastDsc = twi_dsc = 0;
			};

		};
	};

	HW::TWI->ISTAT = stat;

	HW::PIOB->DATA_CLR = PB4;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void I2C_Init()
{
	u32 sclk_mhz = Get_SCLK0_MHz();

	HW::TWI->CTL		= TWI_CTL_EN | TWI_CTL_PRESCALE(sclk_mhz/10);
	HW::TWI->CLKDIV		= TWI_CLKHI(10)|TWI_CLKLO(12);
	HW::TWI->IMSK		= 0;
	HW::TWI->MSTRADDR	= 0;

	InitSEC(PID_TWI0_DATA, TWI_ISR);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool TWI_Write(DSCI2C *d)
{
	//	using namespace HW;

	if (twi_dsc != 0 || d == 0) { return false; };
	if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

	twi_dsc = d;

	twi_dsc->ready = false;
	twi_dsc->ack = false;
	twi_dsc->readedLen = 0;

	if (twi_dsc->wdata2 == 0) twi_dsc->wlen2 = 0;

	u32 t = cli();

	HW::TWI->MSTRCTL	= 0;
	HW::TWI->MSTRSTAT	= ~0;
	HW::TWI->FIFOCTL	= 0;//XMTINTLEN|RCVINTLEN;

	twiWriteData = (byte*)twi_dsc->wdata;
	twiWriteCount = twi_dsc->wlen;
	twiReadData = (byte*)twi_dsc->rdata;
	twiReadCount = twi_dsc->rlen;

	u16 len = twiWriteCount + twi_dsc->wlen2;

	HW::TWI->MSTRADDR = twi_dsc->adr;
	HW::TWI->FIFOCTL = 0;

	if (len != 0)
	{
		HW::TWI->TXDATA8 = *twiWriteData++; twiWriteCount--;
		HW::TWI->IMSK = TWI_TXSERV|TWI_MERR|TWI_MCOMP;
		HW::TWI->MSTRCTL = TWI_MST_DCNT(len)|TWI_MST_FAST|TWI_MST_EN;
	}
	else
	{
		HW::TWI->IMSK = TWI_RXSERV|TWI_MERR|TWI_MCOMP;
		HW::TWI->MSTRCTL = TWI_MST_DCNT(twiReadCount)|TWI_MST_DIR|TWI_MST_FAST|TWI_MST_EN;
	};

	sti(t);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_AddRequest(DSCI2C *d)
{
	if (d == 0) { return false; };
	if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

	d->next = 0;
	d->ready = false;

	if (d->wdata2 == 0) d->wlen2 = 0;

	u32 t = cli();

	if (twi_lastDsc == 0)
	{
		twi_lastDsc = d;

		sti(t);

		return TWI_Write(d);
	}
	else
	{
		twi_lastDsc->next = d;
		twi_lastDsc = d;

		sti(t);
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma optimize_as_cmd_line

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update_ADC_DAC()
{
	static byte i = 0;
	static DSCI2C dsc;
	static byte wbuf[4];
	static byte rbuf[4];
	static CTM32 tm;
	static CTM32 ctm;
	static i32 filtFV = 0;
	static u16 correction = 0x200;
	static u16 dstFV = 0;
	static byte adr = 0x28;

	if (!ctm.Check(US2CTM(50))) return;

	HW::ResetWDT();

	switch (i)
	{
		case 0:

			if (tm.Check(MS2CTM(10)))
			{
				wbuf[0] = 0x60;	

				dsc.adr = adr;
				dsc.wdata = wbuf;
				dsc.wlen = 1;
				dsc.rdata = rbuf;
				dsc.rlen = 4;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 1:

			if (dsc.ready)
			{
				if (dsc.ack)
				{
					byte *p = rbuf;

					for (u32 i = dsc.readedLen; i > 0; i -= 2, p += 2)
					{
						byte ch = (p[0] >> 4) & 3;

						i32 res = ((p[0]<<8)|p[1]) & 0xFFF;

						if (ch == 1)
						{
							filtFV += (res * 160 - filtFV + 8) / 16;

							curFireVoltage = (filtFV * 674 + 32768) / 65536; //51869

							u16 t = dstFireVoltage10;

							if (t > curFireVoltage)
							{
								if (correction < 0x3FF)
								{
									correction += 1;
								};
							}
							else if (t < curFireVoltage)
							{
								if (correction > 0)
								{
									correction -= 1;
								};
							};
						};
					};
				}
				else
				{
					adr ^= 1;
				};

				i++;
			};

			break;

		case 2:

			if (dsc.ready)
			{
				wbuf[0] = 2;
				wbuf[1] = 0;
				wbuf[2] = 0;

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 3:

			if (dsc.ready)
			{
				wbuf[0] = 3;	
				wbuf[1] = 1;	
				wbuf[2] = 0;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 4:

			if (dsc.ready)
			{
				wbuf[0] = 4;	
				wbuf[1] = 1;	
				wbuf[2] = 1;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 5:

			if (dsc.ready)
			{
				wbuf[0] = 5;	
				wbuf[1] = 0;	
				wbuf[2] = 0;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 6:

			if (dsc.ready)
			{
				dstFV += (i16)dstFireVoltage - (dstFV+4)/8;

				u16 t = (dstFV+4)/8+10;

				u32 k = (0x1E00 + correction) >> 3;

				t = (k*t+128) >> 10;

				if (t > 500) t = 500;

				t = ~(((u32)t * (65535*16384/500)) / 16384); 

				wbuf[0] = 8;	
				wbuf[1] = t>>8;
				wbuf[2] = t;

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = rbuf;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 7:

			if (dsc.ready)
			{
				// write ADG2128	sens1

				wbuf[0] = 0x80|(9<<3)|3; // X7 to Y3 on; SIG_1 -> CH2 -> SPORT0 PRI	
				wbuf[1] = 1;	

				dsc.adr = 0x70;
				dsc.wdata = wbuf;
				dsc.wlen = 2;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 8:

			if (dsc.ready)
			{
				// write ADG2128	sens2

				wbuf[0] = 0x80|(5<<3)|1; // X5 to Y1 on; SIG_3 -> CH4 -> SPORT1 PRI	
				wbuf[1] = 1;	

				dsc.adr = 0x70;
				dsc.wdata = wbuf;
				dsc.wlen = 2;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 9:

			if (dsc.ready)
			{
				// write ADG2128	refSens

				wbuf[0] = 0x80|(4<<3)|0; // X4 to Y0 on; SIG_4 -> CH3 -> SPORT1 SEC
				wbuf[1] = 1;	

				dsc.adr = 0x70;
				dsc.wdata = wbuf;
				dsc.wlen = 2;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 10:

			if (dsc.ready)
			{
				i = 0;
			};

			break;


	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
	LowLevelInit();

	InitRTT();

//	InitPPI();

#ifdef CPU_BF592
	I2C_Init(SCLK_MHz, IVG_TWI, PID_TWI);
#elif defined(CPU_BF706)
	I2C_Init();
#endif	

	InitFire();

	InitShaft();

	//InitRot();

	InitGain();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( Update_ADC_DAC()	);
		CALL( UpdateGain()		);
	};

	i &= 1; // i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma diag(pop)
