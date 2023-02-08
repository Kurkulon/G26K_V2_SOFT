#include "hardware.h"

#include <bfrom.h>
#include <sys\exception.h>
//#include <cdefBF592-A.h>
//#include <ccblkfn.h>

#include "list.h"
#include "DMA.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <system_imp.h>
#include <twi_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PPI_BUF_NUM 8


//#define Start_SPORT01()	{ HW::SPORT0->TCR1 = DITFS|LATFS|LTFS|TFSR|ITFS|ITCLK|TSPEN; HW::SPORT1->TCR1 = DITFS|LATFS|LTFS|TFSR|ITFS|ITCLK|TSPEN; }
//#define Stop_SPORT01()	{ HW::SPORT0->TCR1 = 0; HW::SPORT1->TCR1 = 0; }

#define Start_SPORT0()	{/* HW::SPORT0->RCR1 |= RSPEN;*/ HW::SPORT0->TCR1 = DITFS|LATFS|LTFS|TFSR|ITFS|ITCLK|TSPEN; }
#define Start_SPORT1()	{ HW::SPORT1->TCR1 = DITFS|LATFS|LTFS|TFSR|ITFS|ITCLK|TSPEN; }

#define Stop_SPORT0()	{ HW::SPORT0->TCR1 = 0; /*HW::SPORT0->RCR1 = 0;*/}
#define Stop_SPORT1()	{ HW::SPORT1->TCR1 = 0; }

#define StartFire()	{ HW::TIMER->Enable = FIRE1_TIMEN|FIRE2_TIMEN; }
#define StopFire()	{ HW::TIMER->Disable = FIRE1_TIMEN|FIRE2_TIMEN; }

#define DisableSwArr()	{ PIO_RST_SW_ARR->CLR(BM_RST_SW_ARR); }
#define EnableSwArr()	{ PIO_RST_SW_ARR->SET(BM_RST_SW_ARR); }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void LowLevelInit();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//byte bitGain[16] = {GAIN_M0, GAIN_M1, GAIN_M2, GAIN_M3, GAIN_M4, GAIN_M5, GAIN_M6, GAIN_M7, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8 };

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DMA_CH	dmaRxSp0(SPORT0_RX_DMA);
DMA_CH	dmaRxSp1(SPORT1_RX_DMA);

static DSCPPI *curDscPPI0 = 0;
static DSCPPI *curDscPPI1 = 0;
//static DSCPPI *lastDscPPI = 0;

//static u16 ppi_buf[PPI_BUF_LEN][PPI_BUF_NUM];

static DSCPPI ppidsc[PPI_BUF_NUM];
//static u16 startIndPPI = 0;
//static u16 endIndPPI = 0;g118

//u16 ppiClkDiv = NS2CLK(400);
//u16 ppiLen = 16;

u16 ppiOffset = sizeof(RspCM)/2; //19;

//u32 ppiDelay = US2CCLK(10);

u32 mmsec = 0; // 0.1 ms

#pragma instantiate List<DSCPPI>
static List<DSCPPI> freePPI;
static List<DSCPPI> readyPPI;

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

static SENS *curSens = &dspVars.mainSens;

struct PPI 
{
	u16 clkDiv;
	u16 len;
	u32 delay;
	u16 gain;
	u16 sensType;
	u16 st;
	u16 sd;
	u16 fireDiv;
};

static PPI mainPPI;
static PPI refPPI;

u16 dstFireVoltage = 250;
u16 curFireVoltage = 0;
u16 curMotoVoltage = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetFireVoltage(u16 v)
{
	if (v <= 500) dstFireVoltage = v;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16	GetFireVoltage()
{
	return curFireVoltage;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16	GetMotoVoltage()
{
	return curMotoVoltage;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetGain(byte v) 
{
	//*pPORTGIO = (*pPORTGIO & ~0xF) | bitGain[v&0xF];
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetMux(byte a) 
{
	//*pPORTGIO = (*pPORTGIO & ~A0) | ((a & 1) << PIN_A0);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetPPI(PPI &ppi, SENS &sens, u16 sensType)
{
	ppi.st = (sens.st > 0) ? sens.st : 1;

	ppi.clkDiv = ppi.st * NS2CLK(50);

	//if (ppi.clkDiv == 0) ppi.clkDiv = 1;

	ppi.len = sens.sl;

	if (ppi.len < 16) ppi.len = 16;

	ppi.sd = sens.sd;

	i32 d = (i32)ppi.sd + (i32)ppi.st/2;

	if (d < 0) d = 0;

	ppi.delay = d * (NS2CCLK(50));
	
	if (ppi.delay > US2CCLK(1000)) ppi.delay = US2CCLK(1000);

	ppi.gain = sens.gain;
	ppi.sensType = sensType;

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
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetDspVars(const ReqDsp01 *v)
{
	dspVars = *v;

	SetPPI(mainPPI, dspVars.mainSens, 0); 

	SetPPI(refPPI, dspVars.refSens, 1);
	
	firesPerRound = (dspVars.mode == 0) ? dspVars.wavesPerRoundCM : dspVars.wavesPerRoundIM;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DSCPPI* GetDscPPI()
{
	return readyPPI.Get();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DSCPPI* AllocDscPPI()
{
	return freePPI.Get();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeDscPPI(DSCPPI* dsc)
{
	freePPI.Add(dsc);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Read_SPORT0(PPI &ppi)
{
	curDscPPI0 = AllocDscPPI();

	Stop_SPORT0();

	if (curDscPPI0 != 0)
	{
		curDscPPI0->busy = false;
		curDscPPI0->ppidelay = ppi.delay;
		curDscPPI0->sampleDelay = ppi.sd;
		curDscPPI0->sampleTime = ppi.st;
		curDscPPI0->sensType = ppi.sensType;
		curDscPPI0->gain = ppi.gain;
		curDscPPI0->len = ppi.len;

		dmaRxSp0.Disable();

		HW::SPORT0->TCR1 = 0;
		HW::SPORT0->TCR2 = SLEN(12);
		HW::SPORT0->TCLKDIV = 1; curDscPPI0->ppiclkdiv = ppi.clkDiv;
		HW::SPORT0->TFSDIV = 14;

		dmaRxSp0.Read16(curDscPPI0->data+(curDscPPI0->offset = ppiOffset), ppi.len + 32); 

		//ssync();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Read_SPORT1(PPI &ppi)
{
	curDscPPI1 = AllocDscPPI();

	Stop_SPORT1();

	if (curDscPPI1 != 0)
	{
		curDscPPI1->busy = false;
		curDscPPI1->ppidelay = ppi.delay;
		curDscPPI1->sampleDelay = ppi.sd;
		curDscPPI1->sampleTime = ppi.st;
		curDscPPI1->sensType = ppi.sensType;
		curDscPPI1->gain = ppi.gain;
		curDscPPI1->len = ppi.len;

		dmaRxSp1.Disable();

		HW::SPORT1->TCR1 = 0;
		HW::SPORT1->TCR2 = SLEN(13);
		HW::SPORT1->TCLKDIV = 3; curDscPPI1->ppiclkdiv = ppi.clkDiv;
		HW::SPORT1->TFSDIV = 16;

		dmaRxSp0.Read16(curDscPPI1->data+(curDscPPI1->offset = ppiOffset), ppi.len + 32); 

		//ssync();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void Fire()
{
	StartFire(); // Start Fire Pulse

	if (expected_true(curDscPPI0 != 0))
	{
		if (!curDscPPI0->busy)
		{
			curDscPPI0->busy = true;

			if (curDscPPI0->ppidelay == 0)
			{ 
				*pTCNTL = 0;
				Start_SPORT0();
			}
			else
			{
				*pTSCALE = 0;
				*pTCOUNT = curDscPPI0->ppidelay;
				*pTCNTL = TINT|TMPWR|TMREN;
			};

			curDscPPI0->fireIndex = fireSyncCount;

			curDscPPI0->mmsec = mmsec;
			curDscPPI0->shaftTime = shaftMMSEC;
			curDscPPI0->shaftPrev = shaftPrevMMSEC;

			curDscPPI0->rotCount = rotCount;
			curDscPPI0->rotMMSEC = rotMMSEC;

			curDscPPI0->motoCount = motoCount; //dspVars.motoCount;
			curDscPPI0->shaftCount = shaftCount;

			curDscPPI0->ax = dspVars.ax;
			curDscPPI0->ay = dspVars.ay;
			curDscPPI0->az = dspVars.az;
			curDscPPI0->at = dspVars.at;
		};
	}
	else
	{
		Read_SPORT0(mainPPI);
	};
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

EX_INTERRUPT_HANDLER(SPORT0_ISR)
{
	static u32 pt = 0;

	HW::PIOG->BSET(5);

	if (dmaRxSp0.CheckComplete()/* &&  dmaSp1.CheckComplete()*/)
	{
		Stop_SPORT0();

		dmaRxSp0.Disable(); //dmaSp1.Disable();

		curDscPPI0->busy = false;
		readyPPI.Add(curDscPPI0);
		curDscPPI0 = 0;

		//u32 t = mmsec;

		//if ((t - pt) >= 30011)
		//{
		//	pt = t;

		//	Read_SPORT0(refPPI);
		//	
		//	Fire();
		//}
		//else
		{
			Read_SPORT0(mainPPI);
		};

		//ssync();
	};

	HW::PIOG->BCLR(5);
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SPORT1_ISR)
{
	static u32 pt = 0;

	HW::PIOG->BSET(5);

	if (dmaRxSp1.CheckComplete()/* &&  dmaSp1.CheckComplete()*/)
	{
		Stop_SPORT1();

		dmaRxSp1.Disable();

		curDscPPI1->busy = false;
		readyPPI.Add(curDscPPI1);
		curDscPPI1 = 0;

		u32 t = mmsec;

		if ((t - pt) >= 30011)
		{
			pt = t;

			Read_SPORT1(refPPI);
			
			Fire();
		}
		else
		{
			Read_SPORT1(mainPPI);
		};

		//ssync();
	};

	HW::PIOG->BCLR(5);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(TIMER_PPI_ISR)
{
	Start_SPORT0();

	*pTCNTL = 0;

	//ssync();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SYNC_ISR)
{
	PIO_SYNC->ClearTriggerIRQ(BM_SYNC);

	//StartFire();

	Fire();

	fireSyncCount += 1;

	//ssync();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(FIRE_ISR)
{
	if (*pTIMER_STATUS & TIMIL0)
	{
		*pTIMER_STATUS = TIMIL0; 

		ssync();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitFire()
{
	for (u16 i = 0; i < ArraySize(ppidsc); i++)
	{
		DSCPPI &dsc = ppidsc[i];

		dsc.busy = false;

		freePPI.Add(&dsc);
	};

	PIO_FIRE->SetFER(BM_FIRE1|BM_FIRE2);
	PIO_FIRE->ClrMUX(BM_FIRE1|BM_FIRE2);

	PIO_RST_SW_ARR->ClrFER(BM_RST_SW_ARR);
	PIO_RST_SW_ARR->DirSet(BM_RST_SW_ARR);
	PIO_RST_SW_ARR->CLR(BM_RST_SW_ARR);

	//HW::PIOF->Dir |= PF9;

	// PPI clk

	StopFire();
	
	FIRE1_TIMER->Config = EMU_RUN|PWM_OUT|PULSE_HI;
	FIRE1_TIMER->Period = ~0;
	FIRE1_TIMER->Width = NS2SCLK(100);

	FIRE2_TIMER->Config = EMU_RUN|PWM_OUT|PULSE_HI;
	FIRE2_TIMER->Period = ~0;
	FIRE2_TIMER->Width = NS2SCLK(200);

	HW::PIOF->SetFER(PF0|PF1|PF6|PF7);
	HW::PIOF->ClrMUX(PF0|PF1|PF6|PF7);
	HW::PIOG->SetFER(PG0|PG1|PG2|PG6|PG7);
	HW::PIOG->ClrMUX(PG0|PG1|PG2|PG6|PG7);

	dmaRxSp0.Disable();
	dmaRxSp1.Disable();

	HW::SPORT0->RCR1 = 0;
	HW::SPORT1->RCR1 = 0;

	*pSPORT_GATECLK = SPORT0_GATECLK_EN|SPORT1_GATECLK_EN/*|SPORT0_GATECLK_STATE|SPORT1_GATECLK_STATE*/;

	HW::SPORT0->RCR2 = SLEN(12);
	HW::SPORT0->RCR1 = /*RCKFE|LARFS|*/LRFS|RFSR|RSPEN;
	HW::SPORT1->RCR2 = SLEN(12);
	HW::SPORT1->RCR1 = LARFS|LRFS|RFSR|RSPEN;


	//HW::SIC->IntAssign(PID_DMA2_SPORT0_TX, IVG_SPORT_DMA);

	//HW::SIC->IntAssign(PID_DMA1_SPORT0_RX, IVG_SPORT_DMA);
	//HW::EIC->InitIVG(IVG_SPORT_DMA, SPORT_ISR);

	InitIVG(IVG_SPORT0_DMA, PID_DMA1_SPORT0_RX, SPORT0_ISR);
	//InitIVG(IVG_SPORT_DMA, PID_DMA3_SPORT1_RX, SPORT1_ISR);

	InitIVG(IVG_PORTF_SYNC, PID_Port_F_Interrupt_A, SYNC_ISR);

	PIO_SYNC->ClrMaskA(~BM_SYNC);
	PIO_SYNC->EnableIRQA_Rise(BM_SYNC);

	SetPPI(mainPPI, dspVars.mainSens, 0);
	SetPPI(refPPI, dspVars.refSens, 1);

	//ReadPPI(mainPPI);

	//InitIVG(IVG_GPTIMER0_FIRE, PID_GP_Timer_0, FIRE_PPI_ISR);

	
	InitIVG(IVG_CORETIMER, 0, TIMER_PPI_ISR);

	EnableSwArr();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SHAFT_ISR)
{
	PIO_DSHAFT->ClearTriggerIRQ(BM_DSHAFT);

	shaftCount++;

	shaftPrevMMSEC = shaftMMSEC;
	
	shaftMMSEC = mmsec;

	fireSyncCount = 0;

	if ((mmsec - rotMMSEC) <= (rotDeltaMMSEC/4)) rotCount = 0;
	
	ssync();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitShaft()
{
	InitIVG(IVG_PORTF_SHAFT, PID_Port_F_Interrupt_B, SHAFT_ISR);

	PIO_DSHAFT->EnableIRQB_Rise(BM_DSHAFT);

	//PIO_DSHAFT->Inen |= BM_DSHAFT;
	//PIO_DSHAFT->Edge |= BM_DSHAFT;
	//PIO_DSHAFT->Polar |= BM_DSHAFT;	// falling edge
	//PIO_DSHAFT->Both &= ~BM_DSHAFT;
	//PIO_DSHAFT->CLR(BM_DSHAFT);
	//PIO_DSHAFT->MaskB = BM_DSHAFT;

	PIO_DSHAFT->ClrMaskB(~BM_DSHAFT);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(ROT_ISR)
{
	*pPORTGIO_SET = 1<<6;

	PIO_ROT->ClearTriggerIRQ(BM_ROT);

	motoCount++;

	rotCount++;

	if (rotCount >= 147) rotCount = 0;

	rotDeltaMMSEC = mmsec - rotMMSEC;
	
	rotMMSEC = mmsec;

	*pPORTGIO_CLEAR = 1<<6;

	ssync();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitRot()
{
	InitIVG(IVG_PORTG_ROT, PID_Port_G_Interrupt_A, ROT_ISR);

	PIO_ROT->EnableIRQA_Rise(BM_ROT);
	PIO_ROT->ClrMaskA(~BM_ROT);

	//*pPORTG_MUX &= ~BM_ROT;
	//*pPORTGIO_DIR &= ~BM_ROT;
	//*pPORTGIO_INEN |= BM_ROT;
	//*pPORTGIO_EDGE |= BM_ROT;
	//*pPORTGIO_BOTH |= BM_ROT;
	//*pPORTGIO_CLEAR = BM_ROT;
	//*pPORTGIO_MASKA = BM_ROT;
}

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

	InitTWI();

	InitFire();

	InitShaft();

	InitRot();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
	static byte i = 0;
	static DSCTWI dsc;
	static byte wbuf[4];
	static byte rbuf[4];
	static RTM32 tm;
	static RTM32 ctm;
	static i32 filtFV = 0;
	static i32 filtMV = 0;
	static u16 correction = 0x200;
	static u16 dstFV = 0;

	if (!ctm.Check(US2RT(50))) return;

	HW::ResetWDT();

	switch (i)
	{
		case 0:

			if (tm.Check(MS2RT(10)))
			{
				wbuf[0] = 0x60;	

				dsc.adr = 0x28;
				dsc.wdata = wbuf;
				dsc.wlen = 1;
				dsc.rdata = rbuf;
				dsc.rlen = 4;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				TWI_AddRequest(&dsc);

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
							filtFV += (res * 16 - filtFV + 8) / 16;

							curFireVoltage = (filtFV * 674 + 32768) / 65536; //51869

							u16 t = dstFireVoltage;

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
						}
						else if (ch == 2)
						{
							filtMV += (res * 16 - filtMV + 8) / 16;

							curMotoVoltage = (filtMV * 105 + 32768) / 65536; //51869
						};

					};
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

				TWI_AddRequest(&dsc);

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

				TWI_AddRequest(&dsc);

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

				TWI_AddRequest(&dsc);

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

				TWI_AddRequest(&dsc);

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

				TWI_AddRequest(&dsc);

				i++;
			};

			break;

		case 7:

			if (dsc.ready)
			{
				wbuf[0] = 0x80|(9<<3)|3;	
				wbuf[1] = 1;	

				dsc.adr = 0x70;
				dsc.wdata = wbuf;
				dsc.wlen = 2;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				TWI_AddRequest(&dsc);

				i++;
			};

			break;

		case 8:

			if (dsc.ready)
			{
				i = 0;
			};

			break;


	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
