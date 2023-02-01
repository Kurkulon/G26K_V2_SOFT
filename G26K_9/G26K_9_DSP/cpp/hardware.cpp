#include "hardware.h"

#include <bfrom.h>
#include <sys\exception.h>
//#include <cdefBF592-A.h>
//#include <ccblkfn.h>

#include "list.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <system_imp.h>
#include <twi_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PPI_BUF_NUM 8


#define StartPPI()	{ *pTIMER_ENABLE = TIMEN1;  }
#define StopPPI()	{ *pTIMER_DISABLE = TIMDIS1; }

#define StartFire()	{ *pTIMER_ENABLE = TIMEN0; }
//#define StopFire()	{ *pTIMER_DISABLE = TIMDIS0; }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void LowLevelInit();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

byte bitGain[16] = {GAIN_M0, GAIN_M1, GAIN_M2, GAIN_M3, GAIN_M4, GAIN_M5, GAIN_M6, GAIN_M7, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8 };

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static DSCPPI *curDscPPI = 0;
static DSCPPI *lastDscPPI = 0;

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

void SetGain(byte v) 
{
	*pPORTGIO = (*pPORTGIO & ~0xF) | bitGain[v&0xF];
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetMux(byte a) 
{
	*pPORTGIO = (*pPORTGIO & ~A0) | ((a & 1) << PIN_A0);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetPPI(PPI &ppi, SENS &sens, u16 sensType)
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

static void ReadPPI(PPI &ppi)
{
	curDscPPI = AllocDscPPI();

	StopPPI();

	if (curDscPPI != 0)
	{
		curDscPPI->busy = false;
		curDscPPI->ppidelay = ppi.delay;
		curDscPPI->sampleDelay = ppi.sd;
		curDscPPI->sampleTime = ppi.st;

		SetMux(curDscPPI->sensType = ppi.sensType);
		SetGain(curDscPPI->gain = ppi.gain);

		//*pTIMER0_CONFIG = PWM_OUT|PULSE_HI;
		//*pTIMER0_PERIOD = ppi.fireDiv*2;
		*pTIMER0_WIDTH = ppi.fireDiv;

		*pTIMER1_CONFIG = PERIOD_CNT|PWM_OUT|PULSE_HI;
		*pTIMER1_PERIOD = curDscPPI->ppiclkdiv = ppi.clkDiv;
		*pTIMER1_WIDTH = curDscPPI->ppiclkdiv>>1;

		*pDMA0_START_ADDR = curDscPPI->data+(curDscPPI->offset = ppiOffset);
		*pDMA0_X_COUNT = ppi.len + 32; curDscPPI->len = ppi.len;
		*pDMA0_X_MODIFY = 2;

		*pPPI_COUNT = 0;//*pDMA0_X_COUNT - 1;
		*pPPI_DELAY = 0;
		*pDMA0_CONFIG = FLOW_STOP|DI_EN|WDSIZE_16|SYNC|WNR|DMAEN;
		*pPPI_CONTROL = FLD_SEL|PORT_CFG|DLEN_12|XFR_TYPE|PORT_EN;
		
		ssync();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Fire()
{
	StartFire(); // Start Fire Pulse

	if (curDscPPI != 0)
	{
		if (!curDscPPI->busy)
		{
			curDscPPI->busy = true;

			if (curDscPPI->ppidelay == 0)
			{ 
				*pTCNTL = 0;
				StartPPI();
			}
			else
			{
				*pTSCALE = 0;
				*pTCOUNT = curDscPPI->ppidelay;
				*pTCNTL = TINT|TMPWR|TMREN;
			};

			curDscPPI->fireIndex = fireSyncCount;

			curDscPPI->mmsec = mmsec;
			curDscPPI->shaftTime = shaftMMSEC;
			curDscPPI->shaftPrev = shaftPrevMMSEC;

			curDscPPI->rotCount = rotCount;
			curDscPPI->rotMMSEC = rotMMSEC;

			curDscPPI->motoCount = motoCount; //dspVars.motoCount;
			curDscPPI->shaftCount = shaftCount;

			curDscPPI->ax = dspVars.ax;
			curDscPPI->ay = dspVars.ay;
			curDscPPI->az = dspVars.az;
			curDscPPI->at = dspVars.at;
		};
	}
	else
	{
		ReadPPI(mainPPI);
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(PPI_ISR)
{
	static u32 pt = 0;

	if (*pDMA0_IRQ_STATUS & (DMA_DONE|DMA_ERR))
	{
		*pDMA0_IRQ_STATUS = DMA_DONE|DMA_ERR;
		*pPPI_CONTROL = 0;
		*pDMA0_CONFIG = 0;

		StopPPI();

		curDscPPI->busy = false;
		readyPPI.Add(curDscPPI);

		u32 t = mmsec;

		if ((t - pt) >= 30011)
		{
			pt = t;

			ReadPPI(refPPI);
			
			Fire();
		}
		else
		{
			ReadPPI(mainPPI);
		};

		ssync();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(TIMER_PPI_ISR)
{
	StartPPI();
	*pTCNTL = 0;

	ssync();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SYNC_ISR)
{
	*pPORTFIO_CLEAR = BM_SYNC;

	Fire();

	fireSyncCount += 1;

	ssync();
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

	*pPORTF_FER |= PF9;
	*pPORTF_MUX &= ~PF9;

	// PPI clk

	*pTIMER_DISABLE = TIMDIS1;
	*pTIMER1_CONFIG = PERIOD_CNT|PWM_OUT;
	*pTIMER1_PERIOD = 5;
	*pTIMER1_WIDTH = 2;

	*pPPI_CONTROL = 0;
	*pDMA0_CONFIG = 0;

	InitIVG(IVG_PPI_DMA0, PID_DMA0_PPI, PPI_ISR);

	InitIVG(IVG_PORTF_SYNC, PID_Port_F_Interrupt_A, SYNC_ISR);

	*pPORTFIO_INEN |= BM_SYNC;
	*pPORTFIO_EDGE |= BM_SYNC;
	*pPORTFIO_BOTH &= ~BM_SYNC;
	*pPORTFIO_CLEAR = BM_SYNC;
	*pPORTFIO_MASKA = BM_SYNC;

	SetPPI(mainPPI, dspVars.mainSens, 0);
	SetPPI(refPPI, dspVars.refSens, 1);

	ReadPPI(mainPPI);

	//InitIVG(IVG_GPTIMER0_FIRE, PID_GP_Timer_0, FIRE_PPI_ISR);

	*pTIMER0_CONFIG = PWM_OUT|PULSE_HI/*|IRQ_ENA*/;
	*pTIMER0_PERIOD = ~0;//MS2CLK(1000) / 500 / 4;
	*pTIMER0_WIDTH = US2CLK(1);
	*pTIMER_ENABLE = TIMEN0; 
	//InitIVG(IVG_FIRE, PID_GP_Timer_0, FIRE_ISR);
	
	InitIVG(IVG_CORETIMER, 0, TIMER_PPI_ISR);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SHAFT_ISR)
{
	*pPORTFIO_CLEAR = BM_SHAFT;

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

	*pPORTFIO_INEN |= BM_SHAFT;
	*pPORTFIO_EDGE |= BM_SHAFT;
	*pPORTFIO_POLAR |= BM_SHAFT;	// falling edge
	*pPORTFIO_BOTH &= ~BM_SHAFT;
	*pPORTFIO_CLEAR = BM_SHAFT;
	*pPORTFIO_MASKB = BM_SHAFT;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(ROT_ISR)
{
	*pPORTGIO_SET = 1<<6;

	*pPORTGIO_CLEAR = BM_ROT;

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

	*pPORTG_MUX &= ~BM_ROT;
	*pPORTGIO_DIR &= ~BM_ROT;
	*pPORTGIO_INEN |= BM_ROT;
	*pPORTGIO_EDGE |= BM_ROT;
	*pPORTGIO_BOTH |= BM_ROT;
	*pPORTGIO_CLEAR = BM_ROT;
	*pPORTGIO_MASKA = BM_ROT;
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

	//InitRTT();

//	InitPPI();

	//InitTWI();

	//InitFire();

	//InitShaft();

	//InitRot();
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
				i = 0;
			};

			break;


	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
