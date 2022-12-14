#include "hardware.h"
#include <time.h>


//#include "ComPort.h"

//#define OPEN_VALVE_CUR 600
//#define CLOSE_VALVE_CUR 600

#define GEAR_RATIO	12.25
#define CUR_LIM		2000
#define CUR_MAX		3000
#define CUR_MIN		100
#define IMP_CUR_LIM	13000
#define POWER_LIM	30000


const u16 pulsesPerHeadRoundFix4 = GEAR_RATIO * 6 * 16;

u16 curADC = 0;
u16 avrCurADC = 0;
u32 fcurADC = 0;
u16 vAP = 0;
u32 fvAP = 0;
u32 tachoCount = 0;
u32 motoCounter = 0;
u32 targetRPM = 0;
u32 tachoLim = 0;
u32 tachoStep = 1;
u32 tachoStamp = 0;

u16 limCur = CUR_LIM;
u16 maxCur = CUR_MAX;

u32 rpmCounter = 0;
u32 rpmPrevTime = 0;
u32 rpmCount = 0;
u32 rpmTime = 0;
u16 rpm = 0;

//u16 prevT1 = 0;
//u16 prevT2 = 0;
//u16 prevR1 = 0;
//u16 prevR2 = 0;

i32 shaftPos = 0;
//u16 closeCurADC = 0;
//u16 errCloseCount = 0;
//u16 errOpenCount = 0;

byte motorState = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#define UL PIN_LIN1	// 20 
#define VL PIN_LIN2	// 21
#define WL PIN_LIN3	// 22
#define UH PIN_HIN1	// 17
#define VH PIN_HIN2	// 18
#define WH PIN_HIN3	// 19
#define NN 0xFF

#define NC 0x3F
#define UU ((~(LIN1>>17))&NC)	// 0x37
#define VV ((~(LIN2>>17))&NC)	// 0x2F
#define WW ((~(LIN3>>17))&NC)	// 0x1F

//#define UT 0x3E
//#define VT 0x3D
//#define WT 0x3B

// D	HHH	WVUWVU  
// R	WVU	LLLHHH 

// 1	000	111111  
// 1	001 P10P11  1  
// 1	010	10P11P  5
// 1	011	P01P11  6
// 1	100	0P11P1  3
// 1	101	1P01P1  2
// 1	110	01P11P  4
// 1	111	111111  

// 0	000	111111  
// 0	001	01P11P  5
// 0	010	1P01P1  1
// 0	011	0P11P1  6
// 0	100	P01P11  3 
// 0	101	10P11P  4
// 0	110	P10P11  2
// 0	111	111111  



byte t = 0;
byte s = 0;

bool dir = true;

// dir 0
// 1 UH WW
// 3 VH WW
// 2 VH UU
// 6 WH UU
// 4 WH VV
// 5 UH VV

// dir 1
// 1 WH UU
// 3 WH VV
// 2 UH VV
// 6 UH WW
// 4 VH WW
// 5 VH UU

// F/R 1


//                             1   2   3   4   5   6                1   2   3   4   5   6
const byte states[16] =		{ WW, WW, UU, WW, VV, VV, UU, UU,		WW, UU, VV, VV, WW, UU, WW, VV };

const byte LG_pin[16] =		{ UL, UL, VL, VL, WL, UL, WL, VL,		VL, WL, UL, WL, VL, VL, UL, WL };
const byte HG_pin[16] =		{ UH, UH, VH, VH, WH, UH, WH, VH,		VH, WH, UH, WH, VH, VH, UH, WH };

const u16 pwmPeriod = 1250;
const u16 maxDuty = 1200;
static u16 limDuty = maxDuty;
static u16 curDuty = 0;

static u32 impCur = 0; // mA

static u32 tachoPLL = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// HHH 
// WVU  UV W
//     00001111 EW
//     00110011 EV
//     01010101 EU
// 000 00000000
// 001 00+0-000         
// 010 0-00+000        
// 011 0+-00000          
// 100 0+-00000             
// 101 0-00+000             
// 110 00+0-000            
// 111 00000000


static i8 tachoEncoder[8][8] = {
	{0,0,0,0,0,0,0,0},
	{0,0,1,0,-1,0,0,0},
	{0,-1,0,0,1,0,0,0},
	{0,1,-1,0,0,0,0,0},
	{0,1,-1,0,0,0,0,0},
	{0,-1,0,0,1,0,0,0},
	{0,0,1,0,-1,0,0,0},
	{0,0,0,0,0,0,0,0}
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void EnableDriver() 
{ 
	HW::GPIO->CLR(ENABLE); 
	HW::MRT->Channel[3].CTRL = 1;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void DisableDriver() 
{ 
	HW::GPIO->SET(ENABLE);
	HW::MRT->Channel[3].CTRL = 0; 
	//pidOut = 0; 
	SetDutyPWM(0);
}
	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetTargetRPM(u32 v)
{ 
	if (targetRPM != v)
	{
		targetRPM = v;

		motorState = 1;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetLimCurrent(u16 v)
{ 
	limCur = (v < CUR_LIM) ? v : CUR_LIM;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetMaxCurrent(u16 v)
{ 
	maxCur = (v < CUR_MAX) ? v : CUR_MAX;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*static void UpdateADC()
{
	using namespace HW;

	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( curADC = ((ADC->DAT0&0xFFF0) * 9091) >> 16;  fcurADC += curADC - avrCurADC; avrCurADC = fcurADC >> 6;	);
		CALL( fvAP += (((ADC->DAT1&0xFFF0) * 3300) >> 16) - vAP; vAP = fvAP >> 3;	);
	};

//	i = (i > (__LINE__-S-3)) ? 0 : i;
	i &= 1;

	#undef CALL
}*/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitPWM()
{
	using namespace HW;

	SYSCON->SYSAHBCLKCTRL |= CLK::SCT_M;

	SCT->STATE_L = 0;
	SCT->REGMODE_L = 0;

	SCT->MATCHREL_L[0] = maxDuty; 
	SCT->MATCHREL_L[1] = pwmPeriod-49;
	SCT->MATCHREL_L[2] = pwmPeriod; 
	SCT->MATCH_L[3] = 0; 
	SCT->MATCH_L[4] = 0;

	SCT->OUT[0].SET = (1<<0);
	SCT->OUT[0].CLR = (1<<1)|(1<<2);

	SCT->OUT[1].SET = (1<<1)|(1<<2);
	SCT->OUT[1].CLR = (1<<0);

	SCT->EVENT[0].STATE = 1;
	SCT->EVENT[0].CTRL = (1<<5)|(0<<6)|(1<<12)|0;

	SCT->EVENT[1].STATE = 1;
	SCT->EVENT[1].CTRL = (1<<5)|(0<<6)|(1<<12)|1;

	SCT->EVENT[2].STATE = 1;
	SCT->EVENT[2].CTRL = (1<<5)|(0<<6)|(1<<12)|2;

	SCT->EVENT[3].STATE = 0;
	SCT->EVENT[3].CTRL = 0;

	SCT->EVENT[4].STATE = 0;
	SCT->EVENT[4].CTRL = 0;

	SCT->EVENT[5].STATE = 0;
	SCT->EVENT[5].CTRL = 0;

	SCT->START_L = 0;
	SCT->STOP_L = 0;
	SCT->HALT_L = 0;
	SCT->LIMIT_L = (1<<2);

	SCT->CONFIG = 0; 

	//SWM->CTOUT_0 = 20;
	//SWM->CTOUT_1 = 17;

	SCT->CTRL_L = (1<<3);

	SetDutyPWM(0);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetDutyPWM(u16 v)
{
	if (v > limDuty) v = limDuty;
	
	HW::SCT->MATCHREL_L[0] = maxDuty - (curDuty = (v < maxDuty) ? v : maxDuty);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitADC()
{
	using namespace HW;

	//SWM->PINASSIGN[3] = (SWM->PINASSIGN[3] & 0x00FFFFFF) | 0x09000000;
	//SWM->PINASSIGN[4] = (SWM->PINASSIGN[4] & 0xFF000000) | 0x00100FFF;

	SWM->PINENABLE0.B.ADC_0 = 0;
	SWM->PINENABLE0.B.ADC_1 = 0;


	SYSCON->PDRUNCFG &= ~(1<<4);
	SYSCON->SYSAHBCLKCTRL |= CLK::ADC_M;

	ADC->CTRL = (1<<30)|4;

	while(ADC->CTRL & (1<<30));

	ADC->CTRL = 24;
	ADC->SEQA_CTRL = 3|(1UL<<31)|(1<<27);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void TahoHandler()
{
	byte ist = HW::PIN_INT->IST & 7;

	if (ist)
	{
		t = ((HW::GPIO->PIN0 >> 8) & 7 | (dir<<3)) & 0xF;

		s = states[t];

		HW::GPIO->MASK0 = ~(0x3F << 17);
		HW::GPIO->MPIN0 = (u32)s << 17;

		HW::SWM->CTOUT_0 = LG_pin[t];
		HW::SWM->CTOUT_1 = HG_pin[t];

		shaftPos += tachoEncoder[t & 7][ist];

		HW::PIN_INT->IST = ist;

		tachoCount++;
		motoCounter++;

		if (tachoPLL > tachoStep) { tachoPLL -= tachoStep; } else { tachoPLL = 0; };

		if (tachoCount >= tachoLim)
		{
			tachoCount = tachoLim;

			SetDutyPWM(tachoPLL);
		};

		rpmCounter++;

		u32 tm = tachoStamp = GetMilliseconds();
		u32 dt = tm - rpmPrevTime;

		if (dt >= 1000)
		{
			rpmPrevTime = tm;
			rpmCount = rpmCounter;
			rpmTime = dt;
			rpmCounter = 0;
		};

		HW::GPIO->NOT0 = 1<<15;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMotor()
{
	static u16 pt = 0;
	//static u16 pt2 = 0;
	//static u32 prtacho = 0;
	//static u16 dt = 100;
	//static i32 pshp = 0;

	//static byte n = 0;

	static TM32 tm;//, tmRPM;

	curADC = ((HW::ADC->DAT0&0xFFF0) * 9400) >> 16;  

	if (curADC > 110) curADC -= 110; else curADC = 0;

	if (curADC > maxCur)
	{	
		DisableDriver();
		motorState = 6;
	};

	if ((u16)(GetMillisecondsLow() - pt) >= 1)
	{
		pt = GetMillisecondsLow();

		HW::ResetWDT();
	
		fcurADC += curADC - avrCurADC; avrCurADC = fcurADC >> 8;

		if (targetRPM == 0)
		{
			motorState = 0;
		};

		impCur = (u32)avrCurADC * pwmPeriod / (curDuty+10);

		//power = avrCurADC * voltage;

		if (avrCurADC > (limCur+100) || impCur > (IMP_CUR_LIM+1000))
		{
			if (limDuty > 2) limDuty -= 2; else limDuty = 0;

			//tachoCount = 0;
			//tachoStep = 1;
			//tachoPLL >>= 1;

			//motorState = 3;
		}
		else if ((avrCurADC < CUR_MIN) || ((avrCurADC < limCur) && (impCur < IMP_CUR_LIM)))
		{
			if (limDuty < maxDuty) limDuty += 1;

			//tmOvrCrnt.Reset();
		};

		if (rpmCount != 0)
		{
			rpm = rpmCount * 16667 / rpmTime;
			
			rpmCount = 0;
		}
		else if ((GetMilliseconds() - rpmPrevTime) > 1200)
		{
			rpm = 0;
		};

		switch (motorState)
		{
			case 0:		// Idle;

				HW::MRT->Channel[3].CTRL = 0;
				tachoPLL = 0;
				tachoCount = 0;
				DisableDriver();

				dir = false;

				tm.Reset();

				SetDutyPWM(tachoPLL = maxDuty/8);

				break;

			case 1: // ?????

				{
					__disable_irq();

					u32 v = targetRPM;

					tachoStep = 1;

					tachoLim = v + 100;

					tachoCount = 0;

					v *= pulsesPerHeadRoundFix4;
					v /= 16;

					if (v > 0)
					{
						HW::MRT->Channel[3].INTVAL = (((u32)MCK * 100 + v/2) / v)|(1UL<<31);
						//HW::MRT->Channel[3].CTRL = 1;
						EnableDriver();

						motorState = (!dir) ? 2 : 4;
					}
					else
					{
						motorState = 0;
					};

					__enable_irq();

					tm.Reset();

					tachoStamp = GetMilliseconds();
				};

				break;

			case 2: 

				if (tm.Check(500))
				{
					dir = true;

					tm.Reset();

					DisableDriver();

					motorState++;
				};

				break;

			case 3: 

				if (tm.Check(1000))
				{	
					tachoCount = 0;
					tachoStep = 1;

					EnableDriver();

					SetDutyPWM(tachoPLL = maxDuty/8);

					tachoStamp = GetMilliseconds();

					motorState++;
				};

				break;

			case 4: // ??????

				if ((GetMilliseconds()-tachoStamp) > 2000 || avrCurADC > maxCur)
				{	
					motorState = 6;
				}
				else if (tachoCount >= tachoLim)
				{
					tachoCount = tachoLim;
					tachoStep = 64;

					tm.Reset();
					//tmOvrCrnt.Reset();

					motorState++;
				}
				else
				{
					SetDutyPWM(tachoPLL);
				};

				break;

			case 5: // ???????????? ????????

				if ((GetMilliseconds()-tachoStamp) > 1000 || avrCurADC > maxCur)
				{	
					motorState++;
				};

				break;

			case 6:

				tm.Reset();

				HW::MRT->Channel[3].CTRL = 0;
				tachoPLL = 0;
				tachoCount = 0;
				DisableDriver();

				dir = false;

				motorState++;

				break;

			case 7:

				if (tm.Check(2000))
				{
					motorState = 1;
				};
	 
				break;

			case 8:

				break;

			case 9:

				break;

			case 10:

				break;
		};

	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitTaho()
{
	using namespace HW;

	IOCON->PIO0_8.B.HYS = 1;
	IOCON->PIO0_9.B.HYS = 1;

	SYSCON->PINTSEL[0] = PIN_DHU;
	SYSCON->PINTSEL[1] = PIN_DHV;
	SYSCON->PINTSEL[2] = PIN_DHW;
	PIN_INT->ISEL &= ~7;

	PIN_INT->IENR |= 7;

	PIN_INT->IENF |= 7;//((HW::GPIO->PIN0 >> 8)) & 7;

	VectorTableExt[PIN_INT0_IRQ] = TahoHandler;
	VectorTableExt[PIN_INT1_IRQ] = TahoHandler;
	VectorTableExt[PIN_INT2_IRQ] = TahoHandler;
	CM0::NVIC->ISER[0] = 7<<PIN_INT0_IRQ;

	GPIO->SET0 = (0x3F<<17);

//	GPIO->MASK0 = ~(7 << 20);

//	GPIO->MPIN0 = 0xFF;

	t = ((HW::GPIO->PIN0 >> PIN_DHU) & 7 | (dir<<3)) & 0xF;

	s = states[t];

	HW::GPIO->MASK0 = ~(0x3F << 17);
	HW::GPIO->MPIN0 = (u32)s << 17;

	HW::SWM->CTOUT_0 = LG_pin[t];
	HW::SWM->CTOUT_1 = HG_pin[t];


}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__irq void ROT_Handler()
{
	if (HW::PIN_INT->IST & 8)
	{
		if (curDuty <= limDuty)
		{
			if (tachoPLL < (u32)maxDuty)
			{ 
				tachoPLL += tachoStep; 
			};
		};

		if (tachoCount >= tachoLim)
		{
			tachoCount = tachoLim;

			SetDutyPWM(tachoPLL);
		};

		HW::PIN_INT->IST = 8;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__irq void MRT_Handler()
{
	if (HW::MRT->IRQ_FLAG & 8)
	{
		//HW::GPIO->BTGL(15);

		if (curDuty <= limDuty)
		{
			if (tachoPLL < (u32)maxDuty)
			{ 
				tachoPLL += tachoStep; 
			};
		};

		if (tachoCount >= tachoLim)
		{
			tachoCount = tachoLim;

			SetDutyPWM(tachoPLL);
		};
	};

	HW::MRT->IRQ_FLAG = 8;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitRotMRT()
{
	using namespace HW;

	VectorTableExt[MRT_IRQ] = MRT_Handler;
	CM0::NVIC->ICPR[0] = 1 << MRT_IRQ;
	CM0::NVIC->ISER[0] = 1 << MRT_IRQ;
	HW::MRT->Channel[3].CTRL = 0;
	HW::MRT->Channel[3].INTVAL = (MCK/(20*6))|(1UL<<31);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
	using namespace HW;

	Init_time(MCK);
	InitADC();
	InitPWM();
	InitTaho();

//	StopMotor();

//	com.Connect(0, 921600, 0);

	//InitRot();

	InitRotMRT();

	SYSCON->SYSAHBCLKCTRL |= HW::CLK::WWDT_M;
	SYSCON->PDRUNCFG &= ~(1<<6); // WDTOSC_PD = 0
	SYSCON->WDTOSCCTRL = (1<<5)|59; // 600kHz/60 = 10kHz = 0.1ms

#ifndef _DEBUG

	WDT->TC = 250; // * 0.4ms
	WDT->MOD = 0x3;
	ResetWDT();

#endif

	//EnableDriver();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
	UpdateMotor();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <system_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

