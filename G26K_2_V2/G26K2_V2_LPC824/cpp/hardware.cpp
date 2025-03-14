#include "hardware.h"
#include <time.h>


//#include "ComPort.h"

//#define OPEN_VALVE_CUR 600
//#define CLOSE_VALVE_CUR 600

#define GEAR_RATIO			12.25
#define CUR_LIM				5000
#define CUR_MAX				6000
#define CUR_MIN				100
#define ABS_CUR_LIM			7000
#define ABS_CUR_MAX			8000
#define POWER_LIM			30000
#define VREG_MIN			200
#define VREG_MAX			600
#define RPM_VREG_K			(1 * 256/10)
#define RPM_VREG_MIN		480
#define DT_MOSFET			NS2CLK(200)
//#define VAUX_MIN			500
#define VAUX_DEF			3000
#define VAUX_DELTA			50
#define FB90_MAX			700
#define FB90_VREG_DELTA		50
#define DELTA_DUTY_MAX		0x10000
#define DELTA_DUTY_MIN		0x1000
#define PWM_PERIOD			US2CLK(50)
#define MAX_DUTY			US2CLK(48)

const u16 pulsesPerHeadRoundFix4 = GEAR_RATIO * 6 * 16;

u16 curADC = 0;
u16 avrCurADC = 0;
u32 fcurADC = 0;

u16 lowCurADC = 0;
u16 avrLowCurADC = 0;
u32 fLowCurADC = 0;

//u32 tachoMRT = 0;
u32 tachoCount = 0;
u32 motoCounter = 0;
u32 targetRPM = 0;
u32 tachoLim = 0;
u32 tachoStep = 1;
u32 tachoStamp = 0;
bool forcedTargetRPM = false;

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
#define UU (((LIN1>>17))&NC)	// 0x37
#define VV (((LIN2>>17))&NC)	// 0x2F
#define WW (((LIN3>>17))&NC)	// 0x1F

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

const u16 pwmPeriod = PWM_PERIOD;
const u16 maxDuty	= MAX_DUTY;
//const u16 loDuty = maxDuty/4;
//const u16 hiDuty = maxDuty*3/4;

u16 limDuty = maxDuty;
u16 curDuty = 0;

u32 power = 0;
u32 lastMaxPower = 0;
u32 limPower = CUR_LIM * VREG_MAX;
u32 maxPower = CUR_MAX * VREG_MAX;

//u32 impCur = 0; // mA

u32 tachoPLL = 0;

const u16 periodPWM90	= US2CLK(25);
const u16 maxDutyPWM90	= US2CLK(23);

static void SetDutyPWM(u16 v);

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
	if (targetRPM != v && (!forcedTargetRPM || v != 0))
	{
		targetRPM = v;

		motorState = 1;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetForcedTargetRPM(u32 v)
{ 
	if (v != 0)
	{
		if (targetRPM < v)
		{
			targetRPM = v;
			motorState = 1;
		};

		forcedTargetRPM = true;
	}
	else
	{
		forcedTargetRPM = false;
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

	SCT->OUT[0].CLR = (1<<0);
	SCT->OUT[0].SET = (1<<1)|(1<<2);

	SCT->OUT[1].CLR = (1<<1)|(1<<2);
	SCT->OUT[1].SET = (1<<0);

	SCT->EVENT[0].STATE = 1;
	SCT->EVENT[0].CTRL = (1<<5)|(0<<6)|(1<<12)|0;

	SCT->EVENT[1].STATE = 1;
	SCT->EVENT[1].CTRL = (1<<5)|(0<<6)|(1<<12)|1;

	SCT->EVENT[2].STATE = 1;
	SCT->EVENT[2].CTRL = (1<<5)|(0<<6)|(1<<12)|2;

	SCT->START_L = 0;
	SCT->STOP_L = 0;
	SCT->HALT_L = 0;
	SCT->LIMIT_L = (1<<2);


	SCT->STATE_H = 0;
	SCT->REGMODE_H = 0;

	SCT->MATCHREL_H[0] = 0; 
	SCT->MATCHREL_H[1] = periodPWM90-1;
	SCT->MATCH_H[2] = 0;	  
	SCT->MATCH_H[3] = 0; 
	SCT->MATCH_H[4] = 0;

	SCT->OUT[2].CLR = (1<<3);
	SCT->OUT[2].SET = (1<<4);

	SCT->EVENT[3].STATE = 1;
	SCT->EVENT[3].CTRL = SCT_HEVENT|SCT_COMBMODE_MATCH|SCT_MATCHSEL(0);

	SCT->EVENT[4].STATE = 1;
	SCT->EVENT[4].CTRL = SCT_HEVENT|SCT_COMBMODE_MATCH|SCT_MATCHSEL(1);

	SCT->EVENT[5].STATE = 0;
	SCT->EVENT[5].CTRL = 0;

	SCT->EVENT[6].STATE = 0;
	SCT->EVENT[6].CTRL = 0;

	SCT->EVENT[7].STATE = 0;
	SCT->EVENT[7].CTRL = 0;

	SCT->START_H = 0;
	SCT->STOP_H = 0;
	SCT->HALT_H = 0;
	SCT->LIMIT_H = (1<<4);

	SCT->CONFIG = 0; 

	SWM->CTOUT_2 = PIN_PWM90;
	//SWM->CTOUT_1 = 17;

	SCT->CTRL_U = SCT_CLRCTR_L|SCT_CLRCTR_H;

	SetDutyPWM(0);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetDutyPWM(u16 v)
{
	if (v > limDuty) v = limDuty;
	
	HW::SCT->MATCHREL_L[0] = maxDuty - (curDuty = (v < maxDuty) ? v : maxDuty);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetDutyPWM90(u16 v)
{
	HW::SCT->MATCHREL_H[0] = (v < maxDutyPWM90) ? v : maxDutyPWM90;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetDutyPWM()
{
	return curDuty * (10000 / PWM_PERIOD);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitADC()
{
	using namespace HW;

	//SWM->PINASSIGN[3] = (SWM->PINASSIGN[3] & 0x00FFFFFF) | 0x09000000;
	//SWM->PINASSIGN[4] = (SWM->PINASSIGN[4] & 0xFF000000) | 0x00100FFF;

	SWM->PINENABLE0.B.ADC_0		= 0;	//ISEN
	SWM->PINENABLE0.B.ADC_1		= 0;	//ILOW
	SWM->PINENABLE0.B.ADC_3		= 0;	//FB_AUXPWR
	SWM->PINENABLE0.B.ADC_10	= 0;	//FB_90

	IOCON->PIO0_6.D		= 0;
	IOCON->PIO0_7.D		= 0;
	IOCON->PIO0_13.D	= 0;
	IOCON->PIO0_23.D	= 0;

	SYSCON->PDRUNCFG &= ~PDRUNCFG_ADC_PD;
	SYSCON->SYSAHBCLKCTRL |= CLK::ADC_M;

	ADC->CTRL = ADC_CTRL_CALMODE|ADC_CTRL_CLKDIV(4);

	while(ADC->CTRL & ADC_CTRL_CALMODE);

	ADC->CTRL = ADC_CTRL_CLKDIV(24);
	ADC->SEQA_CTRL = ADC_SEQ_CTRL_CHANNELS((1<<0)|(1<<1)|(1<<3)|(1<<10))|ADC_SEQ_CTRL_SEQ_ENA(1)|ADC_SEQ_CTRL_BURST(1);
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

		HW::GPIO->NOT0 = ROT;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union Fix16
{
protected:
	u32 d;
	u16 w[2];
public:
	Fix16() : d(0) {}

	operator u16() { return w[1]; }
	u16 operator=(u16 v) { w[1] = v; return v; }
	u16 operator+=(i32 v) { d += v; return w[1]; }
	u16 operator-=(i32 v) { d -= v; return w[1]; }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

i16 auxADC = 0;
i16 avrAuxADC = 0;
u32 fauxADC = 0;

i16 fb90ADC = 0;
i16 avrFB90ADC = 0;
u32 fFB90ADC = 0;

u16 targetVREG = 0;
u16 rpmVREG = 0;
Fix16 curVREG;

Fix16 curDuty90;
u16 dstDuty90 = 0;

i32 deltaDuty = 0;

u16 corrDuty90 = 0x100;
//u16 addDuty90 = NS2CLK(560);

//bool lockAuxLoop90 = false;

//byte stateVREG = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateVREG()
{
	static CTM32 tm;
	static CTM32 tm2;

	if (tm.Check(US2CTM(1001)))
	{
		auxADC	= ((HW::ADC->DAT3 &0xFFF0) * 3994) >> 16;  
		fb90ADC	= ((HW::ADC->DAT10&0xFFF0) * 1033) >> 16;  

		fauxADC	 += auxADC	- avrAuxADC;	avrAuxADC	= fauxADC >> 4;
		fFB90ADC += fb90ADC	- avrFB90ADC;	avrFB90ADC	= fFB90ADC >> 4;

		bool c = (fb90ADC > FB90_MAX);

		i16 v = /*(stateVREG == 0) ? VAUX_DEF :*/ auxADC;

		dstDuty90 = (periodPWM90 * curVREG + v/2) / v + DT_MOSFET;

		if (c)
		{
			curDuty90 = 0; corrDuty90 = 0x100;
			SetForcedTargetRPM(200);
		}
		else 
		{
			SetForcedTargetRPM(0);

			//switch (stateVREG)
			//{
			//	case 0: // Soft start

			//		if (avrAuxADC > (curVREG+VAUX_DELTA)) stateVREG = 1;

			//		break;

			//	case 1: // main

			//		if (avrAuxADC < curVREG && (avrFB90ADC < avrAuxADC || avrFB90ADC < VREG_MIN/2)) curDuty90 = 0, stateVREG = 0;

			//		break;
			//};

			i32 dv = curVREG - avrFB90ADC;
			if (dv < 0 ) dv = -dv;

			if (dv < 10)
			{
				dv /= 8;

				deltaDuty = (dv != 0) ? (((DELTA_DUTY_MAX-DELTA_DUTY_MIN)+dv/2)/dv+DELTA_DUTY_MIN) : DELTA_DUTY_MAX;
			}
			else
			{
				deltaDuty = DELTA_DUTY_MIN;
			};

			if (curDuty90 < dstDuty90)
			{
				curDuty90 += deltaDuty;
			}
			else if (curDuty90 > dstDuty90)
			{
				curDuty90 = dstDuty90; //curDuty90 -= deltaDuty;
			};
		};

		u16 duty = (curDuty90 * corrDuty90 + 128) >> 8;

		SetDutyPWM90(duty);

		if (tm2.Check(MS2CTM(10)))
		{
			if (curDuty90 == dstDuty90)
			{
				if (avrFB90ADC < ((i16)curVREG-20))
				{
					if (corrDuty90 < 0x140) corrDuty90 += 1;
				}
				else if (avrFB90ADC > ((i16)curVREG+20))
				{
					if (corrDuty90 > 0xC0) corrDuty90 -= 1;
				};
			};

			u16 tv = MAX(targetVREG, rpmVREG);
			
			tv = LIM(tv, VREG_MIN, VREG_MAX);

			if (tv > curVREG)
			{
				if ((avrFB90ADC+FB90_VREG_DELTA) > curVREG) curVREG += 0x8000;
			}
			else if (tv < curVREG)
			{
				curVREG = tv;
			};

			//if (corrDuty90 > 0x130) curVREG = 0, corrDuty90 = 0;
		};
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

	curADC = ((HW::ADC->DAT0&0xFFF0) * 16925) >> 16;  

	if (curADC > 37) curADC -= 37; else curADC = 0;

	if (curADC > ABS_CUR_MAX)
	{	
		DisableDriver();
		motorState = 6;
	};

	if ((u16)(GetMillisecondsLow() - pt) >= 1)
	{
		pt = GetMillisecondsLow();

		HW::ResetWDT();
	
		fcurADC += curADC - avrCurADC; avrCurADC = fcurADC >> 8;

		lowCurADC = ((HW::ADC->DAT1&0xFFF0) * 4950) >> 16; 
		if (lowCurADC > 40) lowCurADC -= 40; else lowCurADC = 0;

		fLowCurADC += lowCurADC - avrLowCurADC; avrLowCurADC = fLowCurADC >> 8;
	
		if (targetRPM == 0)
		{
			motorState = 0;
		};

		//impCur = (u32)avrCurADC * pwmPeriod / (curDuty+10);

		power = avrCurADC * avrFB90ADC;
		limPower = limCur * VREG_MAX;
		maxPower = maxCur * VREG_MAX;

		if (avrCurADC > (ABS_CUR_LIM+100) || power > (limPower+1000))
		{
			if (limDuty > 2) limDuty -= 2; else limDuty = 0;
		}
		else if (avrCurADC < ABS_CUR_LIM && power < limPower)
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

				targetVREG = VREG_MIN;
				rpmVREG = 0;
				//lockAuxLoop90 = false;

				break;

			case 1: // Старт

				{
					__disable_irq();

					u32 v = targetRPM;

					tachoStep = 1;

					tachoLim = v*2 + 100;

					tachoCount = 0;

					v *= pulsesPerHeadRoundFix4;
					v >>= 5;

					if (v > 0)
					{
						HW::MRT->Channel[3].INTVAL = (((u32)MCK * 50 + v/2) / v)|(1UL<<31);
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

					rpmVREG = RPM_VREG_MIN + ((targetRPM * RPM_VREG_K) >> 8);

					motorState++;
				};

				break;

			case 4: // Разгон

				if ((GetMilliseconds()-tachoStamp) > 2000 || power > maxPower)
				{	
					lastMaxPower = power;
					motorState = 6;
				}
				else if (tachoCount >= tachoLim)
				{
					tachoCount = tachoLim;
					tachoStep = 64;
				
					rpmVREG = RPM_VREG_MIN + ((targetRPM * RPM_VREG_K) >> 8);
					//lockAuxLoop90 = true;

					tm.Reset();
					//tmOvrCrnt.Reset();

					motorState++;
				}
				else
				{
					rpmVREG = RPM_VREG_MIN + ((targetRPM * RPM_VREG_K) >> 8);
					SetDutyPWM(tachoPLL);
				};
					
				break;

			case 5: // Стабилизация оборотов

				if ((GetMilliseconds()-tachoStamp) > 1000 || power > maxPower)
				{	
					lastMaxPower = power;
					motorState++;
				}
				else if (tm.Check(1000))
				{
					if (tachoPLL >= (maxDuty*3/4))
					{
						targetVREG = curVREG * 6 / 4;
					}
					else if (tachoPLL <= (maxDuty/4))
					{
						targetVREG = curVREG / 2;
					};
				};

				break;

			case 6:

				tm.Reset();

				HW::MRT->Channel[3].CTRL = 0;
				tachoPLL = 0;
				tachoCount = 0;
				DisableDriver();

				dir = false;

				targetVREG = VREG_MIN;
				rpmVREG = 0;
				//lockAuxLoop90 = false;

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
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateMotor();	);
		CALL( UpdateVREG();		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <system_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

