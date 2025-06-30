#include "hardware.h"
#include "ComPort\ComPort.h"
#include "CRC\CRC16.h"
//#include "at25df021.h" 
#include "list.h"
#include "spi.h"
#include "pack.h"
//#include "FLASH\nand_ecc.h"
#include "MANCH\manch.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define RSPWAVE_BUF_NUM 8

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte build_date[128] __attribute__ ((section("L1_data"))) = "\n" "G26K_9_DSP" "\n" __DATE__ "\n" __TIME__ "\n" ;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static u16 SPI_CS_MASK[] = { PF8 };
//
//static S_SPIM	spi(0, HW::PIOF, SPI_CS_MASK, ArraySize(SPI_CS_MASK), SCLK);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_BF592
	
	static ComPort com;

	static RSPWAVE rspWaveBuf[RSPWAVE_BUF_NUM];

	#define Alloc_RSPWAVE_Buf() (rspWaveBuf+i)

#elif defined(CPU_BF706)
	
	static ComPort com(1, PIO_RTS, PIN_RTS);

	#ifdef RSPWAVE_BUF_MEM_L2
		#define Alloc_RSPWAVE_Buf() (RSPWAVE*)Alloc_L2_CacheWT(sizeof(RSPWAVE))
	#else
		static RSPWAVE rspWaveBuf[RSPWAVE_BUF_NUM] __attribute__ ((section("L1_data")));
		#define Alloc_RSPWAVE_Buf() (rspWaveBuf+i)
		//#define Alloc_RSPWAVE_Buf() (RSPWAVE*)Alloc_UnCached(sizeof(RSPWAVE))
#endif

#endif	

//struct Cmd
//{
//	byte cmd; 
//	byte chnl; 
//	byte clk; 
//	byte disTime; 
//	u16 enTime; 
//	byte chkSum; 
//	byte busy; 
//	byte ready; 
//};

static bool spiRsp = true;

static u16 manReqWord = 0xAD00;
static u16 manReqMask = 0xFF00;

static u16 numDevice = 1;
static u16 verDevice = 0x101;

static u32 manCounter = 0;

static bool startFire = false;

static u16 sampleDelay = US2DSP(30);//800;
static u16 sampleTime = NS2DSP(400);
static u16 sampleLen = 64;
static u16 gain = 0;

static u16 wavesPerRoundCM = 100;	
static u16 wavesPerRoundIM = 100;
//static u16 filtrType = 0;
//static u16 packType = 0;

#pragma instantiate List<RSPWAVE>
static List<RSPWAVE> processWave;
static List<RSPWAVE> freeRspWave;
static List<RSPWAVE> readyRspWave;
static List<RSPWAVE> cmWave;


//static void SaveParams();

static u16 mode = 0; // 0 - CM, 1 - IM

struct SensVars
{
	u16 descriminant;
	u16 deadTimeIndx;
	u16 deadTime;
	u16 delay;
	u16 filtrType;
	u16 fi_type;
	u16 packType;
	u16 fragLen;
	u16 freq;
	u16 st;
	u16 packLen;
};

static SensVars sensVars[3] = {0}; //{{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

static u16 refAmp = 0;
static u16 refTime = 0;

static i32 avrBuf[2][WAVE_MAXLEN+WAVE_OVRLEN] = {0};


//const i16 sin_Table[10] = {	0,	11585,	16384,	11585,	0,	-11585,	-16384,	-11585,	0,	11585 };

//const i16 sin_Table[10] = {	16384,	16384,	16384,	16384,	-16384,	-16384,	-16384,	-16384,	16384,	16384 };

//const i16 wavelet_Table[8] = { 328, 4922, 12442, 9053, -2522, -4922, -1616, -153 };
//const i16 wavelet_Table[8] = { 0, 4176, 12695, 11585, 0, -4176, -1649, -196};
//const i16 wavelet_Table[16] = { 0,509,1649,2352,0,-6526,-12695,-10869,0,10869,12695,6526,0,-2352,-1649,-509 };
//const i16 wavelet_Table[32] = {-1683,-3326,-3184,0,5304,9229,7777,0,-10037,-15372,-11402,0,11402,15372,10037,0,-7777,-9229,-5304,0,3184,3326,1683,0,-783,-720,-321,0,116,94,37,0};
//const i16 wavelet_Table[32] = {0,385,1090,1156,0,-1927,-3270,-2698,0,3468,5450,4239,0,-5010,-7630,-5781,0,6551,9810,7322,0,-8093,-11990,-8864,0,9634,14170,10405,0,-11176,-16350,-11947};
const i16 wavelet_Table[32] __attribute__ ((section("L1_data"))) = {0,-498,-1182,-1320,0,2826,5464,5065,0,-7725,-12741,-10126,0,11476,16381,11290,0,-9669,-12020,-7223,0,4713,5120,2690,0,-1344,-1279,-588,0,226,188,76};
//const i16 wavelet_Table[32] = {-498,-1182,-1320,0,2826,5464,5065,0,-7725,-12741,-10126,0,11476,16381,11290,0,-9669,-12020,-7223,0,4713,5120,2690,0,-1344,-1279,-588,0,226,188,76,0};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void PreProcessDspVars(ReqDsp01 *v, bool forced = false)
{
	static u16 freq[SENS_NUM] = {0};
	static u16 st[SENS_NUM] = {0};

	for (byte n = 0; n < SENS_NUM; n++)
	{
		SENS &sens = v->sens[n];
		SensVars &sv = sensVars[n];

		if (sens.st == 0) sens.st = 1;

		//if (sens.packType >= PACK_DCT0)
		//{
		//	u16 n = (sens.sl + FDCT_N - 1) / FDCT_N;
		//	sens.sl = (sens.sl + FDCT_N*3/4 + (n-1)*7) & ~(FDCT_N-1);
		//};

		if (sv.freq != sens.freq || forced)
		{
			sv.freq = sens.freq;

			u16 f = (sv.freq > 430) ? 430 : sv.freq;

			sv.st = (50000/8 + f/2) / f;
		};

		if (sv.packLen == 0) sv.packLen = FDCT_N;

		if (sens.fi_Type == 1) sens.st = sv.st;

		if (sens.st == 0) sens.st = 1;

		sv.packLen = sv.freq*sens.st*21/(65536*4/FDCT_N);

		if (sv.packLen > FDCT_N) sv.packLen = FDCT_N;
	};

	SetDspVars(v, forced);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc_01(const u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static RSPWAVE *curDsc = 0;
	static RspDsp01 rsp;

	ReqDsp01 *req = (ReqDsp01*)data;

	if (req->wavesPerRoundCM > 72) { req->wavesPerRoundCM = 72; }
	if (req->wavesPerRoundIM > 250) { req->wavesPerRoundIM = 250; }
	
	if (req->sensMask == 0) req->sensMask = 1;

	bool forced = (manCounter&0x7F) == 0;

	PreProcessDspVars(req, forced);

	mode = req->mode;

	for (byte n = 0; n < 3; n++)
	{
		SensVars &sv = sensVars[n];
		SENS &rs = req->sens[n];

		sv.descriminant	= rs.descriminant;
		sv.filtrType	= rs.filtrType;
		sv.fi_type		= rs.fi_Type;
		sv.packType		= rs.packType;
		sv.fragLen		= rs.fragLen;

		if (sv.deadTime != rs.deadTime || sv.delay != rs.sd || forced)
		{
			sv.deadTime = rs.deadTime;
			sv.delay = rs.sd;

			u16 t = sv.deadTime;

			t = (t > sv.delay) ? (t - sv.delay) : 0;

			sv.deadTimeIndx = (t != 0) ? ((t + rs.st/2) / rs.st) : 0;
		};
	};

	wavesPerRoundCM = req->wavesPerRoundCM;	
	wavesPerRoundIM = req->wavesPerRoundIM;

	SetFireVoltage(req->fireVoltage);

	if (wb == 0) return false;

	if (curDsc != 0) freeRspWave.Add(curDsc), curDsc = 0;

	/*if (!spiRsp)*/ curDsc = readyRspWave.Get();

	if (curDsc == 0)
	{
		rsp.v01.rw = data[0];
		rsp.v01.len = sizeof(rsp.v01);
		rsp.v01.version = ReqDsp01::VERSION;
		rsp.v01.fireVoltage = GetFireVoltage();
		rsp.v01.crc = GetCRC16(&rsp, sizeof(rsp.v01)-2);

		wb->data = &rsp;			 
		wb->len = sizeof(rsp.v01);	 
	}
	else
	{
		DEBUG_ASSERT(curDsc->next == 0);

		wb->data = curDsc->data;			 
		wb->len = curDsc->dataLen*2;	 
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFunc_07(const u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	while(1) { };
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc(ComPort::WriteBuffer *wb, ComPort::ReadBuffer *rb)
{
	u16 *p = (u16*)rb->data;
	bool r = false;

	u16 t = p[0];

	if ((t & manReqMask) != manReqWord || rb->len < 2)
	{
//		bfERC++; 
		return false;
	};

	manCounter += 1;

	u16 len = (rb->len)>>1;

	t &= 0xFF;

	switch (t)
	{
		case 1: 	r = RequestFunc_01(p, len, wb); break;
//		case 5: 	r = RequestFunc_05(p, len, wb); break;
//		case 6: 	r = RequestFunc_06(p, len, wb); break;
		case 7: 		RequestFunc_07(p, len, wb); break;
	};

	return r;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBlackFin()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	//static u16 buf[256];

	HW::ResetWDT();

	switch(i)
	{
		case 0:

			rb.data = build_date;
			rb.maxLen = 127;//sizeof(buf);
			com.Read(&rb, ~0, US2COM(50));
			i++;

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0)
				{
					u16 crc = GetCRC16(rb.data, rb.len);

					if (rb.len != sizeof(ReqDsp01))
					{
						//HW::PIOC->SET(PC4);
						i = 0;
						//HW::PIOC->CLR(PC4);
					}
					else if (crc != 0)
					{
						//HW::PIOC->SET(PC5);
						i = 0;
						//HW::PIOC->CLR(PC5);
					}
					else if (RequestFunc(&wb, &rb))
					{
						com.Write(&wb);

						i++;
					}
					else
					{
						//HW::PIOC->SET(PC6);
						i = 0;
						//HW::PIOC->CLR(PC6);
					};
				}
				else
				{
					//HW::PIOC->SET(PC7);
					i = 0;
					//HW::PIOC->CLR(PC7);
				};
			};

			break;

		case 2:

			if (!com.Update())
			{
				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateSPI()
//{
//	static RSPWAVE *curDsc = 0;
//	static CTM32 tm;
//	static byte i = 0;
//
//	switch(i)
//	{
//		case 0:
//
//			if (spiRsp) curDsc = readyRspWave.Get();
//
//			if (curDsc != 0)
//			{
//				spi.SetMode(CPHA|CPOL);
//				spi.WriteAsyncDMA(curDsc->data, (curDsc->dataLen*2+3) & ~3);
//
//				i++;
//			};
//
//			break;
//
//		case 1:
//
//			if (spi.CheckWriteComplete())
//			{
//				if (curDsc != 0)
//				{
//					freeRspWave.Add(curDsc);
//					
//					curDsc = 0;
//				};
//
//				tm.Reset();
//
//				i++;
//			};
//
//			break;
//
//		case 2:
//
//			if (tm.Check(US2CTM(1000)))
//			{
//				i = 0;
//			};
//
//			break;
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//byte eccBuf[8192] __attribute__ ((section("L2_SRAM_WT"))); 
//
//byte ecc[96];
//
//#pragma optimize_off
//
//void UpdateECC()
//{
//	static CTM32 tm;
//
//	if (tm.Check(MS2CTM(1000)))
//	{
//		u32 t = cli();
//
//		HW::PIOB->BSET(4);
//
//		Nand_ECC_Calc_V2(eccBuf, sizeof(eccBuf), ecc);
//
//		HW::PIOB->BCLR(4);
//
//		sti(t);
//	};
//}
//
//#pragma optimize_as_cmd_line
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 manBuf[10];

void UpdateMan()
{
	static CTM32 tm;
	static MTB mtb;

	if (tm.Check(MS2CTM(1000)))
	{
		mtb.data1 = manBuf;
		mtb.len1 = 2;
		mtb.baud = 0;
		SendManData(&mtb);
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateBlackFin()	);
		CALL( UpdateHardware()	);	
		CALL( UpdateMan()		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void Filtr_Data(RSPWAVE &dsc, u32 filtrType)
{
	RspCM &rsp = *((RspCM*)dsc.data);
	u16 *d = rsp.data;

	if (filtrType == 1 && rsp.hdr.sensType < 2)
	{
		i32 *ab = avrBuf[rsp.hdr.sensType]; 

		for (u32 i = rsp.hdr.sl+WAVE_OVRLEN; i > 0; i--)
		{
			i16 v = d[0];

			*(d++) = v -= *ab/32;

			*(ab++) += v;
		};
	}
	else if (filtrType == 2)
	{
		//i32 av = 0;

		for (u32 i = rsp.hdr.sl+WAVE_OVRLEN-1; i > 0; i--)
		{
			i16 v = (d[0] + d[1])/2;
			*(d++) = v;
		};
	}
	else if (filtrType == 3)
	{
		i32 av = 0;
		//i32 *ab = avrBuf;

		for (u32 i = rsp.hdr.sl+WAVE_OVRLEN-3; i > 0; i--)
		{
			i16 v = (d[2] - d[0] + d[3] - d[1])/4;
			*(d++) = v;
		};
	};
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void Filtr_Wavelet(RSPWAVE &dsc, u16 descrIndx)
{
	RspCM &rsp = *((RspCM*)dsc.data);

	rsp.hdr.packLen = descrIndx;

	i16 *d = (i16*)rsp.data;

	i32 max = -32768;
	i32 imax = -1;

	if (expected_true(descrIndx < rsp.hdr.sl))
	{
		//i16 *p = d+rsp.hdr.sl;

		//for (i32 i = ArraySize(wavelet_Table); i > 0; i--) *(p++) = 0;

		d += descrIndx;

		for (i32 i = rsp.hdr.sl - descrIndx; i > 0 ; i--)
		{
			i32 sum = 0;

			for (i32 j = 1; j < ArraySize(wavelet_Table); j += 2)
			{
				sum += (i32)d[j] * wavelet_Table[j]; //sin_Table[j&7];
			};

			sum /= 16384*4;
			
			d++;//*(d++) = sum;

			if (sum < 0) sum = -sum;

			if (sum > max) { max = sum; imax = i; };
		};
	};

	if (imax >= 0)
	{
		imax = rsp.hdr.sl - imax;
		u32 t = rsp.hdr.sd + imax * rsp.hdr.st;
		rsp.hdr.fi_time  = (t < 0xFFFF) ? t : 0xFFFF;
		rsp.hdr.fi_amp = max;
		rsp.hdr.maxAmp = max;
		dsc.fi_index = imax;
	}
	else
	{
		rsp.hdr.fi_time	= ~0;
		rsp.hdr.fi_amp	= 0;
		rsp.hdr.maxAmp	= 0;
		dsc.fi_index	= ~0;
	};
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void GetAmpTimeIM_3(RSPWAVE &dsc, u16 ind, u16 imThr)
{
	RspCM &rsp = *((RspCM*)dsc.data);
	rsp.hdr.fi_amp = 0;
	rsp.hdr.fi_time = ~0;
	rsp.hdr.packLen = ind;
	dsc.fi_index = ~0;

	u16 *data = rsp.data;
	
	//u16 descr = (imDescr > imDelay) ? (imDescr - imDelay) : 0;

	//u16 ind = descr / rsp->st;

	if (ind >= rsp.hdr.sl) return;

	data += ind;

	u16 len = rsp.hdr.sl - ind;

	i32 max = -32768;
	i32 imax = -1;

	i32 ampmax = 0;

	for (u32 i = len; i > 0; i--)
	{
		i32 v = (i16)(*(data++));

		if (v > imThr)
		{ 
			if (v > max) { max = v; imax = ind; };
		}
		else if (imax >= 0 && v < 0)
		{ 
			ind++;
			break;
		};

		if (v < 0) v = -v;
		if (v > ampmax) ampmax = v;
		
		ind++;
	};

	if (imax >= 0)
	{
		rsp.hdr.fi_amp = max;
		u32 t = rsp.hdr.sd + imax * rsp.hdr.st;
		rsp.hdr.fi_time = (t < 0xFFFF) ? t : 0xFFFF;
		dsc.fi_index = (imax>8) ? (imax-8) : 0;
	};

	if (rsp.hdr.sl > ind)
	{
		for (u32 i = rsp.hdr.sl - ind; i > 0; i--)
		{
			i32 v = (i16)(*(data++));

			if (v < 0) v = -v;
			if (v > ampmax) ampmax = v;
		};
	};

	rsp.hdr.maxAmp = (ampmax < 0xFFFF) ? ampmax : 0xFFFF;
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void PackDataCM(RSPWAVE *dsc, u16 pack)
{
	switch (pack)
	{
		default:
		case PACK_NO:								break;
		//case PACK_BIT12:	Pack_1_Bit12(dsc);		break;
		case PACK_ULAW12:	Pack_2_uLaw(dsc);		break;
		case PACK_ADPCMIMA:	Pack_3_ADPCMIMA(dsc);	break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void ProcessDataCM(RSPWAVE *dsc)
{
	cmWave.Add(dsc);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FragDataCM(RSPWAVE *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	u16 fragLen = sensVars[rsp.hdr.sensType].fragLen;

	u16 stind = dsc->fi_index;

	if (fragLen == 0 || stind >= rsp.hdr.sl) return;

	if (fragLen > rsp.hdr.sl)
	{
		fragLen = rsp.hdr.sl;
		stind = 0;
	}
	else if ((stind + fragLen) > rsp.hdr.sl)
	{
		stind = rsp.hdr.sl - fragLen;
	};

	if (stind > 0)
	{
		u16 *s = rsp.data + stind;
		u16 *d = rsp.data;

		for (u32 i = fragLen; i > 0; i--) *(d++) = *(s++);

		rsp.hdr.sd += stind * rsp.hdr.st;
	};

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + fragLen;
	
	rsp.hdr.sl = fragLen;

	DEBUG_ASSERT(dsc->next == 0);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SendReadyDataIM(RSPWAVE *dsc, u16 len)
{
	RspIM *rsp = (RspIM*)dsc->data;

	rsp->hdr.rw			= manReqWord|0x50;	//1. ответное слово

	rsp->hdr.refAmp		= refAmp;
	rsp->hdr.refTime	= refTime;
	rsp->hdr.dataLen	= len;				//11. Длина (макс 1024)

	dsc->dataLen		= (sizeof(RspIM)-sizeof(rsp->data))/2 + len*2;
	dsc->data[dsc->dataLen]	= GetCRC16(&rsp->hdr, sizeof(rsp->hdr));
	dsc->dataLen += 1;

	//dsc->offset = (sizeof(*rsp) - sizeof(rsp->data)) / 2;

	DEBUG_ASSERT(dsc->next == 0);

	readyRspWave.Add(dsc);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void ProcessDataIM(RSPWAVE *dsc)
{
	//static RSPWAVE *imdsc = 0;
	//static u32 count = 0;
	//static u32 cmCount = 0;
	//static u32 i = 0;
	//static u16 prevShaftCount = 0;
	//static u16 wpr = 180;

	//u32 istat = cli();

	struct S_IM
	{
		RSPWAVE *imdsc;
		u32 count;
		u32 cmCount;
		u32 i;
		u16 prevShaftCount;
		u16 wpr;
	};

	static S_IM simArr[SENS_NUM-1] = {{0,0,0,0,0,180},{0,0,0,0,0,180}};

	const RspCM &rsp = *((RspCM*)dsc->data);
	
	S_IM &sim = simArr[rsp.hdr.sensType];

	if (dsc->shaftCount != sim.prevShaftCount)
	{
		sim.prevShaftCount = dsc->shaftCount;

		if (sim.imdsc != 0)
		{
			SendReadyDataIM(sim.imdsc, sim.i);

			sim.imdsc = 0;
		};
	};

	if (sim.imdsc == 0)
	{
		sim.wpr = wavesPerRoundIM;
		sim.count = sim.wpr*9/8; if (sim.count > 512) sim.count = 512;
		sim.cmCount = (sim.wpr+8) / 16;
		sim.i = 0;

		sim.imdsc = freeRspWave.Get();

		if (sim.imdsc != 0)
		{
			RspIM *ir = (RspIM*)sim.imdsc->data;

			ir->hdr.time		= rsp.hdr.time;
			ir->hdr.hallTime	= rsp.hdr.hallTime;
			ir->hdr.gain		= rsp.hdr.gain;
			ir->hdr.ax			= rsp.hdr.ax;
			ir->hdr.ay			= rsp.hdr.ay;
			ir->hdr.az			= rsp.hdr.az;
			ir->hdr.at			= rsp.hdr.at;
			ir->hdr.sensType	= rsp.hdr.sensType;

			u16 *d = ir->data;

			for (u32 i = 10; i > 0; i--) { *(d++) = 0; };
		};
	};

	if (sim.imdsc != 0)
	{
		RspIM *ir = (RspIM*)sim.imdsc->data;

		u16 *data = ir->data + sim.i*2;

		if (dsc->fireIndex < sim.count)
		{
			while (sim.i < dsc->fireIndex)
			{
				*(data++) = 0;
				*(data++) = 0;
				sim.i++;
			};
		};

		if (sim.i < sim.count)
		{
			*(data++) = rsp.hdr.fi_amp;
			*(data++) = rsp.hdr.fi_time;
			sim.i++;
		};

		if (sim.i >= sim.count)
		{
			SendReadyDataIM(sim.imdsc, sim.count);

			sim.imdsc = 0;
		};
	};

	if (sim.cmCount == 0)
	{
		sim.cmCount = (sim.wpr+4) / 8;

		ProcessDataCM(dsc);
	}
	else
	{
		freeRspWave.Add(dsc);
	};

	sim.cmCount -= 1;

	//sti(istat);
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(CPU_BF706) && defined(SPORT_BUF_MEM_L2)

	static DSCSPORT tempSportDscA __attribute__ ((section("L1_data_a")));
	static DSCSPORT tempSportDscB __attribute__ ((section("L1_data_b")));
	static List<DSCSPORT> freeTempSPORT		__attribute__ ((section("L1_data")));
	static List<DSCSPORT> readyTempSPORT	__attribute__ ((section("L1_data")));

//#pragma optimize_for_speed


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitPreProcessSPORT()
{
	freeTempSPORT.Add(&tempSportDscA);
	freeTempSPORT.Add(&tempSportDscB);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void PreProcessSPORT()
{
	static byte state = 0;
	static DSCSPORT *dsc = 0;
	static DSCSPORT *tmp = 0;

#ifdef _DEBUG
	DSCSPORT **pdsc = &dsc;
#endif

	switch (state)
	{
		case 0:

			dsc = GetDscSPORT();

			if (dsc != 0)
			{
				state++;
			}
			else
			{
				break;
			};

		case 1:

			tmp = freeTempSPORT.Get();

			if (tmp != 0)
			{
				Pin_PreProcessSPORT_Set();

				u32 len = dsc->len + WAVE_OVRLEN;

				if (dsc->chMask & 2) len *= 2;

				HW::DMA->SRC0.ADDRSTART = dsc;
				HW::DMA->SRC0.XCNT = HW::DMA->DST0.XCNT = (len + (sizeof(*dsc)-sizeof(dsc->data))/2 + 1)/2;
				HW::DMA->SRC0.XMOD = HW::DMA->DST0.XMOD = 4;
				HW::DMA->SRC0.STAT = ~0;

				HW::DMA->DST0.ADDRSTART = tmp;
				HW::DMA->DST0.STAT = ~0;

				HW::DMA->SRC0.CFG = DMA_INT_XCNT|DMA_FLOW_STOP|DMA_PSIZE32|DMA_MSIZE32|DMA_WNR|	DMA_SYNC|DMA_EN;
				HW::DMA->DST0.CFG = DMA_INT_XCNT|DMA_FLOW_STOP|DMA_PSIZE32|DMA_MSIZE32|			DMA_SYNC|DMA_EN;

				state++;

				break;
			}
			else
			{
				break;
			};

		case 2:

			if (HW::DMA->DST0.STAT & (DMA_STAT_IRQDONE|DMA_STAT_IRQERR))
			{
				HW::DMA->SRC0.CFG = 0;
				HW::DMA->DST0.CFG = 0;
				HW::DMA->SRC0.STAT = ~0;
				HW::DMA->DST0.STAT = ~0;

				//u32 t = cli();

				FreeDscSPORT(dsc);

				readyTempSPORT.Add(tmp);

				//sti(t);

				Pin_PreProcessSPORT_Clr();

				state = 0;
			};

			break;
	};
}

#pragma optimize_as_cmd_line

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma optimize_for_speed

static void ProcessSPORT()
{
	static byte state = 0;
	static DSCSPORT *dsc = 0;
	static RSPWAVE  *rsp = 0;
	static u16 angle = 0;

#ifdef _DEBUG
	DSCSPORT **pdsc = &dsc;
	RSPWAVE  **prsp = &rsp;
#endif

	#if defined(CPU_BF706) && defined(SPORT_BUF_MEM_L2)
		PreProcessSPORT();
	#endif

	switch (state)
	{
		case 0:

			#if defined(CPU_BF592) || !defined(SPORT_BUF_MEM_L2)
				dsc = GetDscSPORT();
			#elif defined(CPU_BF706)
				dsc = readyTempSPORT.Get();
			#endif

			if (dsc == 0)
			{
				break;
			}
			else
			{
				Pin_ProcessSPORT_Set();

				state += 1;
			};

		case 1:

			rsp = freeRspWave.Get();

			if (rsp == 0)
			{
				break;
			}
			else
			{
				DEBUG_ASSERT(rsp->next == 0);
				DEBUG_ASSERT(rsp->dataLen <= ArraySize(rsp->data));

				rsp->mode = 0;
				rsp->dataLen = sizeof(RspHdrCM)/2+dsc->len;
				rsp->fireIndex = dsc->fireIndex;
				rsp->shaftCount = dsc->shaftCount;

				//u32 t = (72000 * dsc->rotCount + 74) / 147;
				u32 t = (501551 * dsc->rotCount + 512) >> 10;

				if (t >= 36000) t -= 36000;

				angle = t;

				RspCM &r = *((RspCM*)rsp->data);
				//RspHdrCM *r = (RspHdrCM*)rsp->data;

				r.hdr.rw		= manReqWord|0x40;			//1. ответное слово
				r.hdr.time		= dsc->mmsec;
				r.hdr.hallTime	= dsc->shaftTime;
				r.hdr.motoCount	= dsc->motoCount;
				r.hdr.headCount	= dsc->shaftCount;
				r.hdr.ax		= dsc->ax;
				r.hdr.ay		= dsc->ay;
				r.hdr.az		= dsc->az;
				r.hdr.at		= dsc->at;
				r.hdr.sensType	= dsc->sensType;
				r.hdr.angle		= angle;
				r.hdr.maxAmp	= 0;
				r.hdr.fi_amp	= 0;
				r.hdr.fi_time	= 0;
				r.hdr.gain		= dsc->gain;
				r.hdr.st 		= dsc->sampleTime;			//15. Шаг оцифровки
				r.hdr.sl 		= dsc->len;					//16. Длина оцифровки (макс 2028)
				r.hdr.sd 		= dsc->sampleDelay;			//17. Задержка оцифровки  
				r.hdr.packType	= 0;						//18. Упаковка
				r.hdr.packLen	= 0;						//19. Размер упакованных данных

				state += 1;
			};

			break;
		
		case 2:
		{
			DEBUG_ASSERT(rsp->next == 0);
			DEBUG_ASSERT(rsp->dataLen <= ArraySize(rsp->data));

			RspCM &r = *((RspCM*)rsp->data);
			u16 *d = r.data;

			if ((dsc->chMask&3) == 1)
			{
				u16 *s = dsc->data+1;

				for (u32 i = dsc->len+WAVE_OVRLEN; i > 0; i--)
				{
					*(d++) = s[0] - 2048; s++;
				};

				//FreeDscSPORT(dsc);

				//Pin_ProcessSPORT_Clr();

				state = 5;
			}
			else if (dsc->chMask & 1)
			{
				u16 *s = dsc->data+1;

				for (u32 i = dsc->len+WAVE_OVRLEN; i > 0; i--)
				{
					*(d++) = s[0] - 2048; s += 2;
				};

				state += 1;
			}
			else // chMask == 2
			{
				u16 *s = dsc->data+3;

				for (u32 i = dsc->len+WAVE_OVRLEN; i > 0; i--)
				{
					*(d++) = s[0] - 2048; s += 2;
				};

				//FreeDscSPORT(dsc);

				//Pin_ProcessSPORT_Clr();

				state = 5;
			};

			DEBUG_ASSERT(rsp->next == 0);
			DEBUG_ASSERT(rsp->dataLen <= ArraySize(rsp->data));

			processWave.Add(rsp); rsp = 0;

			break;
		};

		case 3:

			rsp = freeRspWave.Get();

			if (rsp == 0)
			{
				break;
			}
			else
			{
				DEBUG_ASSERT(rsp->next == 0);
				DEBUG_ASSERT(rsp->dataLen <= ArraySize(rsp->data));

				rsp->mode = 0;
				rsp->dataLen = sizeof(RspHdrCM)/2+dsc->len;
				rsp->fireIndex = dsc->fireIndex;
				rsp->shaftCount = dsc->shaftCount;

				RspCM &r = *((RspCM*)rsp->data);
				//RspHdrCM *r = (RspHdrCM*)rsp->data;

				r.hdr.rw		= manReqWord|0x40;			//1. ответное слово
				r.hdr.time		= dsc->mmsec;
				r.hdr.hallTime	= dsc->shaftTime;
				r.hdr.motoCount	= dsc->motoCount;
				r.hdr.headCount	= dsc->shaftCount;
				r.hdr.ax		= dsc->ax;
				r.hdr.ay		= dsc->ay;
				r.hdr.az		= dsc->az;
				r.hdr.at		= dsc->at;
				r.hdr.sensType	= dsc->sensType;
				r.hdr.angle		= angle;
				r.hdr.maxAmp	= 0;
				r.hdr.fi_amp	= 0;
				r.hdr.fi_time	= 0;
				r.hdr.gain		= dsc->gain;
				r.hdr.st 		= dsc->sampleTime;			//15. Шаг оцифровки
				r.hdr.sl 		= dsc->len;					//16. Длина оцифровки (макс 2028)
				r.hdr.sd 		= dsc->sampleDelay;			//17. Задержка оцифровки  
				r.hdr.packType	= 0;						//18. Упаковка
				r.hdr.packLen	= 0;						//19. Размер упакованных данных

				state += 1;
			};

			break;

		case 4:
		{
			DEBUG_ASSERT(rsp->next == 0);
			DEBUG_ASSERT(rsp->dataLen <= ArraySize(rsp->data));

			RspCM &r = *((RspCM*)rsp->data);
			u16 *d = r.data;
			u16 *s = dsc->data+3;

			for (u32 i = dsc->len+WAVE_OVRLEN; i > 0; i--)
			{
				*(d++) = s[0] - 2048; s += 2;
			};

			DEBUG_ASSERT(rsp->next == 0);

			processWave.Add(rsp); rsp = 0;

			state = 5;

			//break;
		};

		case 5:
		{
			#if defined(CPU_BF592) || !defined(SPORT_BUF_MEM_L2)
				FreeDscSPORT(dsc);
			#elif defined(CPU_BF706)
				freeTempSPORT.Add(dsc);
			#endif

			Pin_ProcessSPORT_Clr();

			state = 0;

			break;
		};

	};
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateCM()
{
	static byte state = 0;
	static RSPWAVE *dsc = 0;
	static u16 packLen = 0;
	static u16 index = 0;
	static byte OVRLAP = 3;
	static u16 scale = 0;

#ifdef __DEBUG
	RSPWAVE **pdsc = &dsc;
#endif

	switch (state)
	{
		case 0: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
			
			dsc = cmWave.Get();

			if (dsc != 0)
			{
				DEBUG_ASSERT(dsc->next == 0);
				DEBUG_ASSERT(dsc->dataLen <= ArraySize(dsc->data));

				Pin_UpdateMode_Set();

				RspCM *rsp = (RspCM*)dsc->data;

				FragDataCM(dsc);

				state++;
			};

			break;

		case 1: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		{
			DEBUG_ASSERT(dsc->next == 0);
			DEBUG_ASSERT(dsc->dataLen <= ArraySize(dsc->data));

			RspCM *rsp = (RspCM*)dsc->data; 
	
			u16 pack = sensVars[rsp->hdr.sensType].packType;

			if (pack < PACK_DCT0)
			{
				PackDataCM(dsc, sensVars[rsp->hdr.sensType].packType);

				dsc->data[dsc->dataLen] = GetCRC16(&rsp->hdr, sizeof(rsp->hdr));
				dsc->dataLen += 1;

				DEBUG_ASSERT(dsc->dataLen <= ArraySize(dsc->data));

				readyRspWave.Add(dsc); dsc = 0;

				state = 0;

				Pin_UpdateMode_Clr();
			}
			else
			{
				index = 0;
				rsp->hdr.packType = pack;
				rsp->hdr.packLen = 0;
				OVRLAP = (rsp->hdr.packType > PACK_DCT0) ? 7 : 3;
				dsc->dataLen -= rsp->hdr.sl;
				rsp->hdr.sl += WAVE_OVRLEN;
				state++;
			};

			break;
		};

		case 2: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
		{
			DEBUG_ASSERT(dsc->next == 0);
			DEBUG_ASSERT(dsc->dataLen <= ArraySize(dsc->data));

			RspCM *rsp = (RspCM*)dsc->data; 

			Pack_FDCT_Transform((i16*)(rsp->data + index));

			state++;

			break;
		};

		case 3: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
		{
			DEBUG_ASSERT(dsc->next == 0);
			DEBUG_ASSERT(dsc->dataLen <= ArraySize(dsc->data));

			RspCM *rsp = (RspCM*)dsc->data; 

			byte shift = 3 - (sensVars[rsp->hdr.sensType].packType - PACK_DCT0);

			packLen = Pack_FDCT_Quant(sensVars[rsp->hdr.sensType].packLen, shift, &scale);

			state++;

			break;
		};

		case 4: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
		{
			DEBUG_ASSERT(dsc->next == 0);
			DEBUG_ASSERT(dsc->dataLen <= ArraySize(dsc->data));

			RspCM *rsp = (RspCM*)dsc->data; 

			PackDCT *pdct = (PackDCT*)(rsp->data+rsp->hdr.packLen);

			Pack_FDCT_uLaw(pdct->data, packLen, scale);

			pdct->len = packLen;
			pdct->scale = scale;

			rsp->hdr.packLen += 1 + packLen/2;

			index += FDCT_N - OVRLAP;

			if ((index+FDCT_N) <= rsp->hdr.sl)
			{
				state = 2;
			}
			else
			{
				dsc->dataLen += rsp->hdr.packLen;

				rsp->hdr.sl = index + OVRLAP;

				DEBUG_ASSERT(dsc->dataLen <= ArraySize(dsc->data));

				readyRspWave.Add(dsc); 

				dsc->data[dsc->dataLen] = GetCRC16(&rsp->hdr, sizeof(rsp->hdr));
				dsc->dataLen += 1;

				dsc = 0;

				state = 0;

				Pin_UpdateMode_Clr();
			};

			break;
		};

	}; // switch (i);
};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMode()
{
	static byte i = 0;
	static RSPWAVE *dsc = 0;

	ProcessSPORT();

	switch(i)
	{
		case 0:
		
			dsc = processWave.Get();

			if (dsc != 0)
			{
				Pin_UpdateMode_Set();
				
				i++;
			}
			else
			{
				Update();
			};

			break;

		case 1: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		{
			RspHdrCM *rsp = (RspHdrCM*)dsc->data;

			Filtr_Data(*dsc, sensVars[rsp->sensType].filtrType);

			i++;

			break;
		};

		case 2: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		{
			RspHdrCM *rsp = (RspHdrCM*)dsc->data;

			const SensVars &sens = sensVars[rsp->sensType];

			if (sens.fi_type != 0)
			{
				Filtr_Wavelet(*dsc, sens.deadTimeIndx);
			}
			else
			{
				GetAmpTimeIM_3(*dsc, sens.deadTimeIndx, sens.descriminant);
			};

			i++;

			break;
		};

		case 3: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		{		

			RspHdrCM *rsp = (RspHdrCM*)dsc->data;

			if (rsp->sensType != 2)
			{
				switch (mode)
				{
					case 0:		ProcessDataCM(dsc);		break;
					case 1:		ProcessDataIM(dsc);		break;
					default:	freeRspWave.Add(dsc);	break;
				};
			}
			else
			{
				refAmp	= rsp->fi_amp;
				refTime = rsp->fi_time;
				
				ProcessDataCM(dsc);
			};

			dsc = 0;

			Pin_UpdateMode_Clr();

			i = 0;

			break;
		};
	};

	UpdateCM();

	if ((i&1) == 0) Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void Check_RSPWAVE_BUF()
//{
//	for (byte i = 0; i < RSPWAVE_BUF_NUM; i++)
//	{
//		if (rspWaveBuf[i].dataLen & 0x8000) Pin_PreProcessSPORT_Set(), Pin_PreProcessSPORT_Clr();
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main( void )
{
	//static byte s = 0;
	//static byte i = 0;

	//static u32 pt = 0;

	//static CTM32 tm;

	InitHardware();

	Pack_Init();

	com.Connect(ComPort::ASYNC, DSP_COM_BAUDRATE, DSP_COM_PARITY, DSP_COM_STOPBITS);

	//CheckFlash();

	//spi.Connect(25000000);

	u32 t = cli();

	for (u16 i = 0; i < RSPWAVE_BUF_NUM; i++)
	{
		RSPWAVE *rsp = Alloc_RSPWAVE_Buf();

		freeRspWave.Add(rsp);
	};

	#if defined(CPU_BF706) && defined(SPORT_BUF_MEM_L2)
		InitPreProcessSPORT();
	#endif

	sti(t);

	while (1)
	{
		MAIN_LOOP_PIN_TGL();

		UpdateMode();
		
#ifdef _DEBUG
		//Check_RSPWAVE_BUF();
#endif

		//MAIN_LOOP_PIN_CLR();
	};

//	return 0;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

