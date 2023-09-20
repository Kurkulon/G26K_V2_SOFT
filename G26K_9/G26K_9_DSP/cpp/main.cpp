#include "hardware.h"
#include "ComPort.h"
#include "CRC16.h"
//#include "at25df021.h"
#include "list.h"
#include "spi.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define RSPWAVE_BUF_NUM 8

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte build_date[128] = "\n" "G26K_9_DSP" "\n" __DATE__ "\n" __TIME__ "\n";

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 SPI_CS_MASK[] = { PF8 };

static S_SPIM	spi(0, HW::PIOF, SPI_CS_MASK, ArraySize(SPI_CS_MASK), SCLK);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ComPort com;

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

static bool spiRsp = false;

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


static RSPWAVE rspWaveBuf[RSPWAVE_BUF_NUM];


//static void SaveParams();

static u16 mode = 0; // 0 - CM, 1 - IM

struct SensVars
{
	u16 thr;
	u16 descrIndx;
	u16 descr;
	u16 delay;
	u16 filtr;
	u16 fi_type;
};

static SensVars sensVars[3] = {0}; //{{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

static u16 refAmp = 0;
static u16 refTime = 0;

static i32 avrBuf[2][RSPWAVE_BUF_LEN - sizeof(RspHdrCM)/2] = {0};


//const i16 sin_Table[10] = {	0,	11585,	16384,	11585,	0,	-11585,	-16384,	-11585,	0,	11585 };

//const i16 sin_Table[10] = {	16384,	16384,	16384,	16384,	-16384,	-16384,	-16384,	-16384,	16384,	16384 };

//const i16 wavelet_Table[8] = { 328, 4922, 12442, 9053, -2522, -4922, -1616, -153 };
//const i16 wavelet_Table[8] = { 0, 4176, 12695, 11585, 0, -4176, -1649, -196};
//const i16 wavelet_Table[16] = { 0,509,1649,2352,0,-6526,-12695,-10869,0,10869,12695,6526,0,-2352,-1649,-509 };
//const i16 wavelet_Table[32] = {-1683,-3326,-3184,0,5304,9229,7777,0,-10037,-15372,-11402,0,11402,15372,10037,0,-7777,-9229,-5304,0,3184,3326,1683,0,-783,-720,-321,0,116,94,37,0};
//const i16 wavelet_Table[32] = {0,385,1090,1156,0,-1927,-3270,-2698,0,3468,5450,4239,0,-5010,-7630,-5781,0,6551,9810,7322,0,-8093,-11990,-8864,0,9634,14170,10405,0,-11176,-16350,-11947};
const i16 wavelet_Table[32] = {0,-498,-1182,-1320,0,2826,5464,5065,0,-7725,-12741,-10126,0,11476,16381,11290,0,-9669,-12020,-7223,0,4713,5120,2690,0,-1344,-1279,-588,0,226,188,76};
//const i16 wavelet_Table[32] = {-498,-1182,-1320,0,2826,5464,5065,0,-7725,-12741,-10126,0,11476,16381,11290,0,-9669,-12020,-7223,0,4713,5120,2690,0,-1344,-1279,-588,0,226,188,76,0};

#define K_DEC (1<<2)
#define K_DEC_MASK (K_DEC-1)


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void PreProcessDspVars(ReqDsp01 *v, bool forced = false)
{
	static u16 freq[SENS_NUM] = {0};
	static u16 st[SENS_NUM] = {0};

	for (byte n = 0; n < SENS_NUM; n++)
	{
		SENS &sens = v->sens[n];
		u16 &fr = freq[n];
		u16 &s = st[n];

		if (sens.fi_Type != 0)
		{
			if (fr != sens.freq || forced)
			{
				u16 f = fr = sens.freq;

				f = (f > 430) ? 430 : f;

				s = (50000/8 + f/2) / f;
			};

			sens.st = s;
		};
	};

	SetDspVars(v, forced);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

		sv.thr		= rs.thr;
		sv.filtr	= rs.filtr;
		sv.fi_type	= rs.fi_Type;

		if (sv.descr != rs.descr || sv.delay != rs.sd || forced)
		{
			sv.descr = rs.descr;
			sv.delay = rs.sd;

			u16 t = sv.descr;

			t = (t > sv.delay) ? (t - sv.delay) : 0;

			if (t != 0)
			{
				sv.descrIndx = (t + rs.st/2) / rs.st;
			}
			else
			{
				sv.descrIndx = 0;
			};
		};
	};

	wavesPerRoundCM = req->wavesPerRoundCM;	
	wavesPerRoundIM = req->wavesPerRoundIM;

	SetFireVoltage(req->fireVoltage);

	if (wb == 0) return false;

	if (curDsc != 0)
	{
		freeRspWave.Add(curDsc);

		curDsc = 0;
	};

	if (!spiRsp) curDsc = readyRspWave.Get();

	if (curDsc == 0)
	{
		rsp.rw = data[0];
		rsp.len = sizeof(rsp);
		rsp.version = rsp.VERSION;
		rsp.fireVoltage = GetFireVoltage();
		rsp.crc = GetCRC16(&rsp, sizeof(rsp)-2);

		wb->data = &rsp;			 
		wb->len = sizeof(rsp);	 
	}
	else
	{
		wb->data = curDsc->data;			 
		wb->len = curDsc->dataLen*2;	 
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFunc_07(const u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	while(1) { };
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBlackFin()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	//static u16 buf[256];

	ResetWDT();

	switch(i)
	{
		case 0:

			rb.data = build_date;
			rb.maxLen = sizeof(build_date);
			com.Read(&rb, ~0, US2COM(50));
			i++;

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0 && GetCRC16(rb.data, rb.len) == 0)
				{
					if (RequestFunc(&wb, &rb))
					{
						com.Write(&wb);

						i++;
					}
					else
					{
						i = 0;
					};
				}
				else
				{
					i = 0;
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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSPI()
{
	static RSPWAVE *curDsc = 0;
	static RTM32 tm;
	static byte i = 0;

	switch(i)
	{
		case 0:

			if (spiRsp) curDsc = readyRspWave.Get();

			if (curDsc != 0)
			{
				spi.SetMode(CPHA|CPOL);
				spi.WriteAsyncDMA(curDsc->data, curDsc->dataLen*2);

				i++;
			};

			break;

		case 1:

			if (spi.CheckWriteComplete())
			{
				if (curDsc != 0)
				{
					freeRspWave.Add(curDsc);
					
					curDsc = 0;
				};

				tm.Reset();

				i++;
			};

			break;

		case 2:

			if (tm.Check(US2RT(200)))
			{
				i = 0;
			};

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateBlackFin()	);
		CALL( UpdateHardware()	);
		CALL( UpdateSPI()		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void Filtr_Data(RSPWAVE &dsc, u32 filtrType)
{
	RspCM &rsp = *((RspCM*)dsc.data);
	u16 *d = rsp.data;

	if (filtrType == 1 && rsp.hdr.sensType < 2)
	{
		i32 *ab = avrBuf[rsp.hdr.sensType]; 

		for (u32 i = rsp.hdr.sl; i > 0; i--)
		{
			i16 v = d[0];

			*(d++) = v -= *ab/32;

			*(ab++) += v;
		};
	}
	else if (filtrType == 2)
	{
		//i32 av = 0;

		for (u32 i = rsp.hdr.sl; i > 0; i--)
		{
			i16 v = (d[0] + d[1])/2;
			*(d++) = v;
		};
	}
	else if (filtrType == 3)
	{
		i32 av = 0;
		//i32 *ab = avrBuf;

		for (u32 i = rsp.hdr.sl; i > 0; i--)
		{
			i16 v = (d[2] - d[0] + d[3] - d[1])/4;
			*(d++) = v;
		};
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
		i16 *p = d+rsp.hdr.sl;

		for (i32 i = ArraySize(wavelet_Table); i > 0; i--) *(p++) = 0;

		d += descrIndx;

		for (i32 i = rsp.hdr.sl - descrIndx; i > 0 ; i--)
		{
			i32 sum = 0;

			for (i32 j = 0; j < ArraySize(wavelet_Table); j += 2)
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
	}
	else
	{
		rsp.hdr.fi_time  = ~0;
		rsp.hdr.fi_amp = 0;
		rsp.hdr.maxAmp = 0;
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void GetAmpTimeIM_3(RSPWAVE &dsc, u16 ind, u16 imThr)
{
	RspCM &rsp = *((RspCM*)dsc.data);
	rsp.hdr.fi_amp = 0;
	rsp.hdr.fi_time = ~0;
	rsp.hdr.packLen = ind;

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void ProcessDataCM(RSPWAVE *dsc)
{
	RspCM *rsp = (RspCM*)dsc->data;

	rsp->data[rsp->hdr.sl] = GetCRC16(&rsp->hdr, sizeof(rsp->hdr));
	dsc->dataLen += 1;

	readyRspWave.Add(dsc);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma optimize_for_speed

static void SendReadyDataIM(RSPWAVE *dsc, u16 len)
{
	RspIM *rsp = (RspIM*)dsc->data;

	rsp->hdr.rw			= manReqWord|0x50;	//1. ответное слово
	//rsp->mmsecTime	= dsc->mmsec;		//2. Время (0.1мс). младшие 2 байта
	//rsp->shaftTime	= dsc->shaftTime;	//4. Время датчика Холла (0.1мс). младшие 2 байта
	//rsp->ax			= dsc->ax;
	//rsp->ay			= dsc->ay;
	//rsp->az			= dsc->az;
	//rsp->at			= dsc->at;
	//rsp->gain		= dsc->gain;		//10. КУ
	rsp->hdr.refAmp		= refAmp;
	rsp->hdr.refTime	= refTime;
	rsp->hdr.len		= len;				//11. Длина (макс 1024)

	rsp->data[len*2]	= GetCRC16(&rsp->hdr, sizeof(rsp->hdr));

	//dsc->offset = (sizeof(*rsp) - sizeof(rsp->data)) / 2;
	dsc->dataLen = (sizeof(RspIM)-sizeof(rsp->data))/2 + len*2 + 1;

	readyRspWave.Add(dsc);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma optimize_for_speed

static void ProcessDataIM(RSPWAVE *dsc)
{
	static RSPWAVE *imdsc = 0;
	static u32 count = 0;
	static u32 cmCount = 0;
	static u32 i = 0;
	static u16 prevShaftCount = 0;
	static u16 wpr = 180;

	const RspCM &rsp = *((RspCM*)dsc->data);

	if (dsc->shaftCount != prevShaftCount)
	{
		prevShaftCount = dsc->shaftCount;

		if (imdsc != 0)
		{
			SendReadyDataIM(imdsc, i);

			imdsc = 0;
		};
	};

	if (imdsc == 0)
	{
		wpr = wavesPerRoundIM;
		count = wpr*9/8; if (count > 512) count = 512;
		cmCount = (wpr+8) / 16;
		i = 0;

		imdsc = freeRspWave.Get();

		if (imdsc != 0)
		{
			RspIM *ir = (RspIM*)imdsc->data;

			ir->hdr.mmsecTime	= rsp.hdr.mmsecTime;
			ir->hdr.shaftTime	= rsp.hdr.shaftTime;
			ir->hdr.gain		= rsp.hdr.gain;
			ir->hdr.ax			= rsp.hdr.ax;
			ir->hdr.ay			= rsp.hdr.ay;
			ir->hdr.az			= rsp.hdr.az;
			ir->hdr.at			= rsp.hdr.at;

			u16 *d = ir->data;

			for (u32 i = 10; i > 0; i--) { *(d++) = 0; };
		};
	};

	if (rsp.hdr.sensType == 2)
	{
		ProcessDataCM(dsc);
	}
	else 
	{
		if (imdsc != 0)
		{
			RspIM *ir = (RspIM*)imdsc->data;

			u16 *data = ir->data + i*2;

			if (dsc->fireIndex < count)
			{
				while (i < dsc->fireIndex)
				{
					*(data++) = 0;
					*(data++) = 0;
					i++;
				};
			};

			if (i < count)
			{
				*(data++) = rsp.hdr.fi_amp;
				*(data++) = rsp.hdr.fi_time;
				i++;
			};

			if (i >= count)
			{
				SendReadyDataIM(imdsc, count);

				imdsc = 0;
			};
		};

		if (cmCount == 0)
		{
			cmCount = (wpr+4) / 8;

			ProcessDataCM(dsc);
		}
		else
		{
			freeRspWave.Add(dsc);
		};

		cmCount -= 1;
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void ProcessSPORT()
{
	static byte state = 0;
	static DSCSPORT *dsc = 0;
	static RSPWAVE  *rsp = 0;
	static u16 angle = 0;

	switch (state)
	{
		case 0:

			dsc = GetDscSPORT();

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
				rsp->mode = 0;
				rsp->dataLen = sizeof(RspHdrCM)/2+dsc->len;
				rsp->fireIndex = dsc->fireIndex;
				rsp->shaftCount = dsc->shaftCount;

				u32 t = (72000 * dsc->rotCount + 74) / 147;

				if (t >= 36000) t -= 36000;

				angle = t;

				RspCM &r = *((RspCM*)rsp->data);
				//RspHdrCM *r = (RspHdrCM*)rsp->data;

				r.hdr.rw		= manReqWord|0x40;			//1. ответное слово
				r.hdr.mmsecTime	= dsc->mmsec;
				r.hdr.shaftTime	= dsc->shaftTime;
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
		
		case 2:
		{
			RspCM &r = *((RspCM*)rsp->data);
			u16 *d = r.data;

			if ((dsc->chMask&3) == 1)
			{
				u16 *s = dsc->data+1;

				for (u32 i = dsc->len+4; i > 0; i--)
				{
					*(d++) = s[0] - 2048; s++;
				};

				FreeDscSPORT(dsc);

				Pin_ProcessSPORT_Clr();

				state = 0;
			}
			else if (dsc->chMask & 1)
			{
				u16 *s = dsc->data+1;

				for (u32 i = dsc->len+4; i > 0; i--)
				{
					*(d++) = s[0] - 2048; s += 2;
				};

				state += 1;
			}
			else // chMask == 2
			{
				u16 *s = dsc->data+3;

				for (u32 i = dsc->len+4; i > 0; i--)
				{
					*(d++) = s[0] - 2048; s += 2;
				};

				FreeDscSPORT(dsc);

				Pin_ProcessSPORT_Clr();

				state = 0;
			};

			processWave.Add(rsp);

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
				rsp->mode = 0;
				rsp->dataLen = sizeof(RspHdrCM)/2+dsc->len;
				rsp->fireIndex = dsc->fireIndex;
				rsp->shaftCount = dsc->shaftCount;

				RspCM &r = *((RspCM*)rsp->data);
				//RspHdrCM *r = (RspHdrCM*)rsp->data;

				r.hdr.rw		= manReqWord|0x40;			//1. ответное слово
				r.hdr.mmsecTime	= dsc->mmsec;
				r.hdr.shaftTime	= dsc->shaftTime;
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
			}

		case 4:
		{
			RspCM &r = *((RspCM*)rsp->data);
			u16 *d = r.data;
			u16 *s = dsc->data+3;

			for (u32 i = dsc->len+4; i > 0; i--)
			{
				*(d++) = s[0] - 2048; s += 2;
			};

			FreeDscSPORT(dsc);
			processWave.Add(rsp);

			Pin_ProcessSPORT_Clr();

			state = 0;

			break;
		};
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


static void UpdateMode()
{
	ProcessSPORT();

	RSPWAVE *dsc = processWave.Get();

	if (dsc != 0)
	{
		Pin_UpdateMode_Set();

		RspHdrCM *rsp = (RspHdrCM*)dsc->data;

		u16 n = rsp->sensType;

		const SensVars &sens = sensVars[n];

		Filtr_Data(*dsc, sens.filtr);

		if (sens.fi_type != 0)
		{
			Filtr_Wavelet(*dsc, sens.descrIndx);
		}
		else
		{
			GetAmpTimeIM_3(*dsc, sens.descrIndx, sens.thr);
		};
			
		if (n != 2)
		{
			switch (mode)
			{
				case 0: ProcessDataCM(dsc); break;
				case 1: ProcessDataIM(dsc); break;
			};
		}
		else
		{
			refAmp	= rsp->fi_amp;
			refTime = rsp->fi_time;
			
			ProcessDataCM(dsc);
		};

		//idle();
		Pin_UpdateMode_Clr();
	}
	else
	{
		Update();
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//i16 index_max = 0;

//byte buf[100];

int main( void )
{
	static byte s = 0;
	static byte i = 0;

	static u32 pt = 0;

	static RTM32 tm;

	InitHardware();

	com.Connect(12500000, 0);

	//CheckFlash();

	spi.Connect(25000000);

	for (u16 i = 0; i < ArraySize(rspWaveBuf); i++)
	{
		freeRspWave.Add(rspWaveBuf+i);
	};

	while (1)
	{
		Pin_MainLoop_Set();

		UpdateMode();

		Pin_MainLoop_Clr();
	};

//	return 0;
}


