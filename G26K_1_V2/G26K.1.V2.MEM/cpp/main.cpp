#include "hardware.h"
//#include "options.h"
//#include "hw_emac.h"
#include "EMAC\xtrap.h"
#include "FLASH\NandFlash.h"
#include "CRC\CRC16.h"
#include "CRC\CRC16_CCIT.h"
#include "req.h"
#include "list.h"
#include "PointerCRC.h"
#include "SEGGER_RTT\SEGGER_RTT.h"
#include "hw_com.h"
#include "TaskList.h"
#include "BOOT\boot_req.h"

#ifdef WIN32

#include <conio.h>
//#include <stdio.h>

static const bool __WIN32__ = true;

#else

static const bool __WIN32__ = false;

//#pragma diag_suppress 546,550,177

#endif

#define __TEST__

enum { VERSION = 0x201 };

//#pragma O3
//#pragma Otime

#ifndef _DEBUG
	static const bool __debug = false;
#else
	static const bool __debug = true;
#endif

#define SENS_NUM	3
#define NS2DSP(v) (((v)+10)/20)
#define US2DSP(v) ((((v)*1000)+10)/20)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#ifdef CPU_SAME53
//
//	#define FPS_PIN_SET()	HW::PIOA->BSET(25)
//	#define FPS_PIN_CLR()	HW::PIOA->BCLR(25)
//
//#elif defined(CPU_XMC48)
//
//	#define FPS_PIN_SET()	HW::P2->BSET(13)
//	#define FPS_PIN_CLR()	HW::P2->BCLR(13)
//
//#elif defined(WIN32)
//
//	#define FPS_PIN_SET()	
//	#define FPS_PIN_CLR()	
//
//#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct MainVars // NonVolatileVars  
{
	u32 timeStamp;

	u16 numDevice;
	u16 numMemDevice;

	SENS	sens1;
	SENS	sens2;
	SENS	refSens;

	u16 cmSPR;
	u16 imSPR;
	u16 fireVoltage;
	u16 motoLimCur;
	u16 motoMaxCur;
	u16 sensMask;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static MainVars mv;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 req40_count1 = 0;
u32 req40_count2 = 0;
u32 req40_count3 = 0;

u32 fps;
i16 tempClock = 0;
i16 cpu_temp = 0;
u32 i2cResetCount = 0;

//inline u16 ReverseWord(u16 v) { __asm	{ rev16 v, v };	return v; }

//static void* eepromData = 0;
//static u16 eepromWriteLen = 0;
//static u16 eepromReadLen = 0;
//static u16 eepromStartAdr = 0;

//static MTB mtb;

//static u16 manBuf[16];

u16 manRcvData[10];
u16 manTrmData[50];
static u16 manTrmBaud = 0;
static u16 memTrmBaud = 0;

u16 txbuf[128 + 512 + 16];

//static ComPort comdsp;
//static ComPort commoto;

static RequestQuery qmoto(&commoto);
static RequestQuery qdsp(&comdsp);
//static RequestQuery qmem(&commem);

//static R01 r02[8];

static Ptr<MB> manVec40[SENS_NUM];

static Ptr<MB> curManVec40;
static Ptr<MB> manVec50[SENS_NUM-1];
static Ptr<MB> curManVec50;

//static RspMan60 rspMan60;

//static byte curRcv[3] = {0};
//static byte curVec[3] = {0};

//static List<R01> freeR01;
//static List<R01> readyR01;
static ListPtr<MB> readyR01;

//static RMEM rmem[4];
//static List<RMEM> lstRmem;
//static List<RMEM> freeRmem;

//static byte fireType = 0;

//static u16 gain = 0;
//static u16 sampleTime = 5;
//static u16 sampleLen = 1024;
//static u16 sampleDelay = 200;
//static u16 deadTime = 400;
//static u16 descriminant = 400;
//static u16 freq = 500;

//static u16 gainRef = 0;
//static u16 sampleTimeRef = 5;
//static u16 sampleLenRef = 1024;
//static u16 sampleDelayRef = 200;
//static u16 deadTimeRef = 400;
//static u16 descriminantRef = 400;
//static u16 refFreq = 500;
//static u16 filtrType = 0;
//static u16 packType = 0;
//static u16 vavesPerRoundCM = 100;	
//static u16 vavesPerRoundIM = 100;

static u16 mode = 0;

static TM32 imModeTimeout;

//static u16 motoEnable = 0;		// ��������� �������� ��� ���������
static u16 motoTargetRPS = 0;		// �������� ������� ���������
static u16 motoRPS = 0;				// ������� ���������, ��/���
static u16 motoCur = 0;				// ��� ���������, 0...9400 ��
static u16 motoCurLow = 0;			// ��� ���������, 0...4500 ��
static u16 motoStat = 0;			// ������ ���������
static u16 motoCounter = 0;			// ������� �������� ��������� 1/6 �������
//static u16 cmSPR = 32;			// ���������� �������� ������ �� ������ ������� � ������ �����������
//static u16 imSPR = 100;			// ���������� ����� �� ������ ������� � ������ ��������
//static u16 *curSPR = &cmSPR;		// ���������� ��������� ���������� �� ������ � ������� ������
static u16 auxVoltage = 0;
static u16 motoVoltage = 90;
static u16 motoRcvCount = 0;
static u16 motoDuty = 0;			// 0 ... 10000 (10000 = 100%) 

static u16 curFireVoltage = 500;

//static u32 dspMMSEC = 0;
//static u32 shaftMMSEC = 0;

const u16 dspReqWord = 0xAD00;
//const u16 dspReqMask = 0xFF00;

static u16 manReqWord = 0xAD00;
static u16 manReqMask = 0xFF00;

static u16 memReqWord = 0x3D00;
static u16 memReqMask = 0xFF00;

//static u16 numDevice = 0;
static u16 verDevice = VERSION;

//static u16 numMemDevice = 0;
static u16 verMemDevice = 0x300;

//static u32 manCounter = 0;
//static u32 fireCounter = 0;

static byte mainModeState = 0;
static byte dspStatus = 0;

static bool cmdWriteStart_00 = false;
static bool cmdWriteStart_10 = false;
static bool cmdWriteStart_20 = false;

static u32 dspRcv40 = 0;
static u32 dspRcv50 = 0;
static u16 dspRcvCount = 0;
static u32 dspRcvErr = 0;
static u16 dspNotRcv = 0;


//static u32 rcvCRCER = 0;

//static u32 chnlCount[4] = {0};

//static u32 crcErr02 = 0;
//static u32 crcErr03 = 0;
static u32 crcErr06 = 0;
static u32 wrtErr06 = 0;

static u32 notRcv02 = 0;
//static u32 lenErr02 = 0;

static i16 ax = 0, ay = 0, az = 0, at = 0;
i16 temperature = 0;
i16 cpuTemp = 0;
i16 temp = 0;

static byte svCount = 0;


struct AmpTimeMinMax { bool valid; u16 ampMax, ampMin, timeMax, timeMin; };

static AmpTimeMinMax sensMinMax[SENS_NUM]		= { {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0} };
static AmpTimeMinMax sensMinMaxTemp[SENS_NUM]	= { {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0} };

static u32 testDspReqCount = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SaveMainParams()
{
	svCount = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetNumDevice(u16 num)
{
	mv.numDevice = num;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetNumDevice()
{
	return mv.numDevice;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetVersionDevice()
{
	return verDevice;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update_RPS_SPR()
{
	Set_Sync_Rot(motoTargetRPS, (mode == 0) ? mv.cmSPR : mv.imSPR);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetModeCM()
{
	if (mode != 0)
	{
		mode = 0;

		Update_RPS_SPR();
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetModeIM()
{
	if (mode == 0)
	{
		mode = 1;

		Update_RPS_SPR();
	};

	imModeTimeout.Reset();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void Response_0(u16 rw, MTB &mtb)
//{
//	__packed struct Rsp {u16 rw; u16 device; u16 session; u32 rcvVec; u32 rejVec; u32 wrVec; u32 errVec; u16 wrAdr[3]; u16 numDevice; u16 version; u16 temp; byte status; byte flags; RTC rtc; };
//
//	Rsp &rsp = *((Rsp*)&txbuf);
//
//	rsp.rw = rw;
//	rsp.device = GetDeviceID();  
//	rsp.session = FLASH_Session_Get();	  
//	rsp.rcvVec =  FLASH_Vectors_Recieved_Get();
//	rsp.rejVec = FLASH_Vectors_Rejected_Get();
//	rsp.wrVec = FLASH_Vectors_Saved_Get();
//	rsp.errVec = FLASH_Vectors_Errors_Get();
//	*((__packed u64*)rsp.wrAdr) = FLASH_Current_Adress_Get();
//	rsp.temp = temp*5/2;
//	rsp.status = FLASH_Status();
//
//	GetTime(&rsp.rtc);
//
//	mtb.data1 = txbuf;
//	mtb.len1 = sizeof(rsp)/2;
//	mtb.data2 = 0;
//	mtb.len2 = 0;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackDspReq01(Ptr<REQ> &q)
{
	RspDsp01 &rsp = *((RspDsp01*)q->rb.data);
	 
	if (q->rb.recieved)
	{
		if (rsp.CM.hdr.rw == (dspReqWord|0x40))
		{
			if (rsp.CM.hdr.packType == 0)
			{
				q->crcOK = (q->rb.len == (rsp.CM.hdr.sl*2 + sizeof(rsp.CM.hdr)+2)) && (GetCRC16(&rsp.CM.hdr, sizeof(rsp.CM.hdr)) == rsp.CM.data[rsp.CM.hdr.sl]);
			}
			else
			{
				q->crcOK = (q->rb.len == (rsp.CM.hdr.packLen*2 + sizeof(rsp.CM.hdr) + 2)) && (GetCRC16(&rsp.CM.hdr, sizeof(rsp.CM.hdr)) == rsp.CM.data[rsp.CM.hdr.packLen]);
			};

			q->rsp->len = q->rb.len;

			dspStatus |= 1;
			dspRcv40++;
			dspRcvCount++;
			
			//dspMMSEC	= rsp.CM.hdr.time;
			//shaftMMSEC	= rsp.CM.hdr.hallTime;
		}
		else if (rsp.IM.hdr.rw == (dspReqWord|0x50))
		{
			q->crcOK = (q->rb.len == (rsp.IM.hdr.dataLen*4 + sizeof(rsp.IM.hdr) + 2)) && (GetCRC16(&rsp.IM.hdr, sizeof(rsp.IM.hdr)) == rsp.IM.data[rsp.IM.hdr.dataLen*2]);
			q->rsp->len = q->rb.len;

			dspStatus |= 1;
			dspRcv50++;
			dspRcvCount++;

			//dspMMSEC = rsp.IM.hdr.time;
			//shaftMMSEC = rsp.IM.hdr.hallTime;
		}
		else
		{
			q->crcOK = GetCRC16(q->rb.data, q->rb.len) == 0;

			if (q->crcOK && rsp.v01.rw == (dspReqWord|1))
			{
				curFireVoltage = rsp.v01.fireVoltage;
				//motoVoltage = rsp.v01.motoVoltage;
				
				dspStatus |= 1;
				dspRcvCount++;

				q->rsp->len = 0;
				q->crcOK = false;
				q->tryCount = 0;
			};
		};
	}
	else
	{
		q->crcOK = false;
		notRcv02++;
	};

	if (!q->crcOK)
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qdsp.Add(q);
		}
		else
		{
			dspStatus &= ~1; 

			//q.Free();
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Ptr<REQ> CreateDspReq01(u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());

	if (!rq.Valid()) 
	{
		rq.Free();
		return rq;
	};

	rq->rsp = NandFlash_AllocWB(sizeof(RspDsp01)+2);

	if (!rq->rsp.Valid())
	{ 
		rq.Free();
		return rq; 
	};

	ReqDsp01 &req = *((ReqDsp01*)rq->reqData);
	RspDsp01 &rsp = *((RspDsp01*)(rq->rsp->GetDataPtr()));

	rq->rsp->len = 0;
	
	REQ &q = *rq;

	q.CallBack = CallBackDspReq01;
	//q.rb = &rb;
	//q.wb = &wb;
	q.preTimeOut = MS2COM(1);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	//q.ptr = &r;
	q.checkCRC = false;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = &rsp;
	q.rb.maxLen = rq->rsp->GetDataMaxLen();
	q.rb.recieved = false;
	
	req.rw				= dspReqWord|1;
	req.len				= sizeof(req);
	req.version			= req.VERSION;
	req.mode 			= mode;
	req.ax				= ax;
	req.ay				= ay;
	req.az				= az;
	req.at				= at;
	req.sens[0]			= mv.sens1;
	req.sens[1]			= mv.sens2;
	req.sens[2]			= mv.refSens;
	req.wavesPerRoundCM = mv.cmSPR;
	req.wavesPerRoundIM = mv.imSPR;
	req.fireVoltage		= mv.fireVoltage;
	req.sensMask		= mv.sensMask;

	req.crc	= GetCRC16(&req, sizeof(ReqDsp01)-2);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Ptr<MB> CreateTestDspReq01()
{
	Ptr<MB> rq;
	
	rq = NandFlash_AllocWB(sizeof(RspDsp01));

	if (!rq.Valid()) { return rq; };

	RspDsp01 &rsp = *((RspDsp01*)(rq->GetDataPtr()));

	rsp.CM.hdr.rw = manReqWord|0x40;
	rsp.CM.hdr.time = 1;
	rsp.CM.hdr.hallTime = 2;
	rsp.CM.hdr.motoCount = 3;
	rsp.CM.hdr.headCount = 4;
	rsp.CM.hdr.ax = 5;
	rsp.CM.hdr.ay = 6;
	rsp.CM.hdr.az = 7;
	rsp.CM.hdr.at = 8;
	rsp.CM.hdr.sensType = 0;
	rsp.CM.hdr.angle = 9;
	rsp.CM.hdr.maxAmp = 10;
	rsp.CM.hdr.fi_amp = 11;
	rsp.CM.hdr.fi_time = 12;
	rsp.CM.hdr.gain = 13;
	rsp.CM.hdr.st = 14;
	rsp.CM.hdr.sl = ArraySize(rsp.CM.data);
	rsp.CM.hdr.sd = 0;
	rsp.CM.hdr.packType = 0;
	rsp.CM.hdr.packLen = 0;

	for (u32 i = 0; i < rsp.CM.hdr.sl; i++)
	{
		rsp.CM.data[i] = 0;
	};

	rq->len = sizeof(rsp.CM);
	
	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackDspBootReq00(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qdsp.Add(q);

			return false;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateDspBootReq00(u16 adr, BootRspV1::SF0 *rspdata, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	if (rspdata == 0 && adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(*rspdata));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
		rspdata = (BootRspV1::SF0*)(rq->rsp->GetDataPtr());
	};

	BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->reqData);
	BootRspV1::SF0 &rsp = *rspdata;
	
	REQ &q = *rq;

	q.CallBack = CallBackDspBootReq00;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(500);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= DSP_BOOT_REQ_WORD|0;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackDspBootReq01(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qdsp.Add(q);

			return false;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateDspBootReq01(u16 adr, u32 len, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF1));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF1 &req = *((BootReqV1::SF1*)rq->reqData);
	BootRspV1::SF1 &rsp = *((BootRspV1::SF1*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackDspBootReq01;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(500);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= DSP_BOOT_REQ_WORD|1;
	req.len = len;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackDspBootReq02(Ptr<REQ> &q)
{
	bool retry = false;

	if (!q->crcOK) 
	{
		//crcErr06++;

		retry = true;
	}
	else
	{
		BootRspV1::SF2 &rsp = *((BootRspV1::SF2*)q->rb.data);

		if (rsp.res == 0) /*wrtErr06++,*/ retry = true;
	};

	if (retry && q->tryCount > 0)
	{
		q->tryCount--;
		qdsp.Add(q);

		return false;
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateDspBootReq02(u16 adr, u16 stAdr, u16 count, void* data, u16 count2, void* data2, u16 pageLen, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp = AllocMemBuffer(sizeof(BootReqV1::SF2) + pageLen);

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	BootRspV1::SF2 &rsp = *((BootRspV1::SF2*)rq->reqData);
	BootReqV1::SF2 &req = *((BootReqV1::SF2*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackDspBootReq02;
	q.preTimeOut = MS2COM(2);
	q.postTimeOut = US2COM(500);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= DSP_BOOT_REQ_WORD|2;

	u16 max = pageLen;

	if (count > max)
	{
		count = max;
		count2 = 0;
	}
	else if ((count + count2) > max)
	{
		count2 = max - count;
	};

	req.padr = stAdr;
	req.plen = count+count2;

	byte *d = (byte*)req.pdata;
	byte *s = (byte*)data;

	while(count > 0) *d++ = *s++, count--;

	if (data2 != 0)	{ s = (byte*)data2;	while(count2 > 0) *d++ = *s++, count2--; };

	u16 len = sizeof(req) - sizeof(req.pdata) + req.plen;

	u16 crc = GetCRC16(&req, len);

	*(d++) = crc;
	*(d++) = crc>>8;

	q.wb.data = &req;
	q.wb.len = len+2;

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackDspBootReq03(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->rb.len >=4)
		{
			BootReqV1::SF3 &req = *((BootReqV1::SF3*)q->wb.data);
			BootRspV1::SF3 &rsp = *((BootRspV1::SF3*)q->rb.data);

			if (rsp.adr == req.adr && rsp.rw == req.rw) return q->crcOK = true;
		};

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qdsp.Add(q);

			return false;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateDspBootReq03(u16 adr, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF3));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF3 &req = *((BootReqV1::SF3*)rq->reqData);
	BootRspV1::SF3 &rsp = *((BootRspV1::SF3*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackDspBootReq03;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(500);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= DSP_BOOT_REQ_WORD|3;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackMotoReq(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qmoto.Add(q);

			return false;
		};
	}
	else
	{
		RspMoto &rsp = *((RspMoto*)q->rb.data);

		if (rsp.rw == 0x101)
		{
			motoRPS		= rsp.rpm;
			motoCur		= rsp.current;
			motoCurLow	= rsp.currentLow;
			motoStat	= rsp.mororStatus;
			motoCounter = rsp.motoCounter;
			auxVoltage	= rsp.auxVoltage;
			motoVoltage	= rsp.motoVoltage;
			motoDuty	= rsp.motoDuty;
			motoRcvCount++;
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateMotoReq()
{
	Ptr<REQ> rq(AllocREQ());

	if (!rq.Valid()) return rq;

	rq->rsp = AllocMemBuffer(sizeof(RspMoto));

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqMoto &req = *((ReqMoto*)rq->reqData);
	RspMoto &rsp = *((RspMoto*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackMotoReq;
	//q.rb = &rb;
	//q.wb = &wb;
	q.preTimeOut = MS2COM(1);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.checkCRC = true;
	q.updateCRC = false;
	q.tryCount = 1;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);
	
	req.rw = 0x101;
	req.enableMotor	= 1;
	req.tRPM = motoTargetRPS;
	req.limCurrent = mv.motoLimCur;
	req.maxCurrent = mv.motoMaxCur;
	req.crc	= GetCRC16(&req, sizeof(req)-2);

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackBootMotoReq(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qmoto.Add(q);

			return false;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateBootMotoReq00(u16 adr, BootRspV1::SF0 *rspdata, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());

	if (!rq.Valid()) return rq;

	if (rspdata == 0 && adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(*rspdata));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
		rspdata = (BootRspV1::SF0*)(rq->rsp->GetDataPtr());
	};

	BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->reqData);
	BootRspV1::SF0 &rsp = *rspdata;

	REQ &q = *rq;

	q.CallBack = CallBackBootMotoReq;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(500);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;

	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= MOTO_BOOT_REQ_WORD|0;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateBootMotoReq01(u16 adr, u32 len, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());

	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF1));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF1 &req = *((BootReqV1::SF1*)rq->reqData);
	BootRspV1::SF1 &rsp = *((BootRspV1::SF1*)(rq->rsp->GetDataPtr()));

	REQ &q = *rq;

	q.CallBack = CallBackBootMotoReq;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(500);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;

	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= MOTO_BOOT_REQ_WORD|1;
	req.len = len;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateBootMotoReq02(u16 adr, u16 stAdr, u16 count, void* data, u16 pageLen, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());

	if (!rq.Valid()) return rq;

	rq->rsp = AllocMemBuffer(sizeof(BootReqV1::SF2) + pageLen);

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	BootRspV1::SF2 &rsp = *((BootRspV1::SF2*)rq->reqData);
	BootReqV1::SF2 &req = *((BootReqV1::SF2*)(rq->rsp->GetDataPtr()));

	REQ &q = *rq;

	q.CallBack = CallBackBootMotoReq;
	q.preTimeOut = MS2COM(300);
	q.postTimeOut = US2COM(500);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;

	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= MOTO_BOOT_REQ_WORD|2;

	u16 max = pageLen;

	if (count > max)
	{
		count = max;
		//		count2 = 0;
	}
	//else if ((count + count2) > max)
	//{
	//	count2 = max - count;
	//};

	req.padr = stAdr;
	req.plen = count;//+count2;

	byte *d = (byte*)req.pdata;
	byte *s = (byte*)data;

	while(count > 0) *d++ = *s++, count--;

	//if (data2 != 0)	{ s = (byte*)data2;	while(count2 > 0) *d++ = *s++, count2--; };

	u16 len = sizeof(req) - sizeof(req.pdata) + req.plen;

	u16 crc = GetCRC16(&req, len);

	*(d++) = crc;
	*(d++) = crc>>8;

	q.wb.data = &req;
	q.wb.len = len+2;

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateBootMotoReq03(u16 adr, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());

	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF3));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF3 &req = *((BootReqV1::SF3*)rq->reqData);
	BootRspV1::SF3 &rsp = *((BootRspV1::SF3*)(rq->rsp->GetDataPtr()));

	REQ &q = *rq;

	q.CallBack = CallBackBootMotoReq;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(500);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;

	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= MOTO_BOOT_REQ_WORD|3;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateBootMotoReq04(u16 adr, u32 timeOut, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());

	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF4));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF4 &req = *((BootReqV1::SF4*)rq->reqData);
	BootRspV1::SF4 &rsp = *((BootRspV1::SF4*)(rq->rsp->GetDataPtr()));

	REQ &q = *rq;

	q.CallBack = CallBackBootMotoReq;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(500);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;

	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr			= adr;
	req.rw			= MOTO_BOOT_REQ_WORD|4;
	req.timeOutMS	= timeOut;
	req.crc			= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_00(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= (manReqWord & manReqMask) | 0;
	*(data++)	= mv.numDevice;
	*(data++)	= verDevice;
	
	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_00(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_00(data) * 2;

	NandFlash_RequestWrite(flwb, data[0], true);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	InitRspMan_00(manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = 3;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_10(__packed u16 *data)
{
	__packed u16 *start = data;

	union { __packed u16 *d; SENS *s; } u;

	u.d = data;

	*(u.d++)	= (manReqWord & manReqMask) | 0x10;		//	1. �������� �����
	*(u.s++)	= mv.sens1;								//	2. �� (������������� ������ 1)
	*(u.s++)	= mv.sens2;								//	13. �� (������������� ������ 2)
	*(u.s++)	= mv.refSens;							//	24. �� (������� ������)
	*(u.d++)	= mv.cmSPR;								//	35. ���������� �������� ������ �� ������ ������� � ������ �����������
	*(u.d++)	= mv.imSPR;								//	36. ���������� ����� �� ������ ������� � ������ ��������
	*(u.d++)	= mv.fireVoltage;						//	37. ���������� ���������� (�)
	*(u.d++)	= mv.motoLimCur;						//	38. ����������� ���� ��������� (��)
	*(u.d++)	= mv.motoMaxCur;						//	39. ��������� ��� ��������� (��)
	*(u.d++)	= mv.sensMask;							//	40. ����� �������������� ������� (��� 0 - ������������� ������ 1, ��� 1 - ������������� ������ 2), ���������� �������� 1,2,3
	
	return u.d - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_10(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_10(data) * 2;

	NandFlash_RequestWrite(flwb, data[0], true);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_10(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	len = InitRspMan_10(manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = len;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_20(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= manReqWord|0x20;				//	1.	�������� �����
	*(data++)  	= motoRPS;						//	2.	������� �������� ��������� (0.01 ��/���)
	*(data++)  	= motoCur;						//	3.	��� ��������� (��)
	*(data++)  	= motoCounter;					//	4.	������� �������� ��������� (1/6 ��)
	*(data++)  	= GetShaftRPS();				//	5.	������� �������� ������� (0.01 ��/���)
	*(data++)  	= GetShaftCount();				//	6.	������� �������� ������� (��)
	*(data++)  	= ax;							//	7.	AX (��)
	*(data++)  	= ay;							//	8.	AY (��)
	*(data++)  	= az;							//	9.	AZ (��)
	*(data++)  	= at;							//	10.	AT (short 0.01 ��)
	*(data++)	= temp;							//	11.	����������� � ������� (short)(0.1��)
	*(data++)	= sensMinMax[0].ampMax;			//	12.	��������� �������������� ������� 1 �������� �� ���� ����� (�.�)
	*(data++)	= sensMinMax[0].ampMin;			//	13.	��������� �������������� ������� 1 ������� �� ���� ����� (�.�)
	*(data++)	= sensMinMax[0].timeMax;		//	14.	����� �������������� ������� 1 �������� �� ������� ���������� (0.02 ���)
	*(data++)	= sensMinMax[0].timeMin;		//	15.	����� �������������� ������� 1 ������� �� ������� ���������� (0.02 ���)
	*(data++)	= sensMinMax[1].ampMax;			//	16.	��������� �������������� ������� 2 �������� �� ���� ����� (�.�)
	*(data++)	= sensMinMax[1].ampMin;			//	17.	��������� �������������� ������� 2 ������� �� ���� ����� (�.�)
	*(data++)	= sensMinMax[1].timeMax;		//	18.	����� �������������� ������� 2 �������� �� ������� ���������� (0.02 ���)
	*(data++)	= sensMinMax[1].timeMin;		//	19.	����� �������������� ������� 2 ������� �� ������� ���������� (0.02 ���)
	*(data++)	= sensMinMax[2].ampMax;			//	20.	��������� �������� ������� �������� �� ���� ����� (�.�)
	*(data++)	= sensMinMax[2].ampMin;			//	21.	��������� �������� ������� ������� �� ���� ����� (�.�)
	*(data++)	= sensMinMax[2].timeMax;		//	22.	����� �������� ������� �������� �� ������� ���������� (0.02 ���)
	*(data++)	= sensMinMax[2].timeMin;		//	23.	����� �������� ������� ������� �� ������� ���������� (0.02 ���)
	*(data++)	= GetShaftState();				//	24.	��������� ������� ����� (0, 1)
	*(data++)	= curFireVoltage;				//	25.	���������� ���������� (�)
	*(data++)	= motoVoltage;					//	26.	���������� ��������� (�)
	*(data++)	= auxVoltage;					//	27.	���������� 3-�� ���� (�)
	*(data++)	= dspRcvCount;					//	28.	������� �������� DSP
	*(data++)	= motoRcvCount;					//	29.	������� �������� ���������
	*(data++)	= GetRcvManQuality();			//	30.	�������� ������� ������� ���������� (%)
	*(data++)	= motoDuty;						//	31. ���������� ��� ��������� (0.01%)
	*(data++)	= motoCurLow;					//	32. ��� ���������, ������ 2 (��)

	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_20(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_20(data) * 2;

	NandFlash_RequestWrite(flwb, data[0], true);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	if (sensMinMaxTemp[0].valid) { sensMinMax[0] = sensMinMaxTemp[0]; };
	if (sensMinMaxTemp[1].valid) { sensMinMax[1] = sensMinMaxTemp[1]; };
	if (sensMinMaxTemp[2].valid) { sensMinMax[2] = sensMinMaxTemp[2]; };

	len = InitRspMan_20(manTrmData);

	sensMinMaxTemp[0].ampMax = 0;
	sensMinMaxTemp[0].ampMin = ~0;
	sensMinMaxTemp[0].timeMax = 0;
	sensMinMaxTemp[0].timeMin = ~0;
	sensMinMaxTemp[0].valid = false;

	sensMinMaxTemp[1].ampMax = 0;
	sensMinMaxTemp[1].ampMin = ~0;
	sensMinMaxTemp[1].timeMax = 0;
	sensMinMaxTemp[1].timeMin = ~0;
	sensMinMaxTemp[1].valid = false;

	sensMinMaxTemp[2].ampMax = 0;
	sensMinMaxTemp[2].ampMin = ~0;
	sensMinMaxTemp[2].timeMax = 0;
	sensMinMaxTemp[2].timeMin = ~0;
	sensMinMaxTemp[2].valid = false;

	mtb->data1 = manTrmData;
	mtb->len1 = len;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_40(u16 *data, u16 reqlen, MTB* mtb)
{
	__packed struct Req { u16 rw; u16 off; u16 len; };

	Req &req = *((Req*)data);

	if (data == 0 || reqlen == 0 || reqlen > 4 || mtb == 0) return false;

	//byte nf = ((req.rw>>4)-3)&3;
	//byte nr = req.rw & 7;

//	curRcv[nf] = nr;

	struct Rsp { u16 rw; };
	
	static Rsp rsp; 
	
	static u16 prevOff = 0;
	static u16 prevLen = 0;
	static u16 maxLen = 200;

	static byte sensInd = 0;

	rsp.rw = req.rw;

	mtb->data1 = (u16*)&rsp;
	mtb->len1 = sizeof(rsp)/2;
	mtb->data2 = 0;
	mtb->len2 = 0;

	//Ptr<MB> &r01 = curManVec40;
	
	//u16 sz = 18 + r01->rsp.CM.sl;

	if (reqlen == 1 || (reqlen >= 2 && data[1] == 0))
	{
		curManVec40 = manVec40[sensInd];

		manVec40[sensInd].Free();

		if (!curManVec40.Valid())
		{
			sensInd += 1; if (sensInd >= SENS_NUM) sensInd = 0;

			curManVec40 = manVec40[sensInd];

			manVec40[sensInd].Free();
		};

		if (curManVec40.Valid())
		{
			RspDsp01 &rsp = *((RspDsp01*)(curManVec40->GetDataPtr()));

			u16 sz = (sizeof(rsp.CM.hdr)-sizeof(rsp.CM.hdr.rw))/2 + ((rsp.CM.hdr.packType == 0) ? rsp.CM.hdr.sl : rsp.CM.hdr.packLen);

			mtb->data2 = ((u16*)&rsp)+1;

			prevOff = 0;

			if (reqlen == 1)
			{
				mtb->len2 = sz;
				prevLen = sz;
			}
			else 
			{
				req40_count1++;

				if (reqlen == 3) maxLen = data[2];

				u16 len = maxLen;

				if (len > sz) len = sz;

				mtb->len2 = len;

				prevLen = len;
			};
		};

		sensInd += 1; if (sensInd >= SENS_NUM) sensInd = 0;
	}
	else if (curManVec40.Valid())
	{
		RspDsp01 &rsp = *((RspDsp01*)(curManVec40->GetDataPtr()));

		req40_count2++;

		u16 off = prevOff + prevLen;
		u16 len = prevLen;

		if (reqlen == 3)
		{
			off = data[1];
			len = data[2];
		};

		u16 sz = (sizeof(rsp.CM.hdr)-sizeof(rsp.CM.hdr.rw))/2 + ((rsp.CM.hdr.packType == 0) ? rsp.CM.hdr.sl : rsp.CM.hdr.packLen);

		if (sz >= off)
		{
			req40_count3++;

			u16 ml = sz - off;

			if (len > ml) len = ml;

			mtb->data2 = (u16*)&rsp + data[1]+1;
			mtb->len2 = len;
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_30(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 3 || mtb == 0) return false;

	manTrmData[0] = data[0];	
 
	motoTargetRPS = (data[0]&15) * 100;
		
	//Set_Sync_Rot(motoTargetRPS, *curSPR);

	Update_RPS_SPR();

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_50(u16 *data, u16 reqlen, MTB* mtb)
{
	__packed struct Req { u16 rw; u16 off; u16 len; };

	Req &req = *((Req*)data);

	if (data == 0 || reqlen == 0 || reqlen > 4 || mtb == 0) return false;

	//byte nf = ((req.rw>>4)-3)&3;
	//byte nr = req.rw & 7;

//	curRcv[nf] = nr;

	struct Rsp { u16 rw; };
	
	static Rsp rsp; 
	
	static u16 prevOff = 0;
	static u16 prevLen = 0;
	static u16 maxLen = 200;

	static byte sensInd = 0;

	SetModeIM();

	rsp.rw = req.rw;

	mtb->data1 = (u16*)&rsp;
	mtb->len1 = sizeof(rsp)/2;
	mtb->data2 = 0;
	mtb->len2 = 0;

	if (reqlen == 1 || (reqlen >= 2 && data[1] == 0))
	{
		curManVec50 = manVec50[sensInd];

		manVec50[sensInd].Free();

		if (!curManVec50.Valid())
		{
			sensInd += 1; if (sensInd >= (SENS_NUM-1)) sensInd = 0;

			curManVec50 = manVec50[sensInd];

			manVec50[sensInd].Free();
		};

		if (curManVec50.Valid())
		{
			RspDsp01 &rsp = *((RspDsp01*)curManVec50->GetDataPtr());

			mtb->data2 = ((u16*)&rsp)+1;

			prevOff = 0;

			u16 sz = (sizeof(rsp.IM.hdr)-sizeof(rsp.IM.hdr.rw))/2 + rsp.IM.hdr.dataLen*2;

			if (reqlen == 1)
			{
				mtb->len2 = sz;
				prevLen = sz;
			}
			else 
			{
				if (reqlen == 3) maxLen = data[2];

				u16 len = maxLen;

				if (len > sz) len = sz;

				mtb->len2 = len;

				prevLen = len;
			};
		};

		sensInd += 1; if (sensInd >= (SENS_NUM-1)) sensInd = 0;
	}
	else if (curManVec50.Valid())
	{
		RspDsp01 &rsp = *((RspDsp01*)curManVec50->GetDataPtr());

		u16 off = prevOff + prevLen;
		u16 len = prevLen;
		u16 sz = (sizeof(rsp.IM.hdr)-sizeof(rsp.IM.hdr.rw))/2 + rsp.IM.hdr.dataLen*2;

		if (reqlen == 3)
		{
			off = data[1];
			len = data[2];
		};

		if (sz >= off)
		{
			u16 ml = sz - off;

			if (len > ml) len = ml;

			mtb->data2 = (u16*)&rsp + data[1]+1;
			mtb->len2 = len;
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_80(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch (data[1])
	{
		case 1:

			mv.numDevice = data[2];

			break;

		case 2:

			manTrmBaud = data[2] - 1;	//SetTrmBoudRate(data[2]-1);

			break;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0x80;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_90(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	cmdWriteStart_10 = true;

	switch(data[1])
	{
		case 0x1:	mv.sens1.gain			= data[2];			break;
		case 0x2:	mv.sens1.st				= data[2];			break;
		case 0x3:	mv.sens1.sl				= data[2];			break;
		case 0x4:	mv.sens1.sd			 	= data[2];			break;
		case 0x5:	mv.sens1.deadTime		= data[2];			break;
		case 0x6:	mv.sens1.descriminant	= data[2];			break;
		case 0x7:	mv.sens1.freq			= data[2];			break;
		case 0x8:	mv.sens1.filtrType		= data[2];			break;
		case 0x9:	mv.sens1.packType		= data[2];			break;
		case 0xA:	mv.sens1.fi_Type		= data[2];			break;
		case 0xB:	mv.sens1.fragLen		= data[2];			break;


		case 0x11:	mv.sens2.gain			= data[2];			break;
		case 0x12:	mv.sens2.st				= data[2];			break;
		case 0x13:	mv.sens2.sl				= data[2];			break;
		case 0x14:	mv.sens2.sd 			= data[2];			break;
		case 0x15:	mv.sens2.deadTime		= data[2];			break;
		case 0x16:	mv.sens2.descriminant	= data[2];			break;
		case 0x17:	mv.sens2.freq			= data[2];			break;
		case 0x18:	mv.sens2.filtrType		= data[2];			break;
		case 0x19:	mv.sens2.packType		= data[2];			break;
		case 0x1A:	mv.sens2.fi_Type		= data[2];			break;
		case 0x1B:	mv.sens2.fragLen		= data[2];			break;

		case 0x21:	mv.refSens.gain			= data[2];			break;
		case 0x22:	mv.refSens.st			= data[2];			break;
		case 0x23:	mv.refSens.sl			= data[2];			break;
		case 0x24:	mv.refSens.sd 			= data[2];			break;
		case 0x25:	mv.refSens.deadTime		= data[2];			break;
		case 0x26:	mv.refSens.descriminant	= data[2];			break;
		case 0x27:	mv.refSens.freq			= data[2];			break;
		case 0x28:	mv.refSens.filtrType	= data[2];			break;
		case 0x29:	mv.refSens.packType		= data[2];			break;
		case 0x2A:	mv.refSens.fi_Type		= data[2];			break;
		case 0x2B:	mv.refSens.fragLen		= data[2];			break;

		case 0x30:	mv.cmSPR 				= data[2]; Update_RPS_SPR();	break;
		case 0x31:	mv.imSPR 				= data[2]; Update_RPS_SPR();	break;

		case 0x40:	mv.fireVoltage			= data[2];			break;

		case 0x51:	mv.motoLimCur			= data[2];			break;
		case 0x52:	mv.motoMaxCur			= data[2];			break;

		case 0x60:	mv.sensMask				= data[2];			break;

		default:

			return false;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0x90;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	SaveMainParams();

	manTrmData[0] = (manReqWord & manReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan(u16 *buf, u16 len, MTB* mtb)
{
	if (buf == 0 || len == 0 || mtb == 0) return false;

	bool r = false;

	byte i = (buf[0]>>4)&0xF;

	switch (i)
	{
		case 0: 	r = RequestMan_00(buf, len, mtb); break;
		case 1: 	r = RequestMan_10(buf, len, mtb); break;
		case 2: 	r = RequestMan_20(buf, len, mtb); break;
		case 3:		r = RequestMan_30(buf, len, mtb); break;
		case 4:		r = RequestMan_40(buf, len, mtb); break;
		case 5: 	r = RequestMan_50(buf, len, mtb); break;
		case 8: 	r = RequestMan_80(buf, len, mtb); break;
		case 9:		r = RequestMan_90(buf, len, mtb); break;
		case 0xF:	r = RequestMan_F0(buf, len, mtb); break;
	};

	if (r) { mtb->baud = manTrmBaud; };

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0;
	manTrmData[1] = mv.numMemDevice;
	manTrmData[2] = verMemDevice;

	mtb->data1 = manTrmData;
	mtb->len1 = 3;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_10(u16 *data, u16 len, MTB* mtb)
{
	//__packed struct T { u16 g[8]; u16 st; u16 len; u16 delay; u16 voltage; };
	//__packed struct Rsp { u16 hdr; u16 rw; T t1, t2, t3; };
	
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0x10;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	__packed struct Rsp {u16 rw; u16 device; u16 session; u32 rcvVec; u32 rejVec; u32 wrVec; u32 errVec; u16 wrAdr[3]; u16 temp; byte status; byte flags; RTC rtc; };

	if (len != 1) return false;

	Rsp &rsp = *((Rsp*)&manTrmData);

	rsp.rw = (memReqWord & memReqMask)|0x20;
	rsp.device = NandFlash_GetDeviceID();  
	rsp.session = NandFlash_Session_Get();	  
	rsp.rcvVec =  NandFlash_Vectors_Recieved_Get();
	rsp.rejVec = dspRcvErr; //FLASH_Vectors_Rejected_Get();
	rsp.wrVec = NandFlash_Vectors_Saved_Get();
	rsp.errVec = NandFlash_Vectors_Errors_Get();
	*((__packed u64*)rsp.wrAdr) = NandFlash_Current_Adress_Get();
	rsp.temp = (temp+5)/10;
	rsp.status = NandFlash_Status();

	GetTime(&rsp.rtc);

	mtb->data1 = manTrmData;
	mtb->len1 = 20;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_30(u16 *data, u16 len, MTB* mtb)
{
	if (len != 5) return false;

	SetClock(*(RTC*)(&data[1]));

	manTrmData[0] = (memReqWord & memReqMask)|0x30;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_31(u16 *data, u16 len, MTB* mtb)
{
	if (len != 1) return false;

	cmdWriteStart_00 = cmdWriteStart_10 = NandFlash_WriteEnable();

	manTrmData[0] = (memReqWord & memReqMask)|0x31;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_32(u16 *data, u16 len, MTB* mtb)
{
	if (len != 1) return false;

	NandFlash_WriteDisable();

	manTrmData[0] = (memReqWord & memReqMask)|0x32;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_33(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	// Erase all

	manTrmData[0] = (memReqWord & memReqMask)|0x33;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_80(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch (data[1])
	{
		case 1:

			mv.numMemDevice = data[2];

			break;

		case 2:

			memTrmBaud = data[2] - 1;	//SetTrmBoudRate(data[2]-1);

			break;
	};

	manTrmData[0] = (memReqWord & memReqMask) | 0x80;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_90(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 1 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0x90;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	//SaveParams();

	manTrmData[0] = (memReqWord & memReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


static bool RequestMem(u16 *buf, u16 len, MTB* mtb)
{
	if (buf == 0 || len == 0 || mtb == 0) return false;

	bool r = false;

	byte i = buf[0]&0xFF;

	switch (i)
	{
		case 0x00: 	r = RequestMem_00(buf, len, mtb); break;
		case 0x10: 	r = RequestMem_10(buf, len, mtb); break;
		case 0x20: 	r = RequestMem_20(buf, len, mtb); break;
		case 0x30: 	r = RequestMem_30(buf, len, mtb); break;
		case 0x31: 	r = RequestMem_31(buf, len, mtb); break;
		case 0x32: 	r = RequestMem_32(buf, len, mtb); break;
		case 0x33: 	r = RequestMem_33(buf, len, mtb); break;
		case 0x80: 	r = RequestMem_80(buf, len, mtb); break;
		case 0x90: 	r = RequestMem_90(buf, len, mtb); break;
		case 0xF0: 	r = RequestMem_F0(buf, len, mtb); break;
	};

	if (r) { mtb->baud = memTrmBaud; };

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMan()
{
	static MTB mtb;
	static MRB mrb;

	static byte i = 0;

	static CTM32 tm;


//	u16 c;

	switch (i)
	{
		case 0:

//			HW::P5->BSET(7);

			mrb.data = manRcvData;
			mrb.maxLen = ArraySize(manRcvData);
			RcvManData(&mrb);

			i++;

			break;

		case 1:

			ManRcvUpdate();

			if (mrb.ready)
			{
				tm.Reset();

				if (mrb.OK && mrb.len > 0 &&	(((manRcvData[0] & manReqMask) == manReqWord && RequestMan(manRcvData, mrb.len, &mtb)) 
										||		((manRcvData[0] & memReqMask) == memReqWord && RequestMem(manRcvData, mrb.len, &mtb))))
				{
					//HW::P5->BCLR(7);

					i++;
				}
				else
				{
					i = 0;
				};
			}
			else if (mrb.len > 0)
			{

			};

			break;

		case 2:

			if (tm.Check(US2CTM(100)))
			{
				//manTrmData[0] = 1;
				//manTrmData[1] = 0;
				//mtb.len1 = 2;
				//mtb.data1 = manTrmData;
				SendManData(&mtb);

				i++;
			};

			break;

		case 3:

			if (mtb.ready)
			{
				i = 0;
			};

			break;

	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void MainMode()
{
	static Ptr<MB> mb;
	//static Ptr<MB> flwb;
	static TM32 tm;
	static RspDsp01 *rsp = 0;

	switch (mainModeState)
	{
		case 0:

			mb = readyR01.Get();

			if (mb.Valid())
			{
				rsp = (RspDsp01*)(mb->GetDataPtr());

				NandFlash_RequestWrite(mb, rsp->CM.hdr.rw, true);

				mainModeState++;
			};

			break;

		case 1:

			if ((rsp->CM.hdr.rw & 0xFF) == 0x40)
			{
				byte n = rsp->CM.hdr.sensType;

				if (n < SENS_NUM)
				{
					manVec40[n] = mb;

					AmpTimeMinMax& mm = sensMinMaxTemp[n];

					u16 amp		= rsp->CM.hdr.maxAmp;
					u16 time	= rsp->CM.hdr.fi_time;

					if (amp > mm.ampMax)	mm.ampMax = amp;
					if (amp < mm.ampMin)	mm.ampMin = amp;
					if (time > mm.timeMax)	mm.timeMax = time;
					if (time < mm.timeMin)	mm.timeMin = time;

					mm.valid = true;
				};
			}
			else if ((rsp->IM.hdr.rw & 0xFF) == 0x50)
			{
				byte n = rsp->IM.hdr.sensType;

				if (n < (SENS_NUM-1))
				{
					manVec50[n] = mb;
				};
			};

			if (imModeTimeout.Check(10000))
			{
				SetModeCM();
			};

			mb.Free();

			mainModeState++;

			break;

		case 2:

			if (cmdWriteStart_00)
			{
				Ptr<MB> b; b = NandFlash_AllocWB(6);

				if (b.Valid())
				{
					RequestFlashWrite_00(b);

					cmdWriteStart_00 = false;
				};
			}
			else if (cmdWriteStart_10)
			{
				Ptr<MB> b; b = NandFlash_AllocWB(44);

				if (b.Valid())
				{
					RequestFlashWrite_10(b);

					cmdWriteStart_10 = false;
				};
			}
			else if (cmdWriteStart_20)
			{
				Ptr<MB> b; b = NandFlash_AllocWB(60);

				if (b.Valid())
				{
					RequestFlashWrite_20(b);

					cmdWriteStart_20 = false;
				};
			}
			else if (tm.Check(1001))
			{
				cmdWriteStart_20 = true;
			};

			mainModeState = 0;

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static DSCSPI dscAccel;

//static i16 ax = 0, ay = 0, az = 0, at = 0;


static u8 txAccel[25] = { 0 };
static u8 rxAccel[50];

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool AccelReadReg(byte reg, u16 count)
{
	dscAccel.adr = (reg<<1)|1;
	dscAccel.alen = 1;
	//dscAccel.baud = 8000000;
	dscAccel.csnum = 0;
	dscAccel.wdata = 0;
	dscAccel.wlen = 0;
	dscAccel.rdata = rxAccel;
	dscAccel.rlen = count;

	return SPI_AddRequest(&dscAccel);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool AccelWriteReg(byte reg, u16 count)
{
	dscAccel.adr = (reg<<1)|0;
	dscAccel.alen = 1;
	dscAccel.csnum = 0;
	dscAccel.wdata = txAccel;
	dscAccel.wlen = count;
	dscAccel.rdata = 0;
	dscAccel.rlen = 0;

	return SPI_AddRequest(&dscAccel);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateAccel()
{
	static byte i = 0; 
	static i32 x = 0, y = 0, z = 0, t = 0;
	static i32 fx = 0, fy = 0, fz = 0;

	static TM32 tm;

	switch (i)
	{
		case 0:

			txAccel[0] = 0x52;
			AccelWriteReg(0x2F, 1); // Reset

			i++;

			break;

		case 1:

			if (dscAccel.ready)
			{
				tm.Reset();

				i++;
			};

			break;

		case 2:

			if (tm.Check(35))
			{
				AccelReadReg(0x1E, 18);

				i++;
			};

			break;

		case 3:

			if (dscAccel.ready)
			{
				txAccel[0] = 0;
				AccelWriteReg(0x28, 1); // FILTER SETTINGS REGISTER

				i++;
			};

			break;

		case 4:

			if (dscAccel.ready)
			{
				txAccel[0] = 0;
				AccelWriteReg(0x2D, 1); // CTRL Set PORST to zero

				i++;
			};

			break;

		case 5:

			if (dscAccel.ready)
			{
				AccelReadReg(0x2D, 1);

				i++;
			};

			break;

		case 6:

			if (dscAccel.ready)
			{
				if (rxAccel[0] != 0)
				{
					txAccel[0] = 0;
					AccelWriteReg(0x2D, 1); // CTRL Set PORST to zero
					i--; 
				}
				else
				{
					txAccel[0] = 0;
					AccelWriteReg(0x2E, 1); // Self Test

					tm.Reset();
					i++;
				};
			};

			break;

		case 7:

			if (dscAccel.ready)
			{
				i++;
			};

			break;

		case 8:

			if (tm.Check(10))
			{
				AccelReadReg(6, 11); // X_MSB 

				i++;
			};

			break;

		case 9:

			if (dscAccel.ready)
			{
				t = (rxAccel[0] << 8) | rxAccel[1];
				x = (rxAccel[2] << 24) | (rxAccel[3] << 16) | (rxAccel[4]<<8);
				y = (rxAccel[5] << 24) | (rxAccel[6] << 16) | (rxAccel[7]<<8);
				z = (rxAccel[8] << 24) | (rxAccel[9] << 16) | (rxAccel[10]<<8);

				//x /= 4096;
				//y /= 4096;
				//z /= 4096;

				fx += (x - fx) / 8;
				fy += (y - fy) / 8;
				fz += (z - fz) / 8;

				ay = -(fz / 65536); 
				ax = (fy / 65536); 
				az = -(fx / 65536);

				at = 250 + ((1852 - t) * 1000 + 452) / 905;
				//at = 250 + (1852 - t) * 1.1049723756906077348066298342541f;

				i--;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTemp()
{
	static byte i = 0;

	static DSCI2C dsc, dsc2;

//	static byte reg = 0;
	static u16 rbuf = 0;
	static byte buf[10];

	static TM32 tm;

	switch (i)
	{
		case 0:

			if (tm.Check(100))
			{
#ifndef WIN32
				if (!__debug) { HW::WDT->Update(); };
#endif

				buf[0] = 0x11;

				dsc.adr = 0x68;
				dsc.wdata = buf;
				dsc.wlen = 1;
				dsc.rdata = &rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				if (I2C_AddRequest(&dsc))
				{
					i++;
				};
			};

			break;

		case 1:

			if (dsc.ready)
			{
				if (dsc.ack && dsc.readedLen == dsc.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);
					
					t = (t * 10 + 128) / 256;

					if (t < (-600))
					{
						t += 2560;
					};

					tempClock = t;
				};
				//else
				//{
				//	tempClock = -2730;
				//};

				i++;
			};

			break;

		case 2:

			buf[0] = 0x0E;
			buf[1] = 0x20;
			buf[2] = 0xC8;

			dsc2.adr = 0x68;
			dsc2.wdata = buf;
			dsc2.wlen = 3;
			dsc2.rdata = 0;
			dsc2.rlen = 0;
			dsc2.wdata2 = 0;
			dsc2.wlen2 = 0;

			if (I2C_AddRequest(&dsc2))
			{
				i++;
			};

			break;

		case 3:

			if (dsc2.ready)
			{
				buf[0] = 0;

				dsc.adr = 0x49;
				dsc.wdata = buf;
				dsc.wlen = 1;
				dsc.rdata = &rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				if (I2C_AddRequest(&dsc))
				{
					i++;
				};
			};

			break;

		case 4:

			if (dsc.ready)
			{
				if (dsc.ack && dsc.readedLen == dsc.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);

					temp = (t * 10 + 64) / 128;
				};
				//else
				//{
				//	temp = -2730;
				//};

#ifdef CPU_SAME53	

				i = 0;
			};

			break;

#elif defined(CPU_XMC48)

				HW::SCU_GENERAL->DTSCON = SCU_GENERAL_DTSCON_START_Msk;
				
				i++;
			};

			break;

		case 5:

			if (HW::SCU_GENERAL->DTSSTAT & SCU_GENERAL_DTSSTAT_RDY_Msk)
			{
				cpu_temp = ((i32)(HW::SCU_GENERAL->DTSSTAT & SCU_GENERAL_DTSSTAT_RESULT_Msk) - 605) * 1000 / 205;

				i = 0;
			};

			break;

#elif defined(WIN32)

				i = 0;
			};

			break;
#endif
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateI2C()
{
#ifdef CPU_SAME53	

	I2C_Update();

#elif defined(CPU_XMC48)

	//if (!comdsp.Update())
	//{
	//	if (I2C_Update())
	//	{
	//		comdsp.InitHW();

	//		i2cResetCount++;
	//	};
	//};

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMoto()
{
	static Ptr<REQ> rq;
	static TM32 tm;

	static byte i = 0;

	switch (i)
	{
		case 0:

			rq = CreateMotoReq();

			if (rq.Valid())
			{
				qmoto.Add(rq);

				i++;
			};

			break;

		case 1:

			if (rq->ready)
			{
				rq.Free();

				i++;
			};

			break;

		case 2:

			if (tm.Check(101)) i = 0;

			break;
	};

	qmoto.Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTestFlashWrite()
{
	static Ptr<MB> ptr;
	static u32 count = 0;

	static CTM32 rtm;

	if (rtm.Check(MS2CTM(1)))
	{
		testDspReqCount++;

		count = 1000;
	};

//	if (count != 0)
	{
		ptr = CreateTestDspReq01();

		if (ptr.Valid())
		{
			count--;

			RspDsp01 *rsp = (RspDsp01*)(ptr->GetDataPtr());
			NandFlash_RequestWrite(ptr, rsp->CM.hdr.rw, true);

		};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateDSP_Com()
{
	static Ptr<REQ> rq;

	static byte i = 0;
//	static TM32 tm;

	switch (i)
	{
		case 0:

			if ((mv.fireVoltage == 0 && motoTargetRPS == 1500) || __WIN32__)
			{
				if (NandFlash_Status() != 0) UpdateTestFlashWrite();
			}
			else
			{
				rq = CreateDspReq01(1);

				if (rq.Valid())	qdsp.Add(rq), i++;
			};

			break;

		case 1:

			if (rq->ready)
			{
				if (rq->crcOK)
				{
					readyR01.Add(rq->rsp);
				};
				
				i = 0;

				rq.Free();
			};

			break;
	};

	qdsp.Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef DSPSPI

static void UpdateDSP_SPI()
{
	static Ptr<MB> mb;

	//static RspDsp01 rsp;

	static byte i = 0;
	static u16 len = 0;
	static bool crc = false;
	//static u32 ptime = 0;

	static S_SPIS::RBUF rb;

	switch (i)
	{
		case 0:

			if (!mb.Valid())
			{
				mb = NandFlash_AllocWB(sizeof(RspDsp01)+6);
			};

			if (mb.Valid())
			{
				mb->dataOffset = (mb->dataOffset + 3) & ~3;

				rb.data = mb->GetDataPtr();
				rb.maxLen = mb->GetDataMaxLen() & ~3;

				spidsp.Read(&rb, ~0, US2SPIS(50));

				HW::PIOC->BSET(19);
				PIO_SS->BCLR(PIN_SS);

				i++;
			};

			break;

		case 1:

			if (!spidsp.Update())
			{
				PIO_SS->BSET(PIN_SS);
				HW::PIOC->BCLR(19);

				bool c = false;

				RspDsp01 &rsp = *((RspDsp01*)rb.data);

				if (rsp.CM.hdr.rw == (dspReqWord|0x40))
				{
					if (rsp.CM.hdr.packType == 0)
					{
						len = rsp.CM.hdr.sl*2 + sizeof(rsp.CM.hdr) + 2;
						crc = (GetCRC16(&rsp.CM.hdr, sizeof(rsp.CM.hdr)) == rsp.CM.data[rsp.CM.hdr.sl]);
					}
					else
					{
						len = rsp.CM.hdr.packLen*2 + sizeof(rsp.CM.hdr) + 2;
						crc = (GetCRC16(&rsp.CM.hdr, sizeof(rsp.CM.hdr)) == rsp.CM.data[rsp.CM.hdr.packLen]);
					};

					c = /*(rb.len >= len) &&*/ crc;

					if (c)
					{
						mb->len = rb.len - 2;

						dspRcv40++;
						//dspRcvCount++;
					};
				}
				else if (rsp.IM.hdr.rw == (dspReqWord|0x50))
				{
					if (rb.len == (rsp.IM.hdr.dataLen*4 + sizeof(rsp.IM.hdr) + 2))
					{
						if (rsp.IM.data[rsp.IM.hdr.dataLen] == GetCRC16(&rsp.IM.hdr, sizeof(rsp.IM.hdr)))
						{
							mb->len = rb.len - 2;
							
							dspRcv50++;
							//dspRcvCount++;

							//dspMMSEC = rsp.IM.hdr.time;
							//shaftMMSEC = rsp.IM.hdr.hallTime;

							c = true;
						};
					};
				};

				if (c)
				{
					readyR01.Add(mb);

					mb.Free();
				}
				else if (rb.len != 0)
				{
					HW::PIOC->BSET(18);
					dspRcvErr++;
					HW::PIOC->BCLR(18);
				}
				else
				{
					HW::PIOC->BSET(17);
					dspNotRcv++;
					HW::PIOC->BCLR(17);
				};

				i = 0;

			};

			break;
	};
}
#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBootFlashMisc()
{
	qdsp.Update();
	qmoto.Update();
	UpdateEMAC();
	UpdateTraps();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

static const u32 dspFlashPagesBF592[] = {
#include "G26K.9V2.DSP.BF592.LDR.H"
};

static const u32 dspFlashPagesBF706[] = {
#include "G26K.9V2.DSP.BF706.LDR.H"
};

//u16 dspFlashLen = 0;
//u16 dspFlashCRC = 0;

//static const u32 dspBootFlashPages[] = {
//#include "G26K_2_BOOT_BF592.LDR.H"
//};

//u16 dspBootFlashLen = 0;
//u16 dspBootFlashCRC = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashDSP()
{
	Ptr<REQ> rq;

	//u32 flashLen = sizeof(rcvFlashPages);
	//u16 flashCRC = GetCRC16(rcvFlashPages, flashLen);

	BootRspV1::SF0	rspF0;

	u16 tryCount = 4;
	
	bool hs = false;
//	byte N = RCV_MAX_NUM_STATIONS;

	CTM32 tm;

	while (tryCount > 0)
	{
		rq = CreateDspBootReq00(DSP_BOOT_NET_ADR, &rspF0, 2);

		if (rq.Valid())
		{
			qdsp.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

			if (rq->crcOK)
			{
				BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->wb.data);
				BootRspV1::SF0 &rsp = rspF0;

				if (rsp.adr == DSP_BOOT_NET_ADR && rsp.rw == req.rw)
				{
					hs = true;
					break;
				};

				if (tryCount > 2) tryCount = 2;
			};

			rq.Free();
		};
		
		tryCount--;
	};

	const u32* dspFlashPages = 0;
	u32 flashLen = 0;
	u16 flashCRC = 0;

	if (hs)
	{
		if (rspF0.guid == DSP_BOOT_SGUID_BF592)
		{
			dspFlashPages = dspFlashPagesBF592;
			flashLen = sizeof(dspFlashPagesBF592);
		}
		else if (rspF0.guid == DSP_BOOT_SGUID_BF706)
		{
			dspFlashPages = dspFlashPagesBF706;
			flashLen = sizeof(dspFlashPagesBF706);
		};
	};

	if (dspFlashPages != 0 && flashLen != 0)
	{
		flashCRC = GetCRC16(dspFlashPages, flashLen);

		rq = CreateDspBootReq01(DSP_BOOT_NET_ADR, flashLen, 2);

		qdsp.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

		BootRspV1::SF1 &rsp = *((BootRspV1::SF1*)rq->rb.data);

		if (rsp.flashCRC != flashCRC || rsp.flashLen != flashLen)
		{
			u16 count = flashLen+2;
			u16 adr = 0;
			byte *p = (byte*)dspFlashPages;

			u16 max = rspF0.pageLen;

			u16 len;

			while (count > 0)
			{
				if (count > max)
				{
					len = max;

					rq = CreateDspBootReq02(DSP_BOOT_NET_ADR, adr, len, p, 0, 0, max, ~0);
				}
				else
				{
					len = count;

					if (len > 2)
					{
						rq = CreateDspBootReq02(DSP_BOOT_NET_ADR, adr, len-2, p, sizeof(flashCRC), &flashCRC, max, ~0);
					}
					else
					{
						rq = CreateDspBootReq02(DSP_BOOT_NET_ADR, adr, sizeof(flashCRC), &flashCRC, 0, 0, max, ~0);
					};
				};

				if (!rq.Valid()) __breakpoint(0);

				qdsp.Add(rq); while(!rq->ready) UpdateBootFlashMisc();



				count -= len;
				p += len;
				adr += len;
			};
		};

		tm.Reset();	while (!tm.Check(MS2CTM(1)));

		//rq = CreateRcvBootReq03(i, 2);

		//qRcv.Add(rq); while(!rq->ready) qRcv.Update();

		//rq.Free();
	};

	tm.Reset(); while (!tm.Check(MS2CTM(1)));

	rq = CreateDspBootReq03(DSP_BOOT_NET_ADR, 2);

	qdsp.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

	rq.Free();

	tm.Reset();	while (!tm.Check(MS2CTM(10)));
	
	//comRcv.Disconnect();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const u32 motoFlashPages[] = {
#include "G26K.2.V2.MOTO.BIN.H"
};



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashMoto()
{
	Ptr<REQ> rq;

	BootRspV1::SF0	rspF0;

	u16 tryCount = 4;

	bool hs = false;

	CTM32 tm;

	while (tryCount > 0)
	{
		rq = CreateBootMotoReq00(MOTO_BOOT_NET_ADR, &rspF0, 2);

		if (rq.Valid())
		{
			qmoto.Add(rq); while(!rq->ready) { qmoto.Update(); HW::WDT->Update(); };

			if (rq->crcOK)
			{
				BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->wb.data);
				BootRspV1::SF0 &rsp = rspF0;

				if (rsp.adr == MOTO_BOOT_NET_ADR && rsp.rw == req.rw)
				{
					hs = true;
					break;
				};

				if (tryCount > 2) tryCount = 2;
			};

			rq.Free();
		};

		tryCount--;
	};

	const u32* flashPages = 0;
	u32 flashLen = 0;
	u16 flashCRC = 0;

	if (hs)
	{
		if (rspF0.guid == MOTO_BOOT_SGUID)
		{
			flashPages = motoFlashPages;
			flashLen = sizeof(motoFlashPages);
		};
	};

	if (flashPages != 0 && flashLen != 0)
	{
		flashCRC = GetCRC16(flashPages, flashLen);

		rq = CreateBootMotoReq01(MOTO_BOOT_NET_ADR, flashLen, 2);

		qmoto.Add(rq); while(!rq->ready) { qmoto.Update(); HW::WDT->Update(); };

		if (rq->crcOK)
		{
			BootRspV1::SF1 &rsp = *((BootRspV1::SF1*)rq->rb.data);

			if (rsp.flashCRC != flashCRC || rsp.flashLen != flashLen)
			{
				u16 count = flashLen;
				u16 adr = 0;
				byte *p = (byte*)flashPages;

				u16 max = rspF0.pageLen;

				u16 len;

				while (count > 0)
				{
					len = (count > max) ? max : count;

					rq = CreateBootMotoReq02(MOTO_BOOT_NET_ADR, adr, len, p, max, 2);

					qmoto.Add(rq); while(!rq->ready) { qmoto.Update(); HW::WDT->Update(); };

					count -= len;
					p += len;
					adr += len;
				};
			};

			tm.Reset();	while (!tm.Check(MS2CTM(100)));
		};

		tm.Reset(); while (!tm.Check(MS2CTM(100)));

		rq = CreateBootMotoReq03(MOTO_BOOT_NET_ADR, 2);

		qmoto.Add(rq); while(!rq->ready) { qmoto.Update(); HW::WDT->Update();	};

		rq.Free();

		tm.Reset();	while (!tm.Check(MS2CTM(10)));
	};
}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitMainVars()
{
	mv.numDevice		= 11111;
	mv.numMemDevice		= 11111;

	mv.sens1.gain			= 0; 
	mv.sens1.st				= NS2DSP(400); 
	mv.sens1.sl				= 500; 
	mv.sens1.sd 			= US2DSP(50); 
	mv.sens1.deadTime		= US2DSP(50); 
	mv.sens1.descriminant	= 400; 
	mv.sens1.freq			= 500;
	mv.sens1.filtrType		= 0;
	mv.sens1.packType		= 0;
	mv.sens1.fi_Type		= 0;
	mv.sens1.fragLen		= 0;

	mv.sens2.gain			= 0; 
	mv.sens2.st				= NS2DSP(400); 
	mv.sens2.sl				= 500; 
	mv.sens2.sd 			= US2DSP(50); 
	mv.sens2.deadTime		= US2DSP(50); 
	mv.sens2.descriminant	= 400; 
	mv.sens2.freq			= 500; 
	mv.sens2.filtrType		= 0;
	mv.sens2.packType		= 0;
	mv.sens2.fi_Type		= 0;
	mv.sens2.fragLen		= 0;

	mv.refSens.gain			= 0; 
	mv.refSens.st			= NS2DSP(400); 
	mv.refSens.sl			= 500; 
	mv.refSens.sd 			= US2DSP(50); 
	mv.refSens.deadTime		= US2DSP(50); 
	mv.refSens.descriminant	= 400; 
	mv.refSens.freq			= 500; 
	mv.refSens.filtrType	= 0;
	mv.refSens.packType		= 0;
	mv.refSens.fi_Type		= 0;
	mv.refSens.fragLen		= 0;

	mv.cmSPR			= 36;
	mv.imSPR			= 180;
	mv.fireVoltage		= 500;
	mv.motoLimCur		= 2000;
	mv.motoMaxCur		= 3000;
	mv.sensMask			= 1;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "Init Main Vars Vars ... OK\n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadVars()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "Load Vars ... ");

	static DSCI2C dsc;
	static DSCSPI spi;
	static u16 romAdr = 0;
	
	byte buf[sizeof(mv)*2+4];

	MainVars mv1, mv2;

	bool c1 = false, c2 = false;

	//spi.adr = ((u32)ReverseWord(FRAM_SPI_MAINVARS_ADR)<<8)|0x9F;
	//spi.alen = 1;
	//spi.csnum = 1;
	//spi.wdata = 0;
	//spi.wlen = 0;
	//spi.rdata = buf;
	//spi.rlen = 9;

	//if (SPI_AddRequest(&spi))
	//{
	//	while (!spi.ready);
	//};

	bool loadVarsOk = false;

	spi.adr = (ReverseDword(FRAM_SPI_MAINVARS_ADR) & ~0xFF) | 3;
	spi.alen = 4;
	spi.csnum = 1;
	spi.wdata = 0;
	spi.wlen = 0;
	spi.rdata = buf;
	spi.rlen = sizeof(buf);

	if (SPI_AddRequest(&spi))
	{
		while (!spi.ready) { SPI_Update(); };
	};

	PointerCRC p(buf);

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv1, sizeof(mv1));
		p.ReadW();

		if (p.CRC.w == 0) { c1 = true; break; };
	};

	romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

	dsc.wdata = &romAdr;
	dsc.wlen = sizeof(romAdr);
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;
	dsc.rdata = buf;
	dsc.rlen = sizeof(buf);
	dsc.adr = 0x50;


	if (I2C_AddRequest(&dsc))
	{
		while (!dsc.ready) { I2C_Update(); };
	};

//	bool c = false;

	p.b = buf;

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv2, sizeof(mv2));
		p.ReadW();

		if (p.CRC.w == 0) { c2 = true; break; };
	};

	SEGGER_RTT_WriteString(0, "FRAM SPI - "); SEGGER_RTT_WriteString(0, (c1) ? (RTT_CTRL_TEXT_BRIGHT_GREEN "OK") : (RTT_CTRL_TEXT_BRIGHT_RED "ERROR"));

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE " ... FRAM I2C - "); SEGGER_RTT_WriteString(0, (c2) ? (RTT_CTRL_TEXT_BRIGHT_GREEN "OK\n") : (RTT_CTRL_TEXT_BRIGHT_RED "ERROR\n"));

	if (c1 && c2)
	{
		if (mv1.timeStamp > mv2.timeStamp)
		{
			c2 = false;
		}
		else if (mv1.timeStamp < mv2.timeStamp)
		{
			c1 = false;
		};
	};

	if (c1)	{ mv = mv1; } else if (c2) { mv = mv2; };

	loadVarsOk = c1 || c2;

	if (!c1 || !c2)
	{
		if (!loadVarsOk) InitMainVars();

		svCount = 2;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SaveVars()
{
	static DSCI2C dsc;
	static DSCSPI spi,spi2;
	static u16 romAdr = 0;
	static byte buf[sizeof(mv) * 2 + 8];

	static byte i = 0;
	static TM32 tm;

	PointerCRC p(buf);

	switch (i)
	{
		case 0:

			if (svCount > 0)
			{
				svCount--;
				i++;
			};

			break;

		case 1:

			mv.timeStamp = GetMilliseconds();

			for (byte j = 0; j < 2; j++)
			{
				p.CRC.w = 0xFFFF;
				p.WriteArrayB(&mv, sizeof(mv));
				p.WriteW(p.CRC.w);
			};

			spi.adr = (ReverseDword(FRAM_SPI_MAINVARS_ADR) & ~0xFF) | 2;
			spi.alen = 4;
			spi.csnum = 1;
			spi.wdata = buf;
			spi.wlen = p.b-buf;
			spi.rdata = 0;
			spi.rlen = 0;

			romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

			dsc.wdata = &romAdr;
			dsc.wlen = sizeof(romAdr);
			dsc.wdata2 = buf;
			dsc.wlen2 = p.b-buf;
			dsc.rdata = 0;
			dsc.rlen = 0;
			dsc.adr = 0x50;

			spi2.adr = 6;
			spi2.alen = 1;
			spi2.csnum = 1;
			spi2.wdata = 0;
			spi2.wlen = 0;
			spi2.rdata = 0;
			spi2.rlen = 0;

			tm.Reset();

			SPI_AddRequest(&spi2);

			i++;

			break;

		case 2:

			if (spi2.ready || tm.Check(200))
			{
				SPI_AddRequest(&spi);

				i++;
			};

			break;

		case 3:

			if (spi.ready || tm.Check(200))
			{
				I2C_AddRequest(&dsc);
				
				i++;
			};

			break;

		case 4:

			if (dsc.ready || tm.Check(100))
			{
				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSPI()
{
	SPI_Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static TaskList taskList;

static void InitTaskList()
{
	static Task tsk[] =
	{
		Task(MainMode,				US2CTM(100)	),
		Task(UpdateDSP_Com,			US2CTM(1)	),
		Task(UpdateMoto,			US2CTM(20)	),
		Task(UpdateMan,				US2CTM(100)	),
		Task(UpdateI2C,				US2CTM(20)	),
		Task(UpdateSPI,				US2CTM(20)	),
		Task(UpdateTemp,			MS2CTM(1)	),
		Task(NandFlash_Update,		MS2CTM(1)	),
		Task(UpdateAccel,			MS2CTM(1)	),
		Task(UpdateEMAC,			MS2CTM(1)	),
		Task(SaveVars,				MS2CTM(1)	),
		Task(UpdateShaft,			MS2CTM(1)	)
	};

	for (u16 i = 0; i < ArraySize(tsk); i++) taskList.Add(tsk+i);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateParams()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( MainMode()				);
		CALL( UpdateMoto()				);
		CALL( UpdateTemp()				);
		CALL( UpdateMan(); 				);
		CALL( NandFlash_Update();		);
		CALL( UpdateHardware();			);
		CALL( UpdateAccel();			);
		CALL( UpdateI2C();				);
		CALL( SaveVars();				);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMisc()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateEMAC();		);
		CALL( UpdateDSP_Com();	);
		CALL( SPI_Update();		);
		CALL( UpdateParams();	);
		CALL( I2C_Update();		); 
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update()
{
	NAND_Idle();	
	//UpdateEMAC();

	if (EmacIsConnected())
	{
		UpdateEMAC();
		UpdateTraps();

#ifndef WIN32
		if (!__debug) { HW::WDT->Update(); };
#endif
	};
	
	if (!(IsComputerFind() && EmacIsConnected()))
	{
		taskList.Update();

		#ifdef DSPSPI
			UpdateDSP_SPI();
		#endif
	}
	else
	{
		I2C_Update();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void Com_test()
//{
//	TM32 tm;
//
//	ComPort::ReadBuffer rb;
//	ComPort::WriteBuffer wb;
//
//	static ReqDsp01 req;
//	static RspDsp01 rsp;
//
//	req.rw				= dspReqWord|1;
//	req.len				= sizeof(req);
//	req.version			= req.VERSION;
//	req.mode 			= mode;
//	req.ax				= ax;
//	req.ay				= ay;
//	req.az				= az;
//	req.at				= at;
//	req.gain 			= mv.gain;
//	req.st	 			= mv.sampleTime;
//	req.sl 				= mv.sampleLen;
//	req.sd 				= mv.sampleDelay;
//	req.thr				= mv.descriminant;
//	req.descr			= mv.deadTime;
//	req.freq			= mv.freq;
//	req.refgain 		= mv.gainRef;
//	req.refst			= mv.sampleTimeRef;
//	req.refsl 			= mv.sampleLenRef;
//	req.refsd 			= mv.sampleDelayRef;
//	req.refthr			= mv.descriminantRef;
//	req.refdescr		= mv.deadTimeRef;
//	req.refFreq			= mv.refFreq;
//	req.vavesPerRoundCM = mv.cmSPR;
//	req.vavesPerRoundIM = mv.imSPR;
//	req.filtrType		= mv.filtrType;
//	req.packType		= mv.packType;
//	req.fireVoltage		= mv.fireVoltage;
//
//	req.crc	= GetCRC16(&req, sizeof(ReqDsp01)-2);
//
//	//byte buf[20] = {0};
//
//	rb.data = &rsp;
//	rb.maxLen = sizeof(rsp.v01)+sizeof(rsp.rw);
//	wb.data = &req;
//	wb.len = sizeof(req);
//	//buf[0] = 0x55;
//	//buf[19] = 0xAA;
//
//	////commoto.Read(&rb, MS2COM(50), US2COM(100));
//
//	static byte i = 0;
//
//	while (1)
//	{
//		switch (i)
//		{
//			case 0:
//
//				if (tm.Check(100)) 
//				{
//					comdsp.Write(&wb);
//					i++;	
//				};
//				
//				break;
//
//			case 1:
//
//				if (!comdsp.Update())
//				{
//					comdsp.Read(&rb, MS2COM(50), US2COM(100));
//
//					i++;
//				};
//
//				break;
//
//			case 2:
//
//				if (!comdsp.Update())
//				{
//					i = 0;
//				};
//		};
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __TEST__

static void Test_Ptr_MB()
{
	const u32 buflen = 1UL<<4;
	const u32 bufmask = buflen-1;

	Ptr<MB> buf[16];

	Ptr<MB> mb = AllocSmallBuffer();

	if (!mb.Valid()) __breakpoint(0);

	u32 count = mb.Count();

	u32 freeCount = mb->FreeCount();

	for (u32 i = 0; i < buflen; i++)
	{
		if (buf[i].Valid()) __breakpoint(0);
	};

	Ptr<MB> ptr;// = AllocSmallBuffer();

	for (u32 i = 0, n = 0; i < 100000; i++)
	{
		n = n * 7727 + (n>>16) + 1;

		if (((n>>8)&0xF) == 0)
		{
			ptr = AllocSmallBuffer();
		}
		else
		{
			buf[n&bufmask] = ptr;
		};
	};

	for (u32 i = 0; i < buflen; i++)
	{
		buf[i].Free();

		if (buf[i].Valid()) __breakpoint(0);
	};

	ptr.Free();

	if (ptr.Valid()) __breakpoint(0);

	u32 freeCount2 = mb->FreeCount();

	if (freeCount2 != freeCount) __breakpoint(0);

	u32 count2 = mb.Count();

	if (count2 != count) __breakpoint(0);

	mb.Free();

	if (mb.Valid()) __breakpoint(0);
}

#endif // __TEST__

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __TEST__

static void Test_ListPtr()
{
	static ListPtr<MB> list;

	for (u32 i = 0; i < 100; i++)
	{
		Ptr<MB> mb = AllocSmallBuffer();

		list.Add(mb);
	};

	for (u32 i = 0; i < 100; i++)
	{
		Ptr<MB> mb = list.Get();
	};

	if(!list.Empty()) __breakpoint(0);
}

#endif // __TEST__

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __TEST__

static void Test_ListRef()
{
	static ListRef<MB> list;

	for (u32 i = 0; i < 100; i++)
	{
		Ptr<MB> mb = AllocSmallBuffer();

		list.Add(mb);
	};

	for (u32 i = 0; i < 100; i++)
	{
		Ptr<MB> mb = list.Get();
	};

	if(!list.Empty()) __breakpoint(0);
}

#endif // __TEST__

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(__TEST__) && defined(WIN32) 

static ListPtr<MB> mt_listptr;

DWORD WINAPI Thread_Test_ListPtr(LPVOID lpParam) 
{
	Ptr<MB> mb;

	while(1)
	{
		mb = mt_listptr.Get();
	};
}

static void Test_MT_ListPtr()
{
	HANDLE ht = CreateThread(0, 0, Thread_Test_ListPtr, 0, 0, 0);

	DEBUG_ASSERT(ht != INVALID_HANDLE_VALUE);

	Ptr<MB> mb;

	//mb = AllocSmallBuffer(); mt_listptr.Add(mb);
	//mb = AllocSmallBuffer(); mt_listptr.Add(mb);
	//mb = AllocSmallBuffer(); mt_listptr.Add(mb);
	//mb = AllocSmallBuffer(); mt_listptr.Add(mb);

	u32 t = GetMilliseconds();

	while ((GetMilliseconds() - t) < 2000) 
	{
		mb = AllocSmallBuffer();

		mt_listptr.Add(mb);
	};

	t = GetMilliseconds();

	while (!mt_listptr.Empty() && (GetMilliseconds() - t) < 1000) 
	{
		Sleep(1);
	};

	if(!mt_listptr.Empty()) __breakpoint(0);

	DEBUG_ASSERT(TerminateThread(ht, 0) != 0);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(__TEST__) && defined(WIN32) 

static ListRef<MB> mt_listref;

DWORD WINAPI Thread_Test_ListRef(LPVOID lpParam) 
{
	Ptr<MB> mb;

	while(1)
	{
		mb = mt_listref.Get();
	};
}

static void Test_MT_ListRef()
{
	HANDLE ht = CreateThread(0, 0, Thread_Test_ListRef, 0, 0, 0);

	DEBUG_ASSERT(ht != INVALID_HANDLE_VALUE);

	Ptr<MB> mb;

	//mb = AllocSmallBuffer(); mt_listref.Add(mb);
	//mb = AllocSmallBuffer(); mt_listref.Add(mb);
	//mb = AllocSmallBuffer(); mt_listref.Add(mb);
	//mb = AllocSmallBuffer(); mt_listref.Add(mb);

	u32 t = GetMilliseconds();

	while ((GetMilliseconds() - t) < 2000) 
	{
		mb = AllocSmallBuffer();

		mt_listref.Add(mb);
	};

	t = GetMilliseconds();

	while (!mt_listref.Empty() && (GetMilliseconds() - t) < 1000) 
	{
		Sleep(1);
	};

	if(!mt_listref.Empty()) __breakpoint(0);

	DEBUG_ASSERT(TerminateThread(ht, 0) != 0);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_WHITE "main() start ...\n");

	#ifdef __TEST__

		Test_Ptr_MB();
		Test_ListPtr();
		Test_ListRef();
		//Test_MT_ListPtr();
		//Test_MT_ListRef();

	#endif

//	static bool c = true;

//	static byte buf[16384];

	//volatile byte * const FLD = (byte*)0x60000000;	
	
	//static ComPort commem;

	DSCSPI dsc, dsc2;

	TM32 tm;

	//__breakpoint(0);

#ifndef WIN32

	DisableDSP();

#endif

	InitHardware();

	LoadVars();

	InitEMAC();

	NandFlash_Init();
 
	Update_RPS_SPR();

#ifndef WIN32

	commoto.Connect(ComPort::ASYNC, MOTO_COM_BAUDRATE, MOTO_COM_PARITY, MOTO_COM_STOPBITS);
	comdsp.Connect(ComPort::ASYNC, DSP_COM_BAUDRATE, DSP_COM_PARITY, DSP_COM_STOPBITS);

	#ifdef DSPSPI
		spidsp.Connect(true, true);
	#endif

	EnableDSP();

	//__breakpoint(0);

	FlashMoto();

	FlashDSP();

#endif

	InitTaskList();

	u32 fc = 0;

	while (1)
	{
//		PIO_SS->BTGL(PIN_SS);

		Pin_MainLoop_Set();

		Update();

		Pin_MainLoop_Clr();

		fc++;

		if (tm.Check(1000))
		{ 
			fps = fc; fc = 0; 

			//static MTB mtb;
			//static u16 buf[20];

			//mtb.baud = 1;
			//mtb.data1 = buf;
			//mtb.len1 = ArraySize(buf);
			//mtb.data2 = 0;
			//mtb.len2 = 0;

			//SendManData(&mtb);

#ifdef WIN32

			extern u32 txThreadCount;

			Printf(0, 0, 0xFC, "FPS=%9i", fps);
			Printf(0, 1, 0xF0, "%lu", testDspReqCount);
			Printf(0, 2, 0xF0, "%lu", txThreadCount);
#endif
		};

#ifdef WIN32

		UpdateDisplay();

		static TM32 tm2;

		byte key = 0;

		if (tm2.Check(50))
		{
			if (_kbhit())
			{
				key = _getch();

				if (key == 27) break;
			};

			if (key == 'w')
			{
				NandFlash_WriteEnable();
			}
			else if (key == 'e')
			{
				NandFlash_WriteDisable();
			}
			else if (key == 'p')
			{
				NandFlash_FullErase();
			};
		};

		Sleep(0);

#endif

	}; // while (1)

#ifdef WIN32

	NAND_FlushBlockBuffers();

	I2C_Destroy();
	SPI_Destroy();

	//_fcloseall();

#endif

}
