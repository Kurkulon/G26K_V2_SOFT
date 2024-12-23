#include "hardware.h"
#include "time.h"
#include "ComPort.h"
#include "crc16.h"
#include <SEGGER_RTT.h>

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//const char build_date[] __attribute__((used)) = "\n" __DATE__ "\n" __TIME__ "\n";


u32 fps = 0;

//static byte sec = 0;

static ComPort com(USART0_USIC_NUM, ~0, PIN_UTX0, PIN_URX0, PIN_RTS0);

static u16 manReqWord = 0x0100;
static u16 manReqMask = 0xFF00;

static u16 numDevice = 1;
static u16 verDevice = 0x103;

//static u32 manCounter = 0;

//static u16 temp = 0;

//inline u16 ReverseWord(u16 v) { __asm	{ rev16 v, v };	return v; }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqMoto
{
	u16 	rw;
	u16 	enableMotor; 
	u32		tRPM;		// время 1/6 оборота двигателя в мкс
	u16		limCurrent; // Ограничение тока двигателя (мА)
	u16		maxCurrent; // Аварийный ток двигателя (мА)
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspMoto
{
	u16 	rw;
	u16 	mororStatus; 
	u16		current;
	u16		currentLow;
	u16		rpm;
	u16		motoCounter;
	u16		auxVoltage;
	u16		motoVoltage;
	u16		motoDuty;
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_00(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 rsp[3];

	if (wb == 0 || len != 1) return false;

	rsp[0] = manReqWord;
	rsp[1] = numDevice;
	rsp[2] = verDevice;

	wb->data = rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_10(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static RspMoto rsp;

	if (wb == 0 || len < 2) return false;

	ReqMoto *req = (ReqMoto*)data;

	SetTargetRPM(req->tRPM);
	SetLimCurrent(req->limCurrent);
	SetMaxCurrent(req->maxCurrent);

	rsp.rw = manReqWord|1;	// 	1. ответное слово
	rsp.mororStatus	= GetMotorState();
	rsp.current		= GetAvrCurrent();
	rsp.currentLow	= GetAvrCurrentLow();
	rsp.rpm			= GetRPM();
	rsp.motoCounter = GetmotoCounter();
	rsp.auxVoltage	= GetAvrAuxVoltage();
	rsp.motoVoltage = GetAvrMotoVoltage();
	rsp.motoDuty	= GetDutyPWM();
	rsp.crc			= GetCRC16(&rsp, sizeof(rsp)-2);

	wb->data = &rsp;			 
	wb->len = sizeof(rsp);	 

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 rsp[9];

	if (wb == 0 || len != 1) return false;


	wb->data = rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan(ComPort::WriteBuffer *wb, ComPort::ReadBuffer *rb)
{
	u16 *p = (u16*)rb->data;
	bool r = false;

	u16 t = p[0];

	if ((t & manReqMask) != manReqWord || rb->len < 2)
	{
		return false;
	};

	u16 len = (rb->len)>>1;

	t &= 0xFF;

	switch (t)
	{
		case 0: 	r = RequestMan_00(p, len, wb); break;
		case 1: 	r = RequestMan_10(p, len, wb); break;
		case 2: 	r = RequestMan_20(p, len, wb); break;

		//default: 	r = RequestMan_10(p, len, wb); break;
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMan()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static byte buf[1024];

	switch(i)
	{
		case 0:

			rb.data = buf;
			rb.maxLen = sizeof(buf);
			com.Read(&rb, ~0, US2COM(250));
			i++;

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0 && GetCRC16(rb.data, rb.len) == 0)
				{
					if (RequestMan(&wb, &rb))
					{
						com.Write(&wb, US2COM(10));
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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMisc()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateHardware();	);
		CALL( UpdateMan();		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_WHITE "main() start ...\n");

//	__breakpoint(0);

	InitHardware();

	com.Connect(ComPort::ASYNC, 1562500, 0, 1);

	while (1)
	{
		Pin_MainLoop_Set();

		UpdateMisc();

		Pin_MainLoop_Clr();
	}; // while (1)
}
