#ifndef HARDWARE_H__15_05_2009__14_35
#define HARDWARE_H__15_05_2009__14_35
  
#include "types.h"
#include "hw_conf.h"

#ifdef WIN32
#include <windows.h>
#endif


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define SPORT_BUF_LEN	(2048+64)
#define SENS_NUM	3
#define NS2DSP(v) (((v)+10)/20)
#define US2DSP(v) ((((v)*1000)+10)/20)

struct DSCSPORT
{
	DSCSPORT	*next;
	u32			mmsec;
	u32			rotCount;
	//u32			rotMMSEC;
	u32			shaftTime;
	//u32			shaftPrev;
	u32			fireIndex;
	u16			sportDelay;
	u16			motoCount;
	u16			shaftCount;
	u16			sensType;
	u16			gain;
	u16			len;
	u16			chMask;
	u16			sport_tfsdiv;
	u16			sampleTime;
	u16			sampleDelay;
	u16			ax;
	u16			ay;
	u16			az;
	u16			at;
	u16			busy;
	u16			data[SPORT_BUF_LEN];
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct SENS
{
	u16 	gain; 
	u16 	st;	 
	u16 	sl; 
	u16 	sd; 
	u16		thr;
	u16		descr;
	u16		freq;
	u16 	filtr;
	u16 	pack;
	u16 	fi_Type;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma pack(1)

struct RspCM	// 0xAD40
{
	u16 	rw;
	u32 	mmsecTime; 
	u32		shaftTime; 
	u16		motoCount; 
	u16		headCount;
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16		sensType; 
	u16		angle;
	u16		maxAmp;
	u16		fi_amp;
	u16		fi_time;
	u16 	gain; 
	u16 	st;	 
	u16 	sl; 
	u16 	sd; 
	u16		packType;
	u16		packLen;
};

#pragma pack()

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma pack(1)

struct RspIM	// 0xAD50
{
	u16 	rw;
	u32 	mmsecTime; 
	u32		shaftTime; 
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16		sensType; 
	u16 	gain; 
	u16		refAmp;
	u16		refTime;
	u16		len;
	u16		data[16];
};

#pragma pack()

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define RSPWAVE_BUF_LEN	(1024+(sizeof(RspCM)+1)/2+16)

struct RSPWAVE
{
	RSPWAVE		*next;
	u32			fireIndex;
	u16			mode;
	u16			shaftCount;
	u16			dataLen;

	u16			data[RSPWAVE_BUF_LEN];

	inline u16 MaxLen() { return ArraySize(data); }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma pack(1)

struct ReqDsp01_old	// чтение вектора
{
	u16 	rw;
	u16 	mode; 
	u32 	mmsecTime; 
	u32		hallTime; 
	u16		motoCount; 
	u16		headCount;
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16		sensType; 
	u16		angle;

	SENS	mainSens;
	SENS	refSens;

	u16		vavesPerRoundCM;
	u16		vavesPerRoundIM;

	u16		filtrType;
	u16		packType;

	u16 	crc;  
};

//#pragma pack()

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma pack(1)

struct ReqDsp01	// чтение вектора
{
	enum { VERSION = 1 };

	u16 	rw;
	u16		len;				// Длина структуры
	u16		version;			// Версия структуры

	u16 	mode;				// 0 - Режим цементомера; !=0 - режим имиджера
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;

	SENS	sens[SENS_NUM];		// измерительный датчик 1
	//SENS	sens2;				// измерительный датчик 2
	//SENS	refSens;			// опорный датчик

	u16		wavesPerRoundCM;	// Количество волновых картин на оборот головки в режиме цементомера
	u16		wavesPerRoundIM;	// Количество точек на оборот головки в режиме имиджера

	u16		fireVoltage;		// Напряжение излучателя (0.1 В)
	u16		sensMask;

	u16 	crc;  
};

//#pragma pack()

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma pack(1)

struct RspDsp01	// чтение вектора
{
	enum { VERSION = 1 };

	u16		rw; 
	u16		len;				// Длина структуры
	u16		version;			// Версия структуры

	u16		fireVoltage;		// Напряжение излучателя (0.1 В)
	u16 	crc;  
};

//pragma pack()

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct  ReqDsp05 { u16 rw; u16 crc; };										// запрос контрольной суммы и длины программы во флэш-памяти
struct  ReqDsp06 { u16 rw; u16 stAdr; u16 len; byte data[256]; u16 crc; }; // запись страницы во флэш
struct  ReqDsp07 { u16 rw; word crc; };										// перезагрузить блэкфин
struct  RspDsp05 { u16 rw; u16 flashLen; u16 flashCRC; u16 crc; };					// запрос контрольной суммы и длины программы во флэш-памяти
struct  RspDsp06 { u16 rw; u16 res; u16 crc; };									// запись страницы во флэш

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();
extern void InitIVG(u32 IVG, u32 PID, void (*EVT)());
extern void SetDspVars(const ReqDsp01 *v, bool forced = false);


//extern bool defPPI_Ready;

//extern void SyncReadSPORT(void *dst1, void *dst2, u16 len1, u16 len2, u16 clkdiv, bool *ready0, bool *ready1);
//extern void ReadPPI(void *dst);
extern DSCSPORT* GetDscSPORT();
extern void FreeDscSPORT(DSCSPORT* dsc);
extern DSCSPORT* AllocDscSPORT();

//extern void SetGain(byte v);

extern void SetFireVoltage(u16 v);
extern u16	GetFireVoltage();


#endif // HARDWARE_H__15_05_2009__14_35
