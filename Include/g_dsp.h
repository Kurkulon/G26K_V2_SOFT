#ifndef G_DSP_H__01_12_2023__12_16
#define G_DSP_H__01_12_2023__12_16

//#pragma once

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "types.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __CC_ARM

#else
	#pragma pack(1)
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define DSP_COM_BAUDRATE		12500000
#define DSP_COM_PARITY			2
#define DSP_COM_STOPBITS		2

#define DSP_BOOT_SGUID_BF592	0X4AC087A349414b96
#define DSP_BOOT_SGUID_BF706	0X950F17B4DB95ABD8
#define DSP_BOOT_REQ_WORD		((~(DSP_MAN_REQ_WORD)) & DSP_MAN_REQ_MASK)
#define DSP_BOOT_REQ_MASK		DSP_MAN_REQ_MASK
#define DSP_BOOT_COM_BAUDRATE	DSP_COM_BAUDRATE
#define DSP_BOOT_COM_PARITY		DSP_COM_PARITY
#define DSP_BOOT_COM_STOPBITS	DSP_COM_STOPBITS
#define DSP_BOOT_NET_ADR		1

#define DSP_MAN_REQ_WORD 		0xAA00
#define DSP_MAN_REQ_MASK 		0xFF00

//#define RCV_FltResist(v)	(((v) * 941 + 2048) / 4096)
//#define RCV_NetResist(v)	(((v) * 941 + 128) / 256)
//#define RCV_NetAdr(v)		(1 + (v)/1024)

//#define RCV_TEST_WAVEPACK 16

//#define RCV_WAVEPACK

//#define RCV_SAMPLE_LEN 1024

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define SENS_NUM	3

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct SENS
{
	u16 gain;
	u16 st;	//sampleTime;
	u16 sl;	//sampleLen;
	u16 sd;	//sampleDelay;
	u16 deadTime;
	u16 descriminant;
	u16 freq;
	u16 filtrType;
	u16 packType;
	u16 fi_Type;
	u16 fragLen;
};
	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqDsp01	// ������ �������
{
	enum { VERSION = 1 };

	u16 	rw;
	u16		len;				// ����� ���������
	u16		version;			// ������ ���������

	//u16		com_spi;			// 0 - ����� �� UART, 1 - ����� �� SPI

	u16 	mode; 
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	
	SENS	sens[SENS_NUM];		//SENS	sens1;
								//SENS	sens2;
								//SENS	refSens;
	u16		wavesPerRoundCM;
	u16		wavesPerRoundIM;
	u16		fireVoltage;		// ���������� ���������� (0.1�)
	u16		sensMask;

	u16 	crc;  
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspHdrCM	// 0xAD40
{
	u16 	rw;
	u32 	time;		//mmsecTime; 
	u32		hallTime;	//shaftTime; 
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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspHdrIM	// 0xAD50
{
	u16 	rw;
	u32 	time;		//mmsecTime; 
	u32		hallTime;	//shaftTime; 
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16		sensType; 
	u16 	gain; 
	u16		refAmp;
	u16		refTime;
	u16		dataLen;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed union RspDsp01	// ������ �������
{
	__packed struct { RspHdrCM hdr; u16 data[1024]; } CM;
	__packed struct { RspHdrIM hdr; u16 data[1024]; } IM;
	__packed struct { u16 rw; u16 len; u16 version; u16 fireVoltage; u16 motoVoltage; u16 crc; } v01;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  ReqDsp05	// ������ ����������� ����� � ����� ��������� �� ����-������
//{ 
//	u16		rw; 
//	u16 	crc; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  RspDsp05	// ������ ����������� ����� � ����� ��������� �� ����-������
//{ 
//	u16		rw; 
//	u16		flashLen; 
//	u32		startAdr; 
//	u16		flashCRC; 
//	u16		crc;
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  ReqDsp06	// ������ �������� �� ����
//{ 
//	u16		rw; 
//	u16		stAdr; 
//	u16		count; 
//	byte	data[258]; 
//	u16		crc; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  RspDsp06	// ������ �������� �� ����
//{ 
//	u16		rw; 
//	u16		res; 
//	word	crc; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  ReqDsp07	// ������������� �������
//{ 
//	u16		rw; 
//	word 	crc; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __CC_ARM

#else
#pragma pack()
#endif



#endif //G_DSP_H__01_12_2023__12_16
