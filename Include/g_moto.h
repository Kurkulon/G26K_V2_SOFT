#ifndef G_MOTO_H__01_12_2023__12_12
#define G_MOTO_H__01_12_2023__12_12

#pragma once

#include "types.h"

#if defined(__CC_ARM) || defined(_MSC_VER)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MOTO_BOOT_SGUID		0x0A89D55DD5274785 
#define MOTO_BOOT_MGUID		0x9119CC18AC79DE35

#define MOTO_COM_BAUDRATE	1562500
#define MOTO_COM_PARITY		0
#define MOTO_COM_STOPBITS	2

#define MOTO_BOOT_REQ_WORD	0X6600
#define MOTO_BOOT_REQ_MASK	0xFF00
#define MOTO_BOOT_NET_ADR	1

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef BOOTLOADER

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqMoto
{
	u16 	rw;
	u16 	enableMotor; 
	u32		tRPM;			// Частота вращения двигателя (0.01 об/сек)
	u16		limCurrent;		// Ограничение тока двигателя (мА)
	u16		maxCurrent;		// Аварийный ток двигателя (мА)
	u16		gearIndex;		// 0 - maxon 12.25, 1 - assun 13/3 4-pole			
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspMoto
{
	u16 	rw;
	u16 	mororStatus; 
	u16		current;		// Ток двигателя (мА)
	u16		currentLow;		// Ток двигателя, датчик 2 (мА)
	u16		rpm;			// Частота вращения двигателя (0.01 об/сек)
	u16		motoCounter;	// счётчик оборотов двигателя 1/6 оборота
	u16		auxVoltage;		// Напряжение 3-ей жилы (В)
	u16		motoVoltage;	// Напряжение двигателя (В)
	u16		motoDuty;		// Скважность ШИМ двигателя (0.01%)
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct ReqBootMotoHS { unsigned __int64 guid; u16 crc; };
//__packed struct RspBootMotoHS { unsigned __int64 guid; u16 crc; };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct ReqBootMoto
//{
//	union
//	{
//		struct { u32 func; u32 len;								u16 align; u16 crc; }	F1; // Get CRC
//		struct { u32 func;										u16 align; u16 crc; }	F2; // Exit boot loader
//		struct { u32 func; u32 padr; u32 plen; u32 pdata[16];	u16 align; u16 crc; }	F3; // Programm page
//	};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct RspBootMoto
//{
//	union
//	{
//		struct { u32 func; u32 pageLen;	u32 len;	u16 sCRC;	u16 crc; }	F1; // Get CRC
//		struct { u32 func;							u16 align;	u16 crc; } 	F2; // Exit boot loader
//		struct { u32 func; u32 padr;	u32 status; u16 align;	u16 crc; } 	F3; // Programm page
//	};
//};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // BOOTLOADER

#endif // __CC_ARM


#endif //G_MOTO_H__01_12_2023__12_12
