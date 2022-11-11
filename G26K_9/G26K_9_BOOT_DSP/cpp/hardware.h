#ifndef HARDWARE_H__15_05_2009__14_35
#define HARDWARE_H__15_05_2009__14_35
  
#include <types.h>

#include "hw_conf.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct  ReqDsp05 { u16 rw; u16 crc; };												// ������ ����������� ����� � ����� ��������� �� ����-������
struct  ReqDsp06 { u16 rw; u16 stAdr; u16 len; byte data[256]; u16 crc; };			// ������ �������� �� ����
struct  ReqDsp07 { u16 rw; word crc; };												// ������������� �������

struct  RspDsp05 { u16 rw; u16 flashLen; u32 startAdr; u16 flashCRC; u16 crc; };	// ������ ����������� ����� � ����� ��������� �� ����-������
struct  RspDsp06 { u16 rw; u16 res; u16 crc; };										// ������ �������� �� ����

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();
extern void InitIVG(u32 IVG, u32 PID, void (*EVT)());

extern void WriteTWI(void *src, u16 len);
extern void ReadTWI(void *dst, u16 len);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#endif // HARDWARE_H__15_05_2009__14_35
