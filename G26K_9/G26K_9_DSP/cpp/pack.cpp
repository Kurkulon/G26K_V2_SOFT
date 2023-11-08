#pragma optimize_for_speed

#include "hardware.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <fdct_imp.h> 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <wavepack_imp.h> 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_1_BitPack(RSPWAVE *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = 1;
	rsp.hdr.packLen = ((rsp.hdr.sl+3)/4)*3;

	u16 *s = rsp.data;
	u16 *d = rsp.data;

	for (u32 i = (rsp.hdr.sl+3)/4; i > 0; i--)
	{
		*(d++) = (s[0]&0xFFF)|(s[1]<<12);
		*(d++) = ((s[1]>>4)&0xFF)|(s[2]<<8);
		*(d++) = ((s[2]>>8)&0xF)|(s[3]<<4);
		s += 4;
	};

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + rsp.hdr.packLen;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_2_8Bit(RSPWAVE *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = 2;
	rsp.hdr.packLen = (rsp.hdr.sl+1)/2;

	u16 *s = rsp.data;
	byte *d = (byte*)rsp.data;

	for (u32 i = rsp.hdr.packLen*2; i > 0; i--)
	{
		*(d++) = (*(s++)+8)>>4;
	};

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + rsp.hdr.packLen;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_3_uLaw(RSPWAVE *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = 3;
	rsp.hdr.packLen = (rsp.hdr.sl+1)/2;

	WavePack_uLaw_12Bit(rsp.data, (byte*)rsp.data, rsp.hdr.packLen*2);

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + rsp.hdr.packLen;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_4_ADPCMIMA(RSPWAVE *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = 4;
	rsp.hdr.packLen = (rsp.hdr.sl+3)/4;

	WavePack_ADPCMIMA(rsp.data, (byte*)rsp.data, rsp.hdr.packLen*4);

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + rsp.hdr.packLen;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
