//#pragma optimize_for_speed

#include "pack.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <fdct_imp.h> 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <wavepack_imp.h> 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static FDCT_DATA	fdct_w[FDCT_N];

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_1_Bit12(RSPWAVE *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = PACK_BIT12;
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

void Pack_2_uLaw(RSPWAVE *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = PACK_ULAW;
	rsp.hdr.packLen = (rsp.hdr.sl+1)/2;

	WavePack_uLaw_12Bit(rsp.data, (byte*)rsp.data, rsp.hdr.packLen*2);

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + rsp.hdr.packLen;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_3_ADPCMIMA(RSPWAVE *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = PACK_ADPCM;
	rsp.hdr.packLen = (rsp.hdr.sl+3)/4;

	WavePack_ADPCMIMA(rsp.data, (byte*)rsp.data, rsp.hdr.packLen*4);

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + rsp.hdr.packLen;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_FDCT_Transform(i16 *s)
{
	for (u32 n = 0; n < FDCT_N; n++) fdct_w[n] = *(s++);

	FastDctLee_transform(fdct_w, FDCT_LOG2N);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 Pack_FDCT_Quant(u16 packLen, byte shift, u16* const scale)
{
	packLen = (packLen+1) & ~1;

	FDCT_DATA max = 0;

	for (u32 i = 1; i < packLen; i++)
	{
		FDCT_DATA t = fdct_w[i];

		if (t < 0) t = -t;

		if (t > max) max = t;
	};

	FDCT_DATA *p = fdct_w + packLen - 1;
	FDCT_DATA lim = max;

	if (lim < 64) lim = 64;

	lim = (i32)lim >> shift;

	*scale = 0;

	if (fdct_w[0] > max) max = fdct_w[0];

	while (max > 32000) { max /= 2; *scale += 1; };

	u32 xx = 8 << *scale;

	if (lim < xx) lim = xx;

	for (u32 i = packLen; i > 2; i--)
	{
		FDCT_DATA t = *(p--);

		if (t < 0) t = -t;

		if (t > lim)
		{
			packLen = i;
			break;
		};

		packLen -= 1;
	};

	return (packLen+1) & ~1;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_FDCT_uLaw(byte *d, u16 len, byte scale)
{
	FDCT_DATA *s = fdct_w;

    byte sign, exponent, mantissa, sample_out;

	for (; len > 0; len--)
	{
		u16 sample_in = (i16)((i32)(*(s++))>>scale);

		sign = 0;

		if ((i16)sample_in < 0)
		{
			sign = 0x80;
			sample_in = -sample_in;
		};

		//if (sample_in > ulaw_0816_clip) sample_in = ulaw_0816_clip;

		sample_in += 0x84;//ulaw_0816_bias;

		exponent = ulaw_0816_expenc[(sample_in >> 7) & 0xff];

		mantissa = (sample_in >> (exponent + 3)) & 0xf;

		sample_out = (sign | (exponent << 4) | mantissa);

		//if (sample_out == 0) sample_out = 2;

		*(d++) = sample_out;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_Init()
{
	__breakpoint();

	FDCT_Init();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
