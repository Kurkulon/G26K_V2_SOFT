#ifndef PACK_H__08_11_2023__17_20
#define PACK_H__08_11_2023__17_20
  
#include "hardware.h"
#include "fdct.h"

#define FDCT_LOG2N 6
#define FDCT_N (1UL<<FDCT_LOG2N)

enum Pack { PACK_NO = 0, PACK_BIT12, PACK_ULAW, PACK_ADPCM, PACK_DCT0, PACK_DCT1, PACK_DCT2 };

extern void Pack_1_Bit12(RSPWAVE *dsc);
extern void Pack_2_uLaw(RSPWAVE *dsc);
extern void Pack_3_ADPCMIMA(RSPWAVE *dsc);

extern void Pack_FDCT_Transform(i16 *s);
extern u16	Pack_FDCT_Quant(u16 packLen, byte shift, u16 * const scale);
extern void Pack_FDCT_uLaw(byte *d, u16 len, byte scale);
extern void Pack_Init();

#endif // PACK_H__08_11_2023__17_20
