#ifndef PACK_H__08_11_2023__17_20
#define PACK_H__08_11_2023__17_20
  
#include "hardware.h"

extern void Pack_1_BitPack(RSPWAVE *dsc);
extern void Pack_2_8Bit(RSPWAVE *dsc);
extern void Pack_3_uLaw(RSPWAVE *dsc);
extern void Pack_4_ADPCMIMA(RSPWAVE *dsc);

#endif // PACK_H__08_11_2023__17_20
