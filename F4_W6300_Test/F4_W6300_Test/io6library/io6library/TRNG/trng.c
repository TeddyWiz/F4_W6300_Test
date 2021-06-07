#include "trng.h"

void     trng_start()
{
   uint8_t crypt;
   setRNGSTR(1);
   setRNGCR((1<<2) | (1<<1));
   while(!getRNGCHKR());
   crypt = getRNGCDR();
   while(crypt != getRNGCDR());
}

void     trng_stop()
{
   setRNGSTR(0);
}

uint32_t get_trng_num()
{
   return getRNGVR();
}


