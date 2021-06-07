#ifndef __IOTRNG_H__
#define __IOTRNG_H__

#include "io6Library.h"

#define _RNGSTR_            (_W6100_IO_BASE_ + (0x4300 << 8) + WIZCHIP_CREG_BLOCK)
#define _RNGLTR_            (_W6100_IO_BASE_ + (0x4301 << 8) + WIZCHIP_CREG_BLOCK)
#define _RNGCR_             (_W6100_IO_BASE_ + (0x4302 << 8) + WIZCHIP_CREG_BLOCK)
#define _RNGPLR_            (_W6100_IO_BASE_ + (0x4303 << 8) + WIZCHIP_CREG_BLOCK)
#define _RNGSDR_            (_W6100_IO_BASE_ + (0x4307 << 8) + WIZCHIP_CREG_BLOCK)
#define _RNGCHKR_           (_W6100_IO_BASE_ + (0x4311 << 8) + WIZCHIP_CREG_BLOCK)
#define _RNGVR_             (_W6100_IO_BASE_ + (0x4312 << 8) + WIZCHIP_CREG_BLOCK)
#define _RNGCPR_            (_W6100_IO_BASE_ + (0x4316 << 8) + WIZCHIP_CREG_BLOCK)
#define _RNGCDR_            (_W6100_IO_BASE_ + (0x4317 << 8) + WIZCHIP_CREG_BLOCK)
#define _TRNGFLGR_          (_W6100_IO_BASE_ + (0x4320 << 8) + WIZCHIP_CREG_BLOCK)
#define _TRNGVALR_          (_W6100_IO_BASE_ + (0x4322 << 8) + WIZCHIP_CREG_BLOCK)


#define getRNGSTR() \
        WIZCHIP_READ(_RNGSTR_) 
#define setRNGSTR(rngstart) \
        WIZCHIP_WRITE(_RNGSTR_,rngstart)

#define getRNGLTR() \
        WIZCHIP_READ(_RNGLTR_)
#define setRNGLTR(rnglatch) \
        WIZCHIP_WRITE(rnglatch)

#define getRNGCR() \
        WIZCHIP_READ(_RNGCR_)
#define setRNGCR() \
        WIZCHIP_WRITE(_RNGCR_, rngctrl)

#define getRNGPLR() \
        (                                                                  \
          ((uint32_t)WIZCHIP_READ(_RNGPLR_)) << 24 +                       \
          ((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(_RNGPLR_,1))) << 16 + \
          ((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(_RNGPLR_,2))) <<  8 + \
          ((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(_RNGPLR_,3)))         \
        )
 
#define setRNGPLR(poly) \
        do{                                                                    \
           WIZCHIP_WRITE(_RNGPLR_, (uint8_t)(poly>>24));                       \
           WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(_RNGPLR_,1), (uint8_t)(poly>>16)); \
           WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(_RNGPLR_,2), (uint8_t)(poly>> 8)); \
           WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(_RNGPLR_,3), (uint8_t)poly);       \
        }while(0);

#define getRNGCHKR() \
        WIZCHIP_READ(_RNGCHKR_)

#define getRNGVR() \
        (                                                                 \
          ((uint32_t)WIZCHIP_READ(_RNGVR_)) << 24 +                       \
          ((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(_RNGVR_,1))) << 16 + \
          ((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(_RNGVR_,2))) <<  8 + \
          ((uint32_t)WIZCHIP_READ(WIZCHIP_OFFSET_INC(_RNGVR_,3)))         \
        )
        
#define getRNGCDR() \
        WIZCHIP_READ(_RNGCDR_)
           
void     trng_start();
void     trng_stop();
uint32_t get_true_random_num();




#endif
