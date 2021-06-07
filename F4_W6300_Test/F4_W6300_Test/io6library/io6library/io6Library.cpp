#include "io6Library.h"


#ifndef _WIZCHIP_
#error Undefined _WIZCHIP_
#endif

#if _WIZCHIP_ == W6100
   #warning "w6100.h include"
   #include "W6100/w6100.c"
#elif _WIZCHIP_ == W6300
   #warning "w6300.h include"
   #include "W6300/w6300.c"
#else
   #error  "Undefined _WIZCHIP_"
#endif


#include "wizchip_conf.c"

/*
#ifdef __cplusplus
namespace IO6LIB
{
#endif

#include "socket.c"

#ifdef __cplusplus
}
#endif
*/
