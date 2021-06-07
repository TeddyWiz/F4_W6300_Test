#ifndef  _IO6LIBRARY_H_
#define _IO6LIBRARY_H_


#include "wizchip_conf.h"

#if   _WIZCHIP_ == W6100
   #warning "w6100.h included"
   #include "W6100/w6100.h"
#elif _WIZCHIP_ == W6300
   #warning "w6300.h included"
   #include "W6300/w6300.h"
#else
  #error "Undefined _WIZCHIP_"
#endif  

/*
#ifdef __cplusplus
namespace IO6LIB
{
#endif

#include "socket.h"

#ifdef __cplusplus
}
#endif
*/

#endif
