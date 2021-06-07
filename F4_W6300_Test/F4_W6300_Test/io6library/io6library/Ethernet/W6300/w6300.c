//*****************************************************************************
//
//! \file w6100.c
//! \brief W6100 HAL Implements file.
//! \version 1.0.0
//! \date 2019/01/01
//! \par  Revision history
//!       <2019/01/01> 1st Release
//! \author MidnightCow
//! \copyright
//!
//! Copyright (c)  2019, WIZnet Co., LTD.
//!
//! Permission is hereby granted, free of charge, to any person obtaining a copy
//! of this software and associated documentation files (the "Software"), to deal
//! in the Software without restriction, including without limitation the rights 
//! to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//! copies of the Software, and to permit persons to whom the Software is 
//! furnished to do so, subject to the following conditions: 
//!
//! The above copyright notice and this permission notice shall be included in
//! all copies or substantial portions of the Software. 
//!
//! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//! SOFTWARE. 
//!
//*****************************************************************************

#include "w6300.h"
#include <stdio.h>
#include <stddef.h>


#define _WIZCHIP_SPI_VDM_OP_    0x00
#define _WIZCHIP_SPI_FDM_LEN1_  0x01
#define _WIZCHIP_SPI_FDM_LEN2_  0x02
#define _WIZCHIP_SPI_FDM_LEN4_  0x03
//
// If you want to use SPI FDM mode, Feel free contact to WIZnet. 
// http://forum.wiznet.io
//

#if _WIZCHIP_ == W6300 
////////////////////////////////////////////////////////////////////////////////////////


#define _W6300_SPI_OP_          _WIZCHIP_SPI_VDM_OP_

//////////////////////////////////////////////////
void WIZCHIP_WRITE(uint32_t AddrSel,  uint16_t wb)
{
   iodata_t tAD[6 / sizeof(iodata_t)];
   AddrSel |= _W6300_RW_WRITE_;

   WIZCHIP_CRITICAL_ENTER();
   WIZCHIP.CS._s_e_l_e_c_t_();

#if( (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_))
   tAD[2] |= (_W6100_SPI_WRITE_ | _W6100_SPI_OP_);
   WIZCHIP.IF.SPI._write_byte_buf(tAD, 4);

#elif ( (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_BUS_INDIR_) )
   #if (_WIZCHIP_IO_BUS_WIDTH_ == 8)
      tAD[0] = (iodata_t)((AddrSel & 0x00FF0000) >> 16);
      tAD[1] = (iodata_t)((AddrSel & 0x0000FF00) >> 8);
      tAD[2] = (iodata_t)((AddrSel & 0x000000FF));
      tAD[3] = (iodata_t)0x00;
      tAD[4] = (iodata_t)(wb >>8);
      tAD[5] = (iodata_t)wb;
      WIZCHIP.IF.BUS._write_data(IDM_AR0, tAD[0]);
      WIZCHIP.IF.BUS._write_data(IDM_AR1, tAD[1]);
      WIZCHIP.IF.BUS._write_data(IDM_BSR, tAD[2]);
      WIZCHIP.IF.BUS._write_data(IDM_OPR, tAD[3]);
      WIZCHIP.IF.BUS._write_data(IDM_DR0, tAD[4]);
      WIZCHIP.IF.BUS._write_data(IDM_DR0, tAD[5]);
   #elif (_WIZCHIP_IO_BUS_WIDTH_ == 16)
      tAD[0] = (iodata_t)(AddrSel >> 8);
      tAD[1] = (iodata_t)((AddrSel & 0x00000FF) << 8);
      tAD[2] = (iodata_t)wb;
      WIZCHIP.IF.BUS._write_data(IDM_AR,  tAD[0]);
      WIZCHIP.IF.BUS._write_data(IDM_BOR, tAD[1]);
      WIZCHIP.IF.BUS._write_data(IDM_DR,  tAD[2]);
   #else
      #error "Abnoraml _WIZCHIP_IO_BUS_WIDTH_. Should be 8 or 16"
   #endif
#else
   #error "Unknown _WIZCHIP_IO_MODE_ in W6300. !!!"
#endif

   WIZCHIP.CS._d_e_s_e_l_e_c_t_();
   WIZCHIP_CRITICAL_EXIT();
}

uint16_t  WIZCHIP_READ(uint32_t AddrSel)
{
   uint16_t ret;
   iodata_t  tAD[4 / sizeof(iodata_t)];
   AddrSel |= _W6300_RW_READ_;

   WIZCHIP_CRITICAL_ENTER();
   WIZCHIP.CS._s_e_l_e_c_t_();

#if( (_WIZCHIP_IO_MODE_ ==  _WIZCHIP_IO_MODE_SPI_VDM_))
   tAD[2] |= (_W6100_SPI_READ_ | _W6100_SPI_OP_);
   WIZCHIP.IF.SPI._write_byte_buf(tAD, 3);
   ret = WIZCHIP.IF.SPI._read_byte();
#elif ( (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_BUS_INDIR_) )
   #if (_WIZCHIP_IO_BUS_WIDTH_ == 8)
      tAD[0] = (iodata_t)(AddrSel >> 16);
      tAD[1] = (iodata_t)(AddrSel >> 8);
      tAD[2] = (iodata_t)AddrSel;
      tAD[3] = 0x00; 
      WIZCHIP.IF.BUS._write_data(IDM_AR0, tAD[0]);
      WIZCHIP.IF.BUS._write_data(IDM_AR1, tAD[1]);
      WIZCHIP.IF.BUS._write_data(IDM_BSR, tAD[2]);
      WIZCHIP.IF.BUS._write_data(IDM_OPR, tAD[3]);
      //WIZCHIP.IF.BUS._write_data_buf(IDM_AR0, tAD, 4, 1);
      ret = (uint16_t) WIZCHIP.IF.BUS._read_data(IDM_DR0);
      ret = (ret << 8) | (((uint16_t)WIZCHIP.IF.BUS._read_data(IDM_DR0)) & 0x00FF);
   #elif (_WIZCHIP_IO_BUS_WIDTH_ == 16)
      tAD[0] = (iodata_t)(AddrSel >> 8);
      tAD[1] = (iodata_t)((AddrSel <<8) & 0x0000FF00);
      WIZCHIP.IF.BUS._write_data(IDM_AR,  tAD[0]);
      WIZCHIP.IF.BUS._write_data(IDM_BOR, tAD[1]);
      //WIZCHIP.IF.BUS._write_data_buf(IDM_AR, tAD, 2, 1);
      ret = (uint16_t) WIZCHIP.IF.BUS._read_data(IDM_DR);
   #else
      #error "Unknown _WIZCHIP_IO_BUS_WIDTH_. Should be 8 or 16"
   #endif
#else
   #error "Unknown _WIZCHIP_IO_MODE_ in W6100. !!!"   
#endif

   WIZCHIP.CS._d_e_s_e_l_e_c_t_();
   WIZCHIP_CRITICAL_EXIT();
   return ret;
}

void WIZCHIP_WRITE_BUF(uint32_t AddrSel, uint8_t* pBuf, datasize_t len)
{
   iodata_t tAD[4/sizeof(iodata_t)];
   datasize_t i;

   AddrSel |= _W6300_RW_WRITE_;
   WIZCHIP_CRITICAL_ENTER();
   WIZCHIP.CS._s_e_l_e_c_t_();

#if((_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_))
   tAD[2] |= (_W6100_SPI_WRITE_ | _W6100_SPI_OP_);

   WIZCHIP.IF.SPI._write_byte_buf(tAD, 3);
   WIZCHIP.IF.SPI._write_byte_buf(pBuf, len);

#elif ( (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_BUS_INDIR_) )
   #if  (_WIZCHIP_IO_BUS_WIDTH_ ==  8)
      tAD[0] = (iodata_t)(AddrSel >> 16);
      tAD[1] = (iodata_t)(AddrSel >> 8);
      tAD[2] = (iodata_t)AddrSel;
      tAD[3] = 0x00; 
    //WIZCHIP.IF.BUS._write_data_buf(IDM_AR0, tAD, 4, 1);
      WIZCHIP.IF.BUS._write_data(IDM_AR0, tAD[0]);
      WIZCHIP.IF.BUS._write_data(IDM_AR1, tAD[1]);
      WIZCHIP.IF.BUS._write_data(IDM_BSR, tAD[2]);
      WIZCHIP.IF.BUS._write_data(IDM_OPR, tAD[3]);
      for(i = 0; i < len; i++)
      {
          WIZCHIP.IF.BUS._write_data(IDM_DR0,pBuf[i]);
      }
    //WIZCHIP.IF.BUS._write_data_buf(IDM_DR0, pBuf, len, 0);
   #elif(_WIZCHIP_IO_BUS_WIDTH_ == 16)
      tAD[0] = (iodata_t)(AddrSel >> 8);
      tAD[1] = (iodata_t)((AddrSel << 8) & 0xFF00);
      WIZCHIP.IF.BUS._write_data(IDM_AR,  tAD[0]);
      WIZCHIP.IF.BUS._write_data(IDM_BOR, tAD[1]);
      for(i=0; i < len ; i += sizeof(iodata_t))
      {
         WIZCHIP.IF.BUS._write_data(IDM_DR, (((iodata_t)pBuf[i]) << 8) + (((iodata_t)pBuf[i+1]) & 0x00FF));
      }
   #else
      #error "Unknown _WIZCHIP_IO_BUS_WIDTH_."
   #endif
#else
   #error "Unknown _WIZCHIP_IO_MODE_ in W6100. !!!!"
#endif

   WIZCHIP.CS._d_e_s_e_l_e_c_t_();
   WIZCHIP_CRITICAL_EXIT();
   
}

void WIZCHIP_READ_BUF (uint32_t AddrSel, uint8_t* pBuf, datasize_t len)
{
   iodata_t tAD[4 / sizeof(iodata_t)];
   datasize_t i;
   AddrSel |= _W6300_RW_READ_;

   WIZCHIP_CRITICAL_ENTER();
   WIZCHIP.CS._s_e_l_e_c_t_();

#if((_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_))
   tAD[2] |= (_W6100_SPI_READ_ | _W6100_SPI_OP_);
   WIZCHIP.IF.SPI._write_byte_buf(tAD,3);
   WIZCHIP.IF.SPI._read_byte_buf(pBuf, len);
#elif ( (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_BUS_INDIR_) )
   #if ( _WIZCHIP_IO_BUS_WIDTH_ == 8 )
      tAD[0] = (iodata_t)(AddrSel >> 16);
      tAD[1] = (iodata_t)(AddrSel >> 8);
      tAD[2] = (iodata_t)AddrSel;
      tAD[3] = 0x00;
      WIZCHIP.IF.BUS._write_data(IDM_AR0, tAD[0]);
      WIZCHIP.IF.BUS._write_data(IDM_AR1, tAD[1]);
      WIZCHIP.IF.BUS._write_data(IDM_BSR, tAD[2]);
      WIZCHIP.IF.BUS._write_data(IDM_OPR, tAD[3]);
    //WIZCHIP.IF.BUS._write_data_buf(IDM_AR0, tAD, 4, 1);
      for(i = 0; i < len; i++)
      {
         pBuf[i] = WIZCHIP.IF.BUS._read_data(IDM_DR0);
      }
    //WIZCHIP.IF.BUS._read_data_buf(IDM_DR0, pBuf, len, 0);
   #elif(_WIZCHIP_IO_BUS_WIDTH_ == 16)
      tAD[0] = (iodata_t)(AddrSel >> 8);
      tAD[1] = (iodata_t)((AddrSel << 8) & 0xFF00);
      WIZCHIP.IF.BUS._write_data(IDM_AR,  tAD[0]);
      WIZCHIP.IF.BUS._write_data(IDM_BOR, tAD[1]);
    //WIZCHIP.BUS.IF._write_data_buf(IDM_AR, tAD, 2, 1);
      for(i = 0; i < len ; i+= sizeof(iodata_t))
      {
         tAD[0]   = (iodata_t)(WIZCHIP.IF.BUS._read_data(IDM_DR));
         pBuf[i]   = (uint8_t)(tAD[0] >> 8);
         pBuf[i+1] = (uint8_t)tAD[0];
      }
   #else
      #error "Unknown _WIZCHIO_IO_BUS_WIDTH_." 
   #endif
#else
   #error "Unknown _WIZCHIP_IO_MODE_ in W6300. !!!!"
#endif
   WIZCHIP.CS._d_e_s_e_l_e_c_t_();
   WIZCHIP_CRITICAL_EXIT();
}

datasize_t getSn_TX_FSR(uint8_t sn)
{
   datasize_t prev_val=-1,val=0;
   do
   {
      prev_val = val;
      val = WIZCHIP_READ(_Sn_TX_FSR_(sn));
      //val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(_Sn_TX_FSR_(sn),1));
   }while (val != prev_val);
   return val;
}

datasize_t getSn_RX_RSR(uint8_t sn)
{
   datasize_t prev_val=-1,val=0;
   do
   {
      prev_val = val;
      val = WIZCHIP_READ(_Sn_RX_RSR_(sn));
      //val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(_Sn_RX_RSR_(sn),1));
   }while (val != prev_val);
   return val;
}

void wiz_send_data(uint8_t sn, uint8_t *wizdata, datasize_t len)
{
   datasize_t ptr = 0;
   
#if (_WIZCHIP_IO_BUS_WIDTH_ == 16)
   datasize_t wrlen = len;
   iodata_t   bor = 0;
   uint16_t   i;
   ptr = getSn_TX_WR(sn);
   bor = (WIZCHIP_TXBUF_BLOCK(sn) | _W6300_RW_WRITE_) << 8;
   WIZCHIP_CRITICAL_ENTER();
   WIZCHIP.CS._s_e_l_e_c_t_();
   WIZCHIP.IF.BUS._write_data(IDM_AR, (iodata_t)ptr);   
   if(((uint32_t)((ptrdiff_t)wizdata)) & 0x01)
   {
      WIZCHIP.IF.BUS._write_data(IDM_BOR,(bor | _W6300_OP_INC12_));
      WIZCHIP.IF.BUS._write_data(IDM_DR, (iodata_t)*wizdata << 8);
      wizdata++;
      wrlen--;
   }
   if(wrlen > 1)
   {
      WIZCHIP.IF.BUS._write_data(IDM_BOR, bor);
   }
   for(i = 0; i < wrlen/2; i++)  // Be care of big or small endian
   {
      WIZCHIP.IF.BUS._write_data(IDM_DR, ((iodata_t)*wizdata << 8) + *(wizdata+1)); 
      wizdata+=sizeof(iodata_t);
   }
   //WIZCHIP.IF.BUS._write_data_buf(IDM_DR,wizdata,wrlen/2,0);   // endian problem
   if(wrlen & 0x01)
   {
      WIZCHIP.IF.BUS._write_data(IDM_BOR, (bor | _W6300_OP_INC12_));
      WIZCHIP.IF.BUS._write_data(IDM_DR,(iodata_t)*wizdata <<8);
   }
   WIZCHIP_CRITICAL_EXIT();
   WIZCHIP.CS._d_e_s_e_l_e_c_t_();
#else
   uint32_t addrsel;
   ptr = getSn_TX_WR(sn);
   addrsel = ((uint32_t)ptr << 8) | WIZCHIP_TXBUF_BLOCK(sn);
   WIZCHIP_WRITE_BUF(addrsel,wizdata, len);
#endif
   ptr += len;
   setSn_TX_WR(sn,ptr);
}

void wiz_recv_data(uint8_t sn, uint8_t *wizdata, datasize_t len)
{
   datasize_t ptr = 0;
  
#if (_WIZCHIP_IO_BUS_WIDTH_ == 16)
   datasize_t rdlen = len;
   uint16_t   bor = 0;
   uint16_t   i;
   iodata_t   rddata;
   if(len == 0) return;
   ptr = getSn_RX_RD(sn);
   bor = (WIZCHIP_RXBUF_BLOCK(sn) | _W6300_RW_READ_) << 8;
   WIZCHIP_CRITICAL_ENTER();
   WIZCHIP.CS._s_e_l_e_c_t_();
   WIZCHIP.IF.BUS._write_data(IDM_AR, (iodata_t)ptr);   
   if(((uint32_t)((ptrdiff_t)wizdata)) & 0x01)
   {
      WIZCHIP.IF.BUS._write_data(IDM_BOR,(bor | _W6300_OP_INC12_));
      *wizdata = (uint8_t)(WIZCHIP.IF.BUS._read_data(IDM_DR) >> 8);
      wizdata++;
      rdlen--;
   }
   if(rdlen > 1)
   {
      WIZCHIP.IF.BUS._write_data(IDM_BOR, bor);
   }
   for(i = 0; i < rdlen/2; i++)  // Be care of big or small endian
   {
      rddata = WIZCHIP.IF.BUS._read_data(IDM_DR);  //when big-endian
      *wizdata     = (uint8_t)(rddata >> 8);
      *(wizdata+1) = (uint8_t)rddata;
      wizdata+=sizeof(iodata_t);
   }
   //WIZCHIP.IF.BUS._read_data_buf(IDM_DR,wizdata,wrlen/2,0);
   if(rdlen & 0x01)
   {
      WIZCHIP.IF.BUS._write_data(IDM_BOR, (bor | _W6300_OP_INC12_));
      *wizdata = (uint8_t)(WIZCHIP.IF.BUS._read_data(IDM_DR) >> 8);
   }
   WIZCHIP_CRITICAL_EXIT();
   WIZCHIP.CS._d_e_s_e_l_e_c_t_();
#else
   uint32_t addrsel;
   if(len == 0) return;
   ptr = getSn_RX_RD(sn);
   addrsel = ((uint32_t)ptr << 8) + WIZCHIP_RXBUF_BLOCK(sn);
   WIZCHIP_READ_BUF(addrsel, wizdata, len);
#endif
   ptr += len;
   setSn_RX_RD(sn,ptr);
}

void wiz_recv_ignore(uint8_t sn, datasize_t len)
{
   setSn_RX_RD(sn,getSn_RX_RD(sn)+len);
}


/// @cond DOXY_APPLY_CODE
#if (_PHY_IO_MODE_ == _PHY_IO_MODE_MII_)
/// @endcond
void wiz_mdio_write(uint8_t phyregaddr, uint16_t var)
{
   setPHYRAR(phyregaddr);
   setPHYDIR(var);
   setPHYACR(PHYACR_WRITE);
   while(getPHYACR());  //wait for command complete
}

uint16_t wiz_mdio_read(uint8_t phyregaddr)
{
   setPHYRAR(phyregaddr);
   setPHYACR(PHYACR_READ);
   while(getPHYACR());  //wait for command complete
   return getPHYDOR();
}
/// @cond DOXY_APPLY_CODE
#endif
/// @endcond

////////////////////////////////////////////////////////////////////////////////////////
#endif
