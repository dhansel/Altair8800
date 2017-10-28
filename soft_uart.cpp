/**
 ** soft_uart library
 ** Copyright (C) 2015
 **
 **   Antonio C. Domínguez Brito <adominguez@iusiani.ulpgc.es>
 **     División de Robótica y Oceanografía Computacional <www.roc.siani.es>
 **     and Departamento de Informática y Sistemas <www.dis.ulpgc.es>
 **     Universidad de Las Palmas de Gran  Canaria (ULPGC) <www.ulpgc.es>
 **  
 ** This file is part of the soft_uart library.
 ** The soft_uart library is free software: you can redistribute it and/or modify
 ** it under  the  terms of  the GNU  General  Public  License  as  published  by
 ** the  Free Software Foundation, either  version  3  of  the  License,  or  any
 ** later version.
 ** 
 ** The  soft_uart library is distributed in the hope that  it  will  be  useful,
 ** but   WITHOUT   ANY WARRANTY;   without   even   the  implied   warranty   of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR  PURPOSE.  See  the  GNU  General
 ** Public License for more details.
 ** 
 ** You should have received a copy  (COPYING file) of  the  GNU  General  Public
 ** License along with the soft_uart library.
 ** If not, see: <http://www.gnu.org/licenses/>.
 **/
/*
 * File: soft_uart.cpp 
 * Description:  This  is  an  implementation  of  a  software	UART  (Universal
 * Asynchronous Receiver  Transmitter)	library  for  the  Arduino  Due's  Atmel
 * ATSAM3X8E micro-controller. (implementation file)
 * Date: June 22nd, 2015
 * Author: Antonio C. Dominguez-Brito <adominguez@iusiani.ulpgc.es>
 * ROC-SIANI - Universidad de Las Palmas de Gran Canaria - Spain
 */

#ifdef __SAM3X8E__

#include "soft_uart.h"

namespace arduino_due
{

  namespace soft_uart
  {

    tc_timer_data 
      tc_timer_table[static_cast<uint32_t>(timer_ids::TIMER_IDS)]=
    {
      {TC0,0,TC0_IRQn},
      {TC0,1,TC1_IRQn},
      {TC0,2,TC2_IRQn},
      {TC1,0,TC3_IRQn},
      {TC1,1,TC4_IRQn},
      {TC1,2,TC5_IRQn},
      {TC2,0,TC6_IRQn},
      {TC2,1,TC7_IRQn},
      {TC2,2,TC8_IRQn},
    };
  
  }


}

#endif
