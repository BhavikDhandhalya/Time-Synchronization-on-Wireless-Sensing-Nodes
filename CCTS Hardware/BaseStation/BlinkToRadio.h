// $Id: BlinkToRadio.h,v 1.4 2006-12-12 18:22:52 vlahan Exp $

#ifndef BLINKTORADIO_H
#define BLINKTORADIO_H

enum {
  AM_BLINKTORADIOMSG = 6,
  TIMER_PERIOD_MILLI = 1000
};

typedef nx_struct BlinkToRadioMsg {
  nx_uint8_t nodeid;
  nx_uint32_t counter;
  nx_uint32_t p_sendts;
  nx_uint32_t receivets;
  nx_float virtual_clk;
  nx_float skew_cmp;
  nx_float offset_cmp;
} BlinkToRadioMsg;

#endif
