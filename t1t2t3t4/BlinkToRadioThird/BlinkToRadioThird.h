// $Id: BlinkToRadio.h,v 1.4 2006-12-12 18:22:52 vlahan Exp $

#ifndef BLINKTORADIO_H
#define BLINKTORADIO_H

enum {
  AM_BLINKTORADIOMSG = 6,
  TIMER_PERIOD_MILLI = 500
};

typedef nx_struct BlinkToRadioMsg {
  nx_uint8_t nodeid;
  nx_uint8_t counter;
  nx_uint32_t sendTS;
  nx_uint32_t receiveTS;
  nx_uint32_t sendTS_of_member;
  nx_uint32_t receiveTS_of_head;
} BlinkToRadioMsg;

#endif
