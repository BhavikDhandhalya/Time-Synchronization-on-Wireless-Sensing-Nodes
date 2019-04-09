/*
@Author: Bhavik Dhandhalya
@Author: Deependra Singh

1st year M.E. Computer Science Students.
Birla Institute of Technology & Science, Pilani.

email[Bhavik]:    h20180118@pilani.bits-pilani.ac.in
email[Deependra]: h20180132@pilani.bits-pilani.ac.in

Paper: Cluster-Based Consensus Time Synchronization for Wireless Sensor Networks
equation number are written in the comments of the code.

Explanation of the code is only written in the 
1. BaseStationP.nc &
2. BlinkToRadioC.nc
*/

#include <Timer.h>
#include "BlinkToRadio.h"

module BlinkToRadioC {
  uses interface Boot;
  uses interface Leds;
  uses interface Timer<TMilli> as Timer0;
  uses interface Packet;
  uses interface AMPacket;
  uses interface AMSend;
  uses interface Receive ;
  uses interface SplitControl as AMControl;
  uses interface PacketTimeStamp<TMicro,uint32_t>;
}
implementation {

  message_t pkt;
  message_t* pktptr;
  bool busy = FALSE;
  uint32_t prevtimestamp;
  uint32_t rectimestamp;

  /*N is number of child nodes*/
  uint32_t N = 3;
  uint32_t receiveTS;
  uint32_t resendTS;
  uint32_t received_counter;
  
  uint32_t sendTS_p, receiveTS_p, resendTS_p, endTS_p;
  uint32_t pp_sendTS, p_sendTS, p_receiveTS, pp_receiveTS;
  float m_offset_cmp;
  float m_prev_offset_cmp;
  float avg_m_relative_skew_cmp;
  float h_virtual_clk;
  float h_prev_skew_cmp;
  float prev_v_clk;
  float m_skew_cmp;
  float m_prev_skew_cmp;
  float m_vc;
  float h_skew;
  float roww;


  void setLeds(uint16_t val) {
    if (val & 0x01)
      call Leds.led0On();
    else 
      call Leds.led0Off();
    if (val & 0x02)
      call Leds.led1On();
    else
      call Leds.led1Off();
    if (val & 0x04)
      call Leds.led2On();
    else
      call Leds.led2Off();
  }

  event void Boot.booted() {
    call AMControl.start();
    m_skew_cmp = 1.0;
    m_offset_cmp = 0.0;
    m_prev_skew_cmp = 1.0;
  }

  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
  }

  event void Timer0.fired() {
    if (call AMSend.send(AM_UNICAST_ADDR, pktptr, sizeof(BlinkToRadioMsg)) == SUCCESS)  {
			busy = TRUE;
			call Leds.led2Toggle();
	  }
  }

  event void AMSend.sendDone(message_t* msg, error_t err) {
    if (pktptr == msg) {
      busy = FALSE;
      resendTS = call PacketTimeStamp.timestamp(msg);
    }
  }

  
  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
    if (len == sizeof(BlinkToRadioMsg)) {
		  BlinkToRadioMsg* btrpkt = (BlinkToRadioMsg*)payload;
		  call Leds.led1Toggle();
      	  
      
		  if (!busy) {
        /*Reply to only base station packets*/
        if (btrpkt->nodeid == 10) {

          received_counter = btrpkt->counter;
          btrpkt->nodeid = TOS_NODE_ID;

          call Leds.led0Toggle();

          /*As described in Base Station file, 2 iterations are necessary to initialize synchronization
          variables*/
          if (received_counter == 1) {
            /* Recording receiving time stamp */
            receiveTS = call PacketTimeStamp.timestamp(msg);
            btrpkt->receivets = receiveTS;
            btrpkt->skew_cmp = m_skew_cmp;
            btrpkt->offset_cmp = m_offset_cmp;

            pktptr = msg;

            /* Sending packets with 1 time delay to avoid packet collission with other members */
            call Timer0.startOneShot(1);

          } else if (received_counter == 2) {
            p_receiveTS = receiveTS;
            receiveTS = call PacketTimeStamp.timestamp(msg);
            btrpkt->receivets = receiveTS;
            btrpkt->skew_cmp = m_skew_cmp;
            btrpkt->offset_cmp = m_offset_cmp;
            btrpkt->virtual_clk = receiveTS;
            p_sendTS = btrpkt->p_sendts;
            h_prev_skew_cmp = btrpkt->skew_cmp;

            prev_v_clk = receiveTS;

            pktptr = msg;

            call Timer0.startOneShot(1);

          } else {
            /* CALCULATIONS */

            pp_receiveTS = p_receiveTS;
            p_receiveTS = btrpkt->receivets;

            pp_sendTS = p_sendTS;
            p_sendTS = btrpkt->p_sendts;

            h_virtual_clk = btrpkt->virtual_clk;
            h_skew = btrpkt->skew_cmp;

            /* equation(19) from CCTS paper */
            /*h_virtual_clk is of previous iteration*/
            m_offset_cmp = m_prev_offset_cmp + (0.5)*(h_virtual_clk - prev_v_clk);

            if (p_receiveTS - pp_receiveTS != 0)
              avg_m_relative_skew_cmp = (float)(p_sendTS - pp_sendTS)/(float)(p_receiveTS - pp_receiveTS);

            /* equation(13) */
            roww = (N * 1.0)/ (float)(N + 1);
            m_skew_cmp = (1.00 - roww) * (float)m_prev_skew_cmp;
            m_skew_cmp = m_skew_cmp + (float)(0.5)*((float)h_prev_skew_cmp)*((float)avg_m_relative_skew_cmp);

            /* equation(7) */
            m_vc = (float)m_skew_cmp * (float)receiveTS + (float)m_offset_cmp;

            prev_v_clk = m_vc;
            m_prev_offset_cmp = m_offset_cmp;
            m_prev_skew_cmp = m_skew_cmp;

            /* updating packet details */
            receiveTS = call PacketTimeStamp.timestamp(msg);
            btrpkt->receivets = receiveTS;
            btrpkt->virtual_clk = m_vc;
            btrpkt->skew_cmp = m_skew_cmp;
            btrpkt->offset_cmp = m_offset_cmp;

            h_prev_skew_cmp = btrpkt->skew_cmp;

            pktptr = msg;

            call Timer0.startOneShot(1);

          }
        }
     } // end of(!busy)

      
    }// end of if(len)
    return msg;
  }// end of event Receive.receive
}
