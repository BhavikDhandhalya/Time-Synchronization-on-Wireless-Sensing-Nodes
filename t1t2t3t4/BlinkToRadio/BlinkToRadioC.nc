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
        if (btrpkt->nodeid == 10 || btrpkt->nodeid == 35) {

          received_counter = btrpkt->counter;
          btrpkt->nodeid = TOS_NODE_ID;

          call Leds.led0Toggle();

          if (1 && received_counter <= 25) {
            /* Recording receiving time stamp */
            btrpkt->receiveTS = receiveTS;
            receiveTS = call PacketTimeStamp.timestamp(msg);
            btrpkt->nodeid = TOS_NODE_ID;
            
            btrpkt->sendTS_of_member = resendTS;

            pktptr = msg;

            /* Sending packets with 1 time delay to avoid packet collission with other members */
            call Timer0.startOneShot(1);

          }
        }
     } // end of(!busy)

      
    }// end of if(len)
    return msg;
  }// end of event Receive.receive
}
