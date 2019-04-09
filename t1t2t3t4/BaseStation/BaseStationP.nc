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

#include "printf.h"
#include "AM.h"
#include "Serial.h"
#include <Timer.h>
#include "BlinkToRadio.h"
#include <math.h>

module BaseStationP @safe() {
  uses {
    interface Boot;
    interface SplitControl as SerialControl;
    interface SplitControl as RadioControl;

    interface AMSend as UartSend[am_id_t id];
    interface Receive as UartReceive[am_id_t id];
    interface Packet as UartPacket;
    interface AMPacket as UartAMPacket;
    
    interface AMSend as RadioSend[am_id_t id];
    interface Receive as RadioReceive[am_id_t id];
    interface Packet as RadioPacket;
    interface AMPacket as RadioAMPacket;
    
    interface Timer<TMilli> as Timer0;
    interface PacketTimeStamp<TMicro,uint32_t>;

    interface Leds;
  }
}

implementation
{
  enum {
    UART_QUEUE_LEN = 100,
    RADIO_QUEUE_LEN = 20,
    N = 3
  };

  message_t  uartQueueBufs[UART_QUEUE_LEN];
  message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
  uint8_t    uartIn, uartOut;
  bool       uartBusy, uartFull;

  message_t  radioQueueBufs[RADIO_QUEUE_LEN];
  message_t  * ONE_NOK radioQueue[RADIO_QUEUE_LEN];
  uint8_t    radioIn, radioOut;
  bool       radioBusy, radioFull;

  uint16_t p;

  /**/
  /*Number of child nodes*/
  //uint32_t N;
  uint32_t sendTS;
  uint32_t endTS;
  uint32_t prev_sendTS;


  float avg_v_clk;
  float h_offset_cmp;
  float h_prev_offset_cmp;
  float prev_v_clk;
  float avg_h_relative_skew_cmp;
  float avg_skew_cmp;
  float h_skew_cmp;
  float h_prev_skew_cmp;
  float h_vc;
  float roww;
  float one_r, two_r, final_r;

  float v_clk_member[N];
  uint32_t array_receivets[N];
  float skew_cmp_of_member[N];
  uint32_t p_array_receivets[N]; 
  float rel_skew[N];


  task void uartSendTask();
  task void radioSendTask();

  void dropBlink() {
    call Leds.led2Toggle();
  }

  void failBlink() {
    call Leds.led2Toggle();
  }

  event void Boot.booted() {
    uint8_t i;

    h_skew_cmp = 1.0;
    h_offset_cmp = 0.0;
    h_prev_skew_cmp = 1.0;

    /*Initialization part for N child nodes*/
    for (i = 0; i < N; i++) {
      v_clk_member[i] = 0.0;
      array_receivets[i] = 0.0;
      skew_cmp_of_member[i] = 0.0;
    }

    for (i = 0; i < UART_QUEUE_LEN; i++)
      uartQueue[i] = &uartQueueBufs[i];
    uartIn = uartOut = 0;
    uartBusy = FALSE;
    uartFull = TRUE;

    for (i = 0; i < RADIO_QUEUE_LEN; i++)
      radioQueue[i] = &radioQueueBufs[i];
    radioIn = radioOut = 0;
    radioBusy = FALSE;
    radioFull = TRUE;

    if (call RadioControl.start() == EALREADY) 
      radioFull = FALSE;
    if (call SerialControl.start() == EALREADY)
      uartFull = FALSE;
  }

  event void RadioControl.startDone(error_t error) {
    if (error == SUCCESS) {
      radioFull = FALSE;
      call Timer0.startPeriodic(TIMER_PERIOD_MILLI);
    }
  }

  event void SerialControl.startDone(error_t error) {
    if (error == SUCCESS) {
      uartFull = FALSE;
    }
  }

  event void SerialControl.stopDone(error_t error) {}
  event void RadioControl.stopDone(error_t error) {}

  message_t pkt;
  uint16_t counter;
  uint32_t timestamp;
  message_t* msgaa;
  am_id_t idd;
  bool Test = FALSE;
  bool both = TRUE;
  bool got_node_one = FALSE;

  uint8_t AM_UNICASTADDR = 1;
    
  event void Timer0.fired() {

    if (!Test){
      
      if (!radioBusy) {
          

        BlinkToRadioMsg* btrpkt = (BlinkToRadioMsg*)(call RadioPacket.getPayload(&pkt, sizeof(BlinkToRadioMsg)));

        counter++;
        if (counter <= 75) {
          
          btrpkt->nodeid = TOS_NODE_ID;
          btrpkt->counter = counter;
          btrpkt->sendTS = sendTS;

          idd = call UartAMPacket.type(msgaa);
            
          if (call RadioSend.send[AM_BLINKTORADIOMSG](AM_BROADCAST_ADDR,&pkt,sizeof(BlinkToRadioMsg)) == SUCCESS) {
            call Leds.led0Toggle();
            radioBusy = TRUE;
          }

        } 
        
      } // end of if(!radioBusy)
    } // end of if(!Test)
    else {
      call Timer0.stop();
    }
  }
 

  message_t* ONE receive(message_t* ONE msg, void* payload, uint8_t len);

  
  uint32_t rectimestamp;
  uint8_t   i = 0;            // no. of packets received
  uint8_t current_node;

  event message_t *RadioReceive.receive[am_id_t id](message_t *msg,   void *payload,  uint8_t len) {
    BlinkToRadioMsg* btrpkt = (BlinkToRadioMsg*)payload;

    if (btrpkt->nodeid >= 5) {
      printf("received\n");
    }

    if (!Test && i == 0) {
      btrpkt->receiveTS_of_head = endTS;
      endTS = call PacketTimeStamp.timestamp(msg);
      i++;
    } else {

      if (!Test) {
       btrpkt->receiveTS_of_head = endTS;
        endTS = call PacketTimeStamp.timestamp(msg);
        //btrpkt->receiveTS_of_head = endTS; 
      }
      else {
        btrpkt->receiveTS_of_head = endTS;
        endTS = call PacketTimeStamp.timestamp(msg);
      //btrpkt->p_endts = endTS;
        //btrpkt->receiveTS_of_head = endTS; 
      }

    /* i/N iterations */
      i++;
      if (i == 250) {
        if (!Test) {
          i = 0;
          AM_UNICASTADDR++;
          //if (AM_UNICASTADDR == 6)
            //Test = TRUE;
          Test = TRUE;
        }
      }
      
    }

    return receive(msg, payload, len);
  }


  message_t* receive(message_t *msg, void *payload, uint8_t len) {
    message_t *ret = msg;

    atomic {
      if (!uartFull)
      {
        ret = uartQueue[uartIn];
        uartQueue[uartIn] = msg;

        uartIn = (uartIn + 1) % UART_QUEUE_LEN;

        if (uartIn == uartOut)
          uartFull = TRUE;

        if (!uartBusy)
        {
          post uartSendTask();
          uartBusy = TRUE;
        }
      }
      else
        dropBlink();
    }
    
    return ret;
  }

  uint8_t tmpLen;
  
  task void uartSendTask() {
    uint8_t len;
    am_id_t id;
    am_addr_t addr, src;
    message_t* msg;
    am_group_t grp;
    atomic
    if (uartIn == uartOut && !uartFull)
    {
      uartBusy = FALSE;
      return;
    }

    msg = uartQueue[uartOut];
    tmpLen = len = call RadioPacket.payloadLength(msg);
    id = call RadioAMPacket.type(msg);
    addr = call RadioAMPacket.destination(msg);
    src = call RadioAMPacket.source(msg);
    grp = call RadioAMPacket.group(msg);
    call UartPacket.clear(msg);
    call UartAMPacket.setSource(msg, src);
    call UartAMPacket.setGroup(msg, grp);

    if (call UartSend.send[id](addr, uartQueue[uartOut], len) == SUCCESS)
      call Leds.led1Toggle();
    else
    {
      failBlink();
      post uartSendTask();
    }
  }

  event void UartSend.sendDone[am_id_t id](message_t* msg, error_t error) {
    if (error != SUCCESS)
      failBlink();
    else
      atomic
    if (msg == uartQueue[uartOut])
    {
      if (++uartOut >= UART_QUEUE_LEN)
        uartOut = 0;
      if (uartFull)
        uartFull = FALSE;
    }
    post uartSendTask();
  }

  event message_t *UartReceive.receive[am_id_t id](message_t *msg,
   void *payload,
   uint8_t len) {
    message_t *ret = msg;
    bool reflectToken = FALSE;

    atomic
    if (!radioFull)
    {
      reflectToken = TRUE;
      ret = radioQueue[radioIn];
      radioQueue[radioIn] = msg;
      if (++radioIn >= RADIO_QUEUE_LEN)
        radioIn = 0;
      if (radioIn == radioOut)
        radioFull = TRUE;

      if (!radioBusy)
      {
        post radioSendTask();
        radioBusy = TRUE;
      }
    }
    else
      dropBlink();

    if (reflectToken) {
      //call UartTokenReceive.ReflectToken(Token);
    }
    
    return ret;
  }

  task void radioSendTask() {
    uint8_t len;
    am_id_t id;
    am_addr_t addr,source;
    message_t* msg;
    
    atomic
    if (radioIn == radioOut && !radioFull)
    {
      radioBusy = FALSE;
      return;
    }

    msg = radioQueue[radioOut];
    len = call UartPacket.payloadLength(msg);
    addr = call UartAMPacket.destination(msg);
    source = call UartAMPacket.source(msg);
    id = call UartAMPacket.type(msg);

    call RadioPacket.clear(msg);
    call RadioAMPacket.setSource(msg, source);
    
    if (call RadioSend.send[id](addr, msg, len) == SUCCESS)
      call Leds.led0Toggle();
    else
    {
      failBlink();
      post radioSendTask();
    }
  }

  event void RadioSend.sendDone[am_id_t id](message_t* msg, error_t error) {
    
    radioBusy = FALSE;
    /* Storing timestamp */
    sendTS = call PacketTimeStamp.timestamp(msg);
    
    if (error != SUCCESS)
      failBlink();
    else
      atomic
      if (msg == radioQueue[radioOut]) {
        if (++radioOut >= RADIO_QUEUE_LEN)
          radioOut = 0;
        if (radioFull)
          radioFull = FALSE;
      }
    
    post radioSendTask();
  }
}  
