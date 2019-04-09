#define NEW_PRINTF_SEMANTICS
#include "printf.h"

configuration BaseStationC {
}
implementation {
  components MainC, BaseStationP, LedsC;
  components ActiveMessageC as Radio, SerialActiveMessageC as Serial;
   components new TimerMilliC() as Timer1;
  
  MainC.Boot <- BaseStationP;

  BaseStationP.Boot -> MainC;

  BaseStationP.RadioControl -> Radio;
  BaseStationP.SerialControl -> Serial;
  
  BaseStationP.UartSend -> Serial;
  BaseStationP.UartReceive -> Serial.Receive;
  BaseStationP.UartPacket -> Serial;
  BaseStationP.UartAMPacket -> Serial;
  
  BaseStationP.RadioSend -> Radio;
  BaseStationP.RadioReceive -> Radio.Receive;
//  BaseStationP.RadioSnoop -> Radio.Snoop;
  BaseStationP.RadioPacket -> Radio;
  BaseStationP.RadioAMPacket -> Radio;
  
  BaseStationP.Leds -> LedsC;
  
  BaseStationP.Timer0 -> Timer1;
  BaseStationP.PacketTimeStamp -> Radio;

  components new TimerMilliC();
  components PrintfC;
  components SerialStartC;

  BaseStationP.Boot -> MainC;
}
