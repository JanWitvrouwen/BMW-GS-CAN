#include "esp32_can.h"            // https://github.com/collin80/esp32_can AND https://github.com/collin80/can_common
#include "BluetoothSerial.h"

#include "bmw_state.h"

#define BLUE_LED          13  // GPIO13  (GPIO4 on v2.0)
#define YELLOW_LED        12  // GPIO12
#define FORCE_KEEP_ON     25  // GPIO25 

// MSG IDs for modules on a BMW R1200 GS, K25
#define MSG_ID_BMSK_Control_Module     0x10C // Throttle position, engine rpm, clutch lever, kill switch & side stand switch
#define MSG_ID_BMSK_Control_Module_2   0x2BC // Gears, Engine Temperature & Air Temperature

#define MSG_ID_ZFE_Control_Module      0x130 // Lighting: turn signals & high beam
#define MSG_ID_ZFE_Control_Module_2    0x2D0 // LAMPF, Fuel Level & Control buttons: info, heated grip & esa

#define MSG_ID_ABS_Control_Module      0x294 // Speed and distance traveled on front wheel, ABS status & brake levers
#define MSG_ID_ABS_Control_Module_2    0x2A8 // Speed and distance traveled on rear wheel

#define MSG_ID_Instrument_Cluster      0x3F8 // Odometer in kilometers
#define MSG_ID_Instrument_Cluster_2    0x3FF // Ambient Light, Clock


BluetoothSerial SerialBT;

/* Globals */
K25_State_t motorcycle_state;
bool status_changed = true;


void setup() {
  SerialBT.begin("GS-CAN");
  
  
  delay(200);
  SerialBT.println("\nBooting CAN getall");

  pinMode(26, OUTPUT); // THE CAN LIBRARY HAS THIS PIN FOR INTERRUPT FOR CAN1 (UNSUSED HERE) INPUT WITHOUT PULLUP, FORCE TO OUTPUT INSTEAD TO PREVENT ERRONEOUS INTERRUPTS.
  pinMode(FORCE_KEEP_ON, OUTPUT);
  digitalWrite(FORCE_KEEP_ON, HIGH);

  pinMode(BLUE_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
   
     
  if( ! CAN0.begin()) { 
    SerialBT.println("CAN0 Init Failed");  
  }
  
  SerialBT.println("===== V1 ================= \n");
  
  SerialBT.print("time");
  SerialBT.print(",fid");
  SerialBT.print(",time");
  SerialBT.print(",rtr");
  SerialBT.print(",pri");
  SerialBT.print(",ext");
  SerialBT.print(",len");
  SerialBT.print(",data");
  Serial.println(",b0,b1,b2,b3,b4,b5,b6,b7");
  
  CAN0.watchFor(MSG_ID_ZFE_Control_Module);
  CAN0.setCallback(0, callback);
}

void loop() {
  unsigned long now = millis();
  delay(150);
  digitalWrite(YELLOW_LED, LOW);

  if ( status_changed )
  {
      // print_status();
      status_changed = false;
  }

}

void callback(CAN_FRAME *message) {
  digitalWrite(YELLOW_LED, HIGH);
//  printFrame(*message);
  printFrameBT(*message);

  
}
//
//void callback(CAN_FRAME *message) {
//  digitalWrite(YELLOW_LED, HIGH);
//  
//
//
//
//
//// /* Print Functions */
//// void print_status()
//// {
//
////   String text = "Turn Signals: ";
////   if ( motorcycle_state.turn_signals == K25_Turn_Signals_State_off ) text += "OFF";
////   else if ( motorcycle_state.turn_signals == K25_Turn_Signals_State_left ) text += "LEFT";
////   else if ( motorcycle_state.turn_signals == K25_Turn_Signals_State_right ) text += "RIGHT";
////   else if ( motorcycle_state.turn_signals == K25_Turn_Signals_State_both ) text += "HAZARDS";
////   else text += "Unknown";
////   text += "    \n";
//
////   SerialBT.println(text);
//// }

void printFrameBT(CAN_FRAME &message) {
  SerialBT.print(millis());
  SerialBT.print(",");
  SerialBT.print(message.id,HEX);
  SerialBT.print(",fid:");
  SerialBT.print(message.fid);
  SerialBT.print(",time:");
  SerialBT.print(message.timestamp);
  SerialBT.print(",rtr:");
  SerialBT.print(message.rtr);
  SerialBT.print(",pri:");
  SerialBT.print(message.priority);
  SerialBT.print(",ext:");
  SerialBT.print(message.extended);
  SerialBT.print(",len:");
  SerialBT.print(message.length,DEC);
  SerialBT.print(",data:  ");
  
  for(int i = 0;i < message.length; i++) {
    SerialBT.print(",");
    if (message.data.uint8[i] <= 0xF ) {
      SerialBT.print("0");
    }
    SerialBT.print(message.data.uint8[i],HEX);
  }
  SerialBT.println();
}
