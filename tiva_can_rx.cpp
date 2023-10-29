#include <Arduino.h>
#include <driverlib/can.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <hw_can.h>
#include "CANCommon.h"

tCANMsgObject x;
uint8_t d[8];
int message = 590;

void blink(){
  digitalWrite(PF_2 , HIGH);
  delay(100);
  digitalWrite(PF_2 , LOW);
  delay(100);
}

void handler(int id , uint8_t buf[]){
  blink();
  Serial.print(buf[0]);
  Serial.println("Hello");
  
}

void setup(){
Serial.begin(921600);

CAN0.startSetup();
CAN0.createReceiver(2,500,handler);
CAN0.finishSetup();

}


void loop(){

}