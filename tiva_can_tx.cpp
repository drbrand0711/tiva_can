#include <Arduino.h>
#include <CANCommon.h>

CANSenderObject* sender;
uint64_t x = 69;

void setup(){
Serial.begin(921600);
CAN0.startSetup();
sender = CAN0.createSender(2,2);
CAN0.finishSetup();


}


void loop(){

sender->setID(500);
sender->send(x);
Serial.println("sent");
delay(2000);

}