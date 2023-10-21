#include <Arduino.h>
#include <CANCommon.h>

CANSenderObject* sender;
uint64_t x = 98373;

void setup(){
Serial.begin(9600);
CAN0.startSetup();
sender = CAN0.createSender(2,2);
CAN0.finishSetup();


}


void loop(){

sender->setID(500);
sender->send_many(x);
Serial.println("sent");
delay(2000);

}