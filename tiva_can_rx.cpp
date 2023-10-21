#include <Arduino.h>
#include <driverlib/can.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <hw_can.h>

tCANMsgObject x;
uint8_t d[8];
int message = 590;


void handler(){
  unsigned long interruptCause = CANIntStatus(CAN0_BASE , CAN_INT_STS_CAUSE);
  if(interruptCause == CAN_INT_INTID_STATUS){
    unsigned long status = CANStatusGet(CAN0_BASE , CAN_STS_CONTROL);
    Serial.println(status);  
    Serial.println("Status Interrupt Received");
  }
  else{
    if(interruptCause >=1 && interruptCause <= 32)
    CANIntClear(CAN0_BASE , interruptCause);
    Serial.println(interruptCause);
    Serial.println("Receive interrupt Received");
  }
}

void setup(){
Serial.begin(9600);

SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)){}
GPIOPinTypeCAN(GPIO_PORTB_BASE , GPIO_PIN_4 | GPIO_PIN_5);
GPIOPinConfigure(GPIO_PB4_CAN0RX);
GPIOPinConfigure(GPIO_PB5_CAN0TX);
CANInit(CAN0_BASE);
CANBitRateSet(CAN0_BASE,SysCtlClockGet(),250000);
x.ui32MsgID = 0x01;
x.ui32MsgIDMask = 0xFF;
x.ui32Flags = MSG_OBJ_RX_INT_ENABLE | CAN_INT_MASTER;
x.pui8MsgData = d;
CANIntEnable(CAN0_BASE , CAN_INT_MASTER | CAN_INT_STATUS);
CANIntRegister(CAN0_BASE , handler);
CANMessageSet(CAN0_BASE,1 , &x ,MSG_OBJ_TYPE_RX);
CANEnable(CAN0_BASE);

IntMasterEnable();
}


void loop(){

}
