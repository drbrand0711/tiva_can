#include <driverlib/can.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <Arduino.h>
#include <driverlib/timer.h>
#include <hw_ints.h>
#include <driverlib/interrupt.h>

void handler(){
  Serial.println("Proper dD");
}

tCANMsgObject x;
uint8_t d[8] = {1,1,1,1,0,0,0,0};

void setup(){
  Serial.begin(9600);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)){
  }
 
  GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
  GPIOPinConfigure(GPIO_PB4_CAN0RX);
  GPIOPinConfigure(GPIO_PB5_CAN0TX);
  CANInit(CAN0_BASE);
  CANBitRateSet(CAN0_BASE,SysCtlClockGet(),1000000u);
  x.ui32MsgID = 0x01;
  x.ui32MsgIDMask = 0xFF;
  x.ui32Flags = MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
  x.pui8MsgData = d;
  CANIntEnable(CAN0_BASE , CAN_INT_MASTER | CAN_INT_STATUS);
  CANIntRegister(CAN0_BASE , handler);
  CANMessageSet(CAN0_BASE,1 , &x ,MSG_OBJ_TYPE_TX);
  CANEnable(CAN0_BASE);

  IntMasterEnable();

}
void loop(){
delay(750);

CANMessageSet(CAN0_BASE,1 , &x ,MSG_OBJ_TYPE_TX);

}