
// TODO have a separate .cpp file for function definitions

// TODO serial

// TODO EEPROM

// TODO redo how LEDs blink
#ifndef CAN_COMMON_H
#define CAN_COMMON_H

#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include <Arduino.h>
#include <CANConfig.h>
#include <CANVars.h>
#include <can.h>
#include <interrupt.h>
#include <pin_map.h>
#include <sysctl.h>
#include <vector>
#include <CANObjDef.h>

extern CANSenderObject CAN_Sender_Objects[32];

extern CANReceiverObject CAN_Receiver_Objects[32];

//Timer defined in timestamp.h
extern time_keeper CAN_tmr;

/**
 * callback to call the timer callback
 */
void tmr_callback(int,uint8_t*);


/**
 * Class containing common functions for interacting with a CAN peripheral
 */
class CANCommon
{
private:
    /**
     * Each CAN peripheral should have only one class associated with it.
     * This variable is accessed in the callbacks for that peripheral
     */
    static CANCommon* baseCommunicator;

    /**
     * List of TIVA CAN objects
     */
    CANObject* canObjects[32];
    
    /**
     * List of Rx true or false for CAN objects
     */
    bool canObjectsRx[32];

    /**
     * List of CAN Receive Message Buffers
     */
    Receive_Message_Buffer canReceiveBufs[10];

    /**
     * Whether the red LED PIN should not be touched.
     */
    bool redLedDisable = false;

    /**
     * Callback used internally as an interrupt handler to call the right
     * callbacks
     */
    void canCallback()
    {
        unsigned long interruptCause =
            CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

        CANReceiverObject* rObj = nullptr;
        CANSenderObject* tObj = nullptr;

#if CAN_COMMON_DEBUG_SERIAL
        Serial.println("interrupt: " + String(interruptCause));
#endif

        /**
         * Clear the interrupt
        */
        CANIntClear(CAN0_BASE , interruptCause);

        if (interruptCause == CAN_INT_INTID_STATUS)
        {
            unsigned long sts = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
#if CAN_COMMON_DEBUG_SERIAL
            Serial.println("status: " + String(status));
#endif
            if (sts & CAN_STATUS_TXOK)
            {
#if CAN_COMMON_DEBUG_SERIAL
                Serial.println("TXOK");
#endif

                // turn blank
                // digitalWrite(PF_1, LOW);
                // digitalWrite(PF_2, LOW);
                // digitalWrite(PF_3, LOW);
            }

            if (sts & CAN_STATUS_RXOK)
            {
#if CAN_COMMON_DEBUG_SERIAL
                Serial.println("RXOK");
#endif

                // turn blank
                // digitalWrite(PF_1, LOW);
                // digitalWrite(PF_2, LOW);
                // digitalWrite(PF_3, LOW);
            }

            // handle error states
            if (sts & CAN_STATUS_BUS_OFF)
            {
#if CAN_COMMON_DEBUG_SERIAL
                Serial.println("BUS_OFF");
#endif
                // bus off state, we're dead
                this->status = BUS_OFF;

                if (resetOnCANError)
                {
                    SysCtlReset();
                }
            }
            else if (sts & CAN_STATUS_EPASS)
            {
#if CAN_COMMON_DEBUG_SERIAL
                Serial.println("PASSIVE_ERROR");
#endif
                // we're in passive state, can't TX
                this->status = PASSIVE_ERROR;
                if (resetOnCANError)
                {
                    SysCtlReset();
                }
            }
            else if (sts & CAN_STATUS_LEC_ACK)
            {
#if CAN_COMMON_DEBUG_SERIAL
                Serial.println("LACKED ACK");
#endif
                // lacked an ACK for our last TX

                // blink purple
                // digitalWrite(PF_1, HIGH);
                // digitalWrite(PF_2, HIGH);
                // digitalWrite(PF_3, LOW);
            }
            else
            {
#if CAN_COMMON_DEBUG_SERIAL
                Serial.println("OTHER STATUS INTERRUPT");
#endif
                // TODO add more status checks

                // all CAN functions are OK, so change our status
                if (this->status == BUS_OFF || this->status == PASSIVE_ERROR)
                {
                    this->status = OK;
                }
                
            }

            showStatusLED();
        }

        else if (interruptCause >= 1 && interruptCause <= 32)
        {

            /**
             * Receive interrupt
            */
            if(canObjectsRx[interruptCause - 1])
            {

#if CAN_COMMON_DEBUG_SERIAL
                Serial.println("RX INTERRUPT");
#endif

                bool message_present = false;

                rObj = (CANReceiverObject*)canObjects[interruptCause-1];

                CANMessageGet(CAN0_BASE , interruptCause , &(rObj->messageObject),0);

                if(!(rObj->messageObject.ui32MsgID == CAN_TIME_SYNCHRONIZATION_MESSAGE_ID)){
                    /**
                     * Going through all receive message buffer objects to check whether this message has already been set
                    */
                    for(int i =0;i<10;i++){
                        if(!canReceiveBufs[i].empty){
                            if(canReceiveBufs[i].obj_num == interruptCause-1)
                            {
                                canReceiveBufs[i].frame_received(rObj->messageObject.ui32MsgID , rObj->messageObject.pui8MsgData);
                                message_present = true;
                                break;
                            }
                        }
                    }

                    /**
                    * Going through all receive message buffer objects to find an empty one to set
                    */
                    if(!message_present)
                    {
                        for(int i=0;i<10;i++){
                            if(canReceiveBufs[i].empty)
                            {
                                canReceiveBufs[i].set_msg_buffer_obj(interruptCause ,rObj->messageObject.ui32MsgID, rObj->messageObject.pui8MsgData);
                                canReceiveBufs[i].frame_received(rObj->messageObject.ui32MsgID , rObj->messageObject.pui8MsgData);
                                break;
                            }
                        }
                    }
                
                }

                else{
                    obj_callback[interruptCause - 1](rObj->messageObject.ui32MsgID , rObj->messageObject.pui8MsgData);
                }
            }

            /**
             * Transmit interrupt
            */
            else if(!canObjectsRx[interruptCause - 1])
            {
#if CAN_COMMON_DEBUG_SERIAL
                Serial.println("TX INTERRUPT");
#endif          

                tObj = (CANSenderObject*)(canObjects[interruptCause - 1]);
                /**
                 * Frame pending transmission
                */
                if(!tObj->are_all_frames_sent()){
                    tObj->_send();                    
                }
                else{
#if CAN_COMMON_DEBUG_SERIAL
                    /**
                     * Transmission of message completed
                    */
                    Serial.println("All frames sent");
#endif
                }
            }
            
        }

        else
        {
#if CAN_COMMON_DEBUG_SERIAL
            Serial.println("interrupt cause: " + String(interruptCause));
#endif
        }
        
    }

    /**
     * Call the baseCommunicator's canCallback. Used as a CAN interrupt
     * callback.
     */
    static void baseCanCallback() { baseCommunicator->canCallback(); }

    /**
     * Do a fancy light show when we boot up
     */
    void showBootupSequence()
    {
        // // rainbow
        // int sequence[] = {0b001, 0b011, 0b010, 0b110, 0b100, 0b101, 0};

        // for (int i = 0; i < 7; i++)
        // {
        //     int s = sequence[i];

        //     digitalWrite(PF_1, s & 1);
        //     digitalWrite(PF_2, (s >> 1) & 1);
        //     digitalWrite(PF_3, (s >> 2) & 1);

        //     delay(100);
        // }

        // fade-blink cyan
        for (int i = 0; i < 50; i++)
        {
            analogWrite(PF_2, i);
            analogWrite(PF_3, i);
            delay(5);
        }

        for (int i = 50; i >= 0; i--)
        {
            analogWrite(PF_2, i);
            analogWrite(PF_3, i);
            delay(5);
        }
    }

    /**
     * Change the LED color based on the current status
     */
    void showStatusLED()
    {
        switch (this->status)
        {
        case OK:  // green
            if (!redLedDisable)
                digitalWrite(PF_1, LOW);
            digitalWrite(PF_2, LOW);
            digitalWrite(PF_3, HIGH);
            break;

        case HEARTBEAT_FAIL:  // yellow
            if (!redLedDisable)
                digitalWrite(PF_1, HIGH);
            digitalWrite(PF_2, LOW);
            digitalWrite(PF_3, HIGH);
            break;

        case PASSIVE_ERROR:  // blue
            if (!redLedDisable)
                digitalWrite(PF_1, LOW);
            digitalWrite(PF_2, HIGH);
            digitalWrite(PF_3, LOW);
            break;

        case BUS_OFF:  // red
            if (!redLedDisable)
                digitalWrite(PF_1, HIGH);
            digitalWrite(PF_2, LOW);
            digitalWrite(PF_3, LOW);
            break;

        case PERIPHERAL_ERROR:  // white
            if (!redLedDisable)
                digitalWrite(PF_1, HIGH);
            digitalWrite(PF_2, HIGH);
            digitalWrite(PF_3, HIGH);
            break;
        }
    }

public:
    /**
     * Current status of the microcontroller
     */
    CANStatus status = BUS_OFF;

    /**
     * Whether we should reset on a CAN error
     */
    bool resetOnCANError = false;

    /**
     * Start CAN setup by registering GPIO pins
     *
     * @param port Port of GPIO pins to use for CAN, either PORTB or PORTE
     * @param bitRate Bitrate in kbps for CAN
     * @param redLedDisable Whether the PF1 should be untouched by the library
     */
    void startSetup(int port = GPIO_PORTB_BASE, int bitRate = 250000,
                    bool redLedDisable = false)
    {

        baseCommunicator = this;
        this->redLedDisable = redLedDisable;

    
        // Set up CAN
        SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
        while (!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)) {}

        switch (port)
        {
        case GPIO_PORTB_BASE:
            GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
            GPIOPinConfigure(GPIO_PB4_CAN0RX);
            GPIOPinConfigure(GPIO_PB5_CAN0TX);
            break;

        case GPIO_PORTE_BASE:
            GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
            GPIOPinConfigure(GPIO_PE4_CAN0RX);
            GPIOPinConfigure(GPIO_PE5_CAN0TX);
            break;

        default:
            // TODO throw error here
            return;
        }

        CANInit(CAN0_BASE);  // Initialises CAN Controller afer reset
        CANBitRateSet(CAN0_BASE, SysCtlClockGet(), bitRate);

        CANIntRegister(CAN0_BASE, baseCanCallback);
        CANIntEnable(CAN0_BASE,
                     CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
        CANEnable(CAN0_BASE);  // Enables CAN
        

        IntMasterEnable();

        // Set LEDs to output
        if (!redLedDisable)
            pinMode(PF_1, OUTPUT);
        pinMode(PF_2, OUTPUT);
        pinMode(PF_3, OUTPUT);

        /**
         * Start the timer used for time synchronization purposes
         */
        CAN_tmr.timer_start();

        // This creates a CAN Receiver object for time synchronization CAN message
        this->createReceiver(1 , CAN_TIME_SYNCHRONIZATION_MESSAGE_ID , tmr_callback);
    
    }

    /**
     * Finish CAN setup - enable interrupts and show bootup LEDs
     */
    void finishSetup()
    {
        showBootupSequence();
        IntEnable(INT_CAN0);
        this->status = OK;
        showStatusLED();
    }

    /**
     * Change the status indicated by the LED on the microcontroller
     *
     * @param status New status of the microcontroller
     */
    void updateStatus(CANStatus status)
    {
        this->status = status;
        showStatusLED();
    }

    /**
     * Register a callback for all messages received with a particular CAN ID
     *
     * @param priority Priority of the CAN Message Object
     * @param messageID ID of the message
     * @param callback Callback for messages received
     */
    int createReceiver(int priority, int messageID,
                        void (*callback)(int id, uint8_t* buf))
    {
        switch(priority){
            case 1:
                #ifdef CAN_SEND_1
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_1
                CAN_R1.messageID = messageID;
                CAN_R1.messageObject = tCANMsgObject();
                CAN_R1.rx = true;
                CAN_R1.objNum = priority;
                CAN_R1.messageObject.ui32MsgID = messageID;
                CAN_R1.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R1.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R1.messageObject.pui8MsgData = CAN_R1.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R1.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R1;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif
            break;
            case 2:
                #ifdef CAN_SEND_2
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_2
                CAN_R2.messageID = messageID;
                CAN_R2.messageObject = tCANMsgObject();
                CAN_R2.rx = true;
                CAN_R2.objNum = priority;
                CAN_R2.messageObject.ui32MsgID = messageID;
                CAN_R2.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R2.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R2.messageObject.pui8MsgData = CAN_R2.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R2.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R2;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif
            break;
            case 3:
                #ifdef CAN_SEND_3
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_3
                CAN_R3.messageID = messageID;
                CAN_R3.messageObject = tCANMsgObject();
                CAN_R3.rx = true;
                CAN_R3.objNum = priority;
                CAN_R3.messageObject.ui32MsgID = messageID;
                CAN_R3.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R3.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R3.messageObject.pui8MsgData = CAN_R3.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R3.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R3;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif
            break;
            case 4:
                #ifdef CAN_SEND_4
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_4
                CAN_R4.messageID = messageID;
                CAN_R4.messageObject = tCANMsgObject();
                CAN_R4.rx = true;
                CAN_R4.objNum = priority;
                CAN_R4.messageObject.ui32MsgID = messageID;
                CAN_R4.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R4.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R4.messageObject.pui8MsgData = CAN_R4.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R4.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R4;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif
            break;
            case 5:
                #ifdef CAN_SEND_5
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_5
                CAN_R5.messageID = messageID;
                CAN_R5.messageObject = tCANMsgObject();
                CAN_R5.rx = true;
                CAN_R5.objNum = priority;
                CAN_R5.messageObject.ui32MsgID = messageID;
                CAN_R5.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R5.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R5.messageObject.pui8MsgData = CAN_R5.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R5.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R5;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 6:
                #ifdef CAN_SEND_6
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_6
                CAN_R6.messageID = messageID;
                CAN_R6.messageObject = tCANMsgObject();
                CAN_R6.rx = true;
                CAN_R6.objNum = priority;
                CAN_R6.messageObject.ui32MsgID = messageID;
                CAN_R6.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R6.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R6.messageObject.pui8MsgData = CAN_R6.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R6.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R6;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 7:
                #ifdef CAN_SEND_7
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_7
                CAN_R7.messageID = messageID;
                CAN_R7.messageObject = tCANMsgObject();
                CAN_R7.rx = true;
                CAN_R7.objNum = priority;
                CAN_R7.messageObject.ui32MsgID = messageID;
                CAN_R7.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R7.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R7.messageObject.pui8MsgData = CAN_R7.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R7.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R7;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 8:
                #ifdef CAN_SEND_8
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_8
                CAN_R8.messageID = messageID;
                CAN_R8.messageObject = tCANMsgObject();
                CAN_R8.rx = true;
                CAN_R8.objNum = priority;
                CAN_R8.messageObject.ui32MsgID = messageID;
                CAN_R8.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R8.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R8.messageObject.pui8MsgData = CAN_R8.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R8.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R8;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 9:
                #ifdef CAN_SEND_9
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_9
                CAN_R9.messageID = messageID;
                CAN_R9.messageObject = tCANMsgObject();
                CAN_R9.rx = true;
                CAN_R9.objNum = priority;
                CAN_R9.messageObject.ui32MsgID = messageID;
                CAN_R9.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R9.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R9.messageObject.pui8MsgData = CAN_R9.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R9.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R9;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 10:
                #ifdef CAN_SEND_10
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_10
                CAN_R10.messageID = messageID;
                CAN_R10.messageObject = tCANMsgObject();
                CAN_R10.rx = true;
                CAN_R10.objNum = priority;
                CAN_R10.messageObject.ui32MsgID = messageID;
                CAN_R10.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R10.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R10.messageObject.pui8MsgData = CAN_R10.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R10.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R10;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 11:
                #ifdef CAN_SEND_11
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_11
                CAN_R11.messageID = messageID;
                CAN_R11.messageObject = tCANMsgObject();
                CAN_R11.rx = true;
                CAN_R11.objNum = priority;
                CAN_R11.messageObject.ui32MsgID = messageID;
                CAN_R11.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R11.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R11.messageObject.pui8MsgData = CAN_R11.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R11.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R11;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 12:
                #ifdef CAN_SEND_12
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_12
                CAN_R12.messageID = messageID;
                CAN_R12.messageObject = tCANMsgObject();
                CAN_R12.rx = true;
                CAN_R12.objNum = priority;
                CAN_R12.messageObject.ui32MsgID = messageID;
                CAN_R12.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R12.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R12.messageObject.pui8MsgData = CAN_R12.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R12.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R12;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 13:
                #ifdef CAN_SEND_13
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_13
                CAN_R13.messageID = messageID;
                CAN_R13.messageObject = tCANMsgObject();
                CAN_R13.rx = true;
                CAN_R13.objNum = priority;
                CAN_R13.messageObject.ui32MsgID = messageID;
                CAN_R13.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R13.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R13.messageObject.pui8MsgData = CAN_R13.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R13.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R13;
                canObjectsRx[priority - 1] = true;
                #endif            
            break;
            case 14:
                #ifdef CAN_SEND_14
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_14
                CAN_R14.messageID = messageID;
                CAN_R14.messageObject = tCANMsgObject();
                CAN_R14.rx = true;
                CAN_R14.objNum = priority;
                CAN_R14.messageObject.ui32MsgID = messageID;
                CAN_R14.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R14.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R14.messageObject.pui8MsgData = CAN_R14.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R14.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R14;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 15:
                #ifdef CAN_SEND_15
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_15
                CAN_R15.messageID = messageID;
                CAN_R15.messageObject = tCANMsgObject();
                CAN_R15.rx = true;
                CAN_R15.objNum = priority;
                CAN_R15.messageObject.ui32MsgID = messageID;
                CAN_R15.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R15.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R15.messageObject.pui8MsgData = CAN_R15.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R15.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R15;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 16:
                #ifdef CAN_SEND_16
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_16
                CAN_R16.messageID = messageID;
                CAN_R16.messageObject = tCANMsgObject();
                CAN_R16.rx = true;
                CAN_R16.objNum = priority;
                CAN_R16.messageObject.ui32MsgID = messageID;
                CAN_R16.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R16.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R16.messageObject.pui8MsgData = CAN_R16.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R16.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R16;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 17:
                #ifdef CAN_SEND_17
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_17
                CAN_R17.messageID = messageID;
                CAN_R17.messageObject = tCANMsgObject();
                CAN_R17.rx = true;
                CAN_R17.objNum = priority;
                CAN_R17.messageObject.ui32MsgID = messageID;
                CAN_R17.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R17.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R17.messageObject.pui8MsgData = CAN_R17.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R17.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R17;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 18:
                #ifdef CAN_SEND_18
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_18
                CAN_R18.messageID = messageID;
                CAN_R18.messageObject = tCANMsgObject();
                CAN_R18.rx = true;
                CAN_R18.objNum = priority;
                CAN_R18.messageObject.ui32MsgID = messageID;
                CAN_R18.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R18.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R18.messageObject.pui8MsgData = CAN_R18.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R18.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R18;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 19:
                #ifdef CAN_SEND_19
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_19
                CAN_R19.messageID = messageID;
                CAN_R19.messageObject = tCANMsgObject();
                CAN_R19.rx = true;
                CAN_R19.objNum = priority;
                CAN_R19.messageObject.ui32MsgID = messageID;
                CAN_R19.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R19.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R19.messageObject.pui8MsgData = CAN_R19.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R19.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R19;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 20:
                #ifdef CAN_SEND_20
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_20
                CAN_R20.messageID = messageID;
                CAN_R20.messageObject = tCANMsgObject();
                CAN_R20.rx = true;
                CAN_R20.objNum = priority;
                CAN_R20.messageObject.ui32MsgID = messageID;
                CAN_R20.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R20.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R20.messageObject.pui8MsgData = CAN_R20.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R20.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R20;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 21:
                #ifdef CAN_SEND_21
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif


                #ifdef CAN_RECEIVE_21
                CAN_R21.messageID = messageID;
                CAN_R21.messageObject = tCANMsgObject();
                CAN_R21.rx = true;
                CAN_R21.objNum = priority;
                CAN_R21.messageObject.ui32MsgID = messageID;
                CAN_R21.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R21.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R21.messageObject.pui8MsgData = CAN_R21.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R21.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R21;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 22:
                #ifdef CAN_SEND_22
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_22
                CAN_R22.messageID = messageID;
                CAN_R22.messageObject = tCANMsgObject();
                CAN_R22.rx = true;
                CAN_R22.objNum = priority;
                CAN_R22.messageObject.ui32MsgID = messageID;
                CAN_R22.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R22.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R22.messageObject.pui8MsgData = CAN_R22.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R22.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R22;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 23:
                #ifdef CAN_SEND_23
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_23
                CAN_R23.messageID = messageID;
                CAN_R23.messageObject = tCANMsgObject();
                CAN_R23.rx = true;
                CAN_R23.objNum = priority;
                CAN_R23.messageObject.ui32MsgID = messageID;
                CAN_R23.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R23.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R23.messageObject.pui8MsgData = CAN_R23.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R23.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R23;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 24:
                #ifdef CAN_SEND_24
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_24
                CAN_R24.messageID = messageID;
                CAN_R24.messageObject = tCANMsgObject();
                CAN_R24.rx = true;
                CAN_R24.objNum = priority;
                CAN_R24.messageObject.ui32MsgID = messageID;
                CAN_R24.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R24.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R24.messageObject.pui8MsgData = CAN_R24.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R24.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R24;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 25:
                #ifdef CAN_SEND_25
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_25
                CAN_R25.messageID = messageID;
                CAN_R25.messageObject = tCANMsgObject();
                CAN_R25.rx = true;
                CAN_R25.objNum = priority;
                CAN_R25.messageObject.ui32MsgID = messageID;
                CAN_R25.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R25.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R25.messageObject.pui8MsgData = CAN_R25.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R25.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R25;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 26:
                #ifdef CAN_SEND_26
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_26
                CAN_R26.messageID = messageID;
                CAN_R26.messageObject = tCANMsgObject();
                CAN_R26.rx = true;
                CAN_R26.objNum = priority;
                CAN_R26.messageObject.ui32MsgID = messageID;
                CAN_R26.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R26.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R26.messageObject.pui8MsgData = CAN_R26.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R26.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R26;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 27:
                #ifdef CAN_SEND_27
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_27
                CAN_R27.messageID = messageID;
                CAN_R27.messageObject = tCANMsgObject();
                CAN_R27.rx = true;
                CAN_R27.objNum = priority;
                CAN_R27.messageObject.ui32MsgID = messageID;
                CAN_R27.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R27.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R27.messageObject.pui8MsgData = CAN_R27.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R27.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R27;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 28:
                #ifdef CAN_SEND_28
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_28
                CAN_R28.messageID = messageID;
                CAN_R28.messageObject = tCANMsgObject();
                CAN_R28.rx = true;
                CAN_R28.objNum = priority;
                CAN_R28.messageObject.ui32MsgID = messageID;
                CAN_R28.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R28.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R28.messageObject.pui8MsgData = CAN_R28.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R28.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R28;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 29:
                #ifdef CAN_SEND_29
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_29
                CAN_R29.messageID = messageID;
                CAN_R29.messageObject = tCANMsgObject();
                CAN_R29.rx = true;
                CAN_R29.objNum = priority;
                CAN_R29.messageObject.ui32MsgID = messageID;
                CAN_R29.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R29.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R29.messageObject.pui8MsgData = CAN_R29.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R29.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R29;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 30:
                #ifdef CAN_SEND_30
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_30
                CAN_R30.messageID = messageID;
                CAN_R30.messageObject = tCANMsgObject();
                CAN_R30.rx = true;
                CAN_R30.objNum = priority;
                CAN_R30.messageObject.ui32MsgID = messageID;
                CAN_R30.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R30.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R30.messageObject.pui8MsgData = CAN_R30.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R30.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R30;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 31:
                #ifdef CAN_SEND_31
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_31
                CAN_R31.messageID = messageID;
                CAN_R31.messageObject = tCANMsgObject();
                CAN_R31.rx = true;
                CAN_R31.objNum = priority;
                CAN_R31.messageObject.ui32MsgID = messageID;
                CAN_R31.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R31.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R31.messageObject.pui8MsgData = CAN_R31.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R31.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R31;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 32:
                #ifdef CAN_SEND_32
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_32
                CAN_R32.messageID = messageID;
                CAN_R32.messageObject = tCANMsgObject();
                CAN_R32.rx = true;
                CAN_R32.objNum = priority;
                CAN_R32.messageObject.ui32MsgID = messageID;
                CAN_R32.messageObject.ui32MsgIDMask = 0x1FFFFFFF;
                CAN_R32.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R32.messageObject.pui8MsgData = CAN_R32.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R32.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R32;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            default:
                Serial.println("Invalid Object number");
                return FAILURE;

        }
    }

    /**
     * Register a callback for all messages received, with IDs matched by a mask
     * and filter
     *
     * @param priority Priority of the CAN Message Object
     * @param filter Filter for message IDs
     * @param mask Filter mask
     * @param callback Function to be called when messages are received
     */
    int createReceiver(int priority, int filter, int mask,
                        void (*callback)(int id, uint8_t* buf))
    {
        switch(priority){
            case 1:
                #ifdef CAN_SEND_1
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_1
                CAN_R1.messageID = filter;
                CAN_R1.messageObject = tCANMsgObject();
                CAN_R1.rx = true;
                CAN_R1.objNum = priority;
                CAN_R1.messageObject.ui32MsgID = filter;
                CAN_R1.messageObject.ui32MsgIDMask = mask;
                CAN_R1.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R1.messageObject.pui8MsgData = CAN_R1.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R1.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R1;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif
            break;
            case 2:
                #ifdef CAN_SEND_2
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_2
                CAN_R2.messageID = filter;
                CAN_R2.messageObject = tCANMsgObject();
                CAN_R2.rx = true;
                CAN_R2.objNum = priority;
                CAN_R2.messageObject.ui32MsgID = filter;
                CAN_R2.messageObject.ui32MsgIDMask = mask;
                CAN_R2.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R2.messageObject.pui8MsgData = CAN_R2.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R2.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R2;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif
            break;
            case 3:
                #ifdef CAN_SEND_3
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_3
                CAN_R3.messageID = filter;
                CAN_R3.messageObject = tCANMsgObject();
                CAN_R3.rx = true;
                CAN_R3.objNum = priority;
                CAN_R3.messageObject.ui32MsgID = filter;
                CAN_R3.messageObject.ui32MsgIDMask = mask;
                CAN_R3.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R3.messageObject.pui8MsgData = CAN_R3.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R3.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R3;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif
            break;
            case 4:
                #ifdef CAN_SEND_4
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_4
                CAN_R4.messageID = filter;
                CAN_R4.messageObject = tCANMsgObject();
                CAN_R4.rx = true;
                CAN_R4.objNum = priority;
                CAN_R4.messageObject.ui32MsgID = filter;
                CAN_R4.messageObject.ui32MsgIDMask = mask;
                CAN_R4.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R4.messageObject.pui8MsgData = CAN_R4.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R4.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R4;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif
            break;
            case 5:
                #ifdef CAN_SEND_5
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_5
                CAN_R5.messageID = filter;
                CAN_R5.messageObject = tCANMsgObject();
                CAN_R5.rx = true;
                CAN_R5.objNum = priority;
                CAN_R5.messageObject.ui32MsgID = filter;
                CAN_R5.messageObject.ui32MsgIDMask = mask;
                CAN_R5.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R5.messageObject.pui8MsgData = CAN_R5.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R5.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R5;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 6:
                #ifdef CAN_SEND_6
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_6
                CAN_R6.messageID = filter;
                CAN_R6.messageObject = tCANMsgObject();
                CAN_R6.rx = true;
                CAN_R6.objNum = priority;
                CAN_R6.messageObject.ui32MsgID = filter;
                CAN_R6.messageObject.ui32MsgIDMask = mask;
                CAN_R6.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R6.messageObject.pui8MsgData = CAN_R6.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R6.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R6;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 7:
                #ifdef CAN_SEND_7
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_7
                CAN_R7.messageID = filter;
                CAN_R7.messageObject = tCANMsgObject();
                CAN_R7.rx = true;
                CAN_R7.objNum = priority;
                CAN_R7.messageObject.ui32MsgID = filter;
                CAN_R7.messageObject.ui32MsgIDMask = mask;
                CAN_R7.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R7.messageObject.pui8MsgData = CAN_R7.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R7.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R7;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 8:
                #ifdef CAN_SEND_8
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_8
                CAN_R8.messageID = filter;
                CAN_R8.messageObject = tCANMsgObject();
                CAN_R8.rx = true;
                CAN_R8.objNum = priority;
                CAN_R8.messageObject.ui32MsgID = filter;
                CAN_R8.messageObject.ui32MsgIDMask = mask;
                CAN_R8.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R8.messageObject.pui8MsgData = CAN_R8.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R8.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R8;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 9:
                #ifdef CAN_SEND_9
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_9
                CAN_R9.messageID = filter;
                CAN_R9.messageObject = tCANMsgObject();
                CAN_R9.rx = true;
                CAN_R9.objNum = priority;
                CAN_R9.messageObject.ui32MsgID = filter;
                CAN_R9.messageObject.ui32MsgIDMask = mask;
                CAN_R9.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R9.messageObject.pui8MsgData = CAN_R9.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R9.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R9;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 10:
                #ifdef CAN_SEND_10
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_10
                CAN_R10.messageID = filter;
                CAN_R10.messageObject = tCANMsgObject();
                CAN_R10.rx = true;
                CAN_R10.objNum = priority;
                CAN_R10.messageObject.ui32MsgID = filter;
                CAN_R10.messageObject.ui32MsgIDMask = mask;
                CAN_R10.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R10.messageObject.pui8MsgData = CAN_R10.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R10.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R10;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 11:
                #ifdef CAN_SEND_11
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_11
                CAN_R11.messageID = filter;
                CAN_R11.messageObject = tCANMsgObject();
                CAN_R11.rx = true;
                CAN_R11.objNum = priority;
                CAN_R11.messageObject.ui32MsgID = filter;
                CAN_R11.messageObject.ui32MsgIDMask = mask;
                CAN_R11.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R11.messageObject.pui8MsgData = CAN_R11.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R11.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R11;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 12:
                #ifdef CAN_SEND_12
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_12
                CAN_R12.messageID = filter;
                CAN_R12.messageObject = tCANMsgObject();
                CAN_R12.rx = true;
                CAN_R12.objNum = priority;
                CAN_R12.messageObject.ui32MsgID = filter;
                CAN_R12.messageObject.ui32MsgIDMask = mask;
                CAN_R12.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R12.messageObject.pui8MsgData = CAN_R12.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R12.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R12;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 13:
                #ifdef CAN_SEND_13
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_13
                CAN_R13.messageID = filter;
                CAN_R13.messageObject = tCANMsgObject();
                CAN_R13.rx = true;
                CAN_R13.objNum = priority;
                CAN_R13.messageObject.ui32MsgID = filter;
                CAN_R13.messageObject.ui32MsgIDMask = mask;
                CAN_R13.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R13.messageObject.pui8MsgData = CAN_R13.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R13.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R13;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 14:
                #ifdef CAN_SEND_14
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_14
                CAN_R14.messageID = filter;
                CAN_R14.messageObject = tCANMsgObject();
                CAN_R14.rx = true;
                CAN_R14.objNum = priority;
                CAN_R14.messageObject.ui32MsgID = filter;
                CAN_R14.messageObject.ui32MsgIDMask = mask;
                CAN_R14.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R14.messageObject.pui8MsgData = CAN_R14.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R14.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R14;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 15:
                #ifdef CAN_SEND_15
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_15
                CAN_R15.messageID = filter;
                CAN_R15.messageObject = tCANMsgObject();
                CAN_R15.rx = true;
                CAN_R15.objNum = priority;
                CAN_R15.messageObject.ui32MsgID = filter;
                CAN_R15.messageObject.ui32MsgIDMask = mask;
                CAN_R15.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R15.messageObject.pui8MsgData = CAN_R15.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R15.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R15;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 16:
                #ifdef CAN_SEND_16
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_16
                CAN_R16.messageID = filter;
                CAN_R16.messageObject = tCANMsgObject();
                CAN_R16.rx = true;
                CAN_R16.objNum = priority;
                CAN_R16.messageObject.ui32MsgID = filter;
                CAN_R16.messageObject.ui32MsgIDMask = mask;
                CAN_R16.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R16.messageObject.pui8MsgData = CAN_R16.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R16.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R16;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 17:
                #ifdef CAN_SEND_17
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_17
                CAN_R17.messageID = filter;
                CAN_R17.messageObject = tCANMsgObject();
                CAN_R17.rx = true;
                CAN_R17.objNum = priority;
                CAN_R17.messageObject.ui32MsgID = filter;
                CAN_R17.messageObject.ui32MsgIDMask = mask;
                CAN_R17.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R17.messageObject.pui8MsgData = CAN_R17.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R17.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R17;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 18:
                #ifdef CAN_SEND_18
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_18
                CAN_R18.messageID = filter;
                CAN_R18.messageObject = tCANMsgObject();
                CAN_R18.rx = true;
                CAN_R18.objNum = priority;
                CAN_R18.messageObject.ui32MsgID = filter;
                CAN_R18.messageObject.ui32MsgIDMask = mask;
                CAN_R18.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R18.messageObject.pui8MsgData = CAN_R18.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R18.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R18;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 19:
                #ifdef CAN_SEND_19
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_19
                CAN_R19.messageID = filter;
                CAN_R19.messageObject = tCANMsgObject();
                CAN_R19.rx = true;
                CAN_R19.objNum = priority;
                CAN_R19.messageObject.ui32MsgID = filter;
                CAN_R19.messageObject.ui32MsgIDMask = mask;
                CAN_R19.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R19.messageObject.pui8MsgData = CAN_R19.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R19.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R19;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 20:
                #ifdef CAN_SEND_20
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_20
                CAN_R20.messageID = filter;
                CAN_R20.messageObject = tCANMsgObject();
                CAN_R20.rx = true;
                CAN_R20.objNum = priority;
                CAN_R20.messageObject.ui32MsgID = filter;
                CAN_R20.messageObject.ui32MsgIDMask = mask;
                CAN_R20.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R20.messageObject.pui8MsgData = CAN_R20.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R20.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R20;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 21:
                #ifdef CAN_SEND_21
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_21
                CAN_R21.messageID = filter;
                CAN_R21.messageObject = tCANMsgObject();
                CAN_R21.rx = true;
                CAN_R21.objNum = priority;
                CAN_R21.messageObject.ui32MsgID = filter;
                CAN_R21.messageObject.ui32MsgIDMask = mask;
                CAN_R21.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R21.messageObject.pui8MsgData = CAN_R21.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R21.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R21;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 22:
                #ifdef CAN_SEND_22
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_22
                CAN_R22.messageID = filter;
                CAN_R22.messageObject = tCANMsgObject();
                CAN_R22.rx = true;
                CAN_R22.objNum = priority;
                CAN_R22.messageObject.ui32MsgID = filter;
                CAN_R22.messageObject.ui32MsgIDMask = mask;
                CAN_R22.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R22.messageObject.pui8MsgData = CAN_R22.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R22.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R22;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 23:
                #ifdef CAN_SEND_23
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_23
                CAN_R23.messageID = filter;
                CAN_R23.messageObject = tCANMsgObject();
                CAN_R23.rx = true;
                CAN_R23.objNum = priority;
                CAN_R23.messageObject.ui32MsgID = filter;
                CAN_R23.messageObject.ui32MsgIDMask = mask;
                CAN_R23.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R23.messageObject.pui8MsgData = CAN_R23.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R23.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R23;
                canObjectsRx[priority - 1] = true;           
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif
            break;
            case 24:
                #ifdef CAN_SEND_24
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_24
                CAN_R24.messageID = filter;
                CAN_R24.messageObject = tCANMsgObject();
                CAN_R24.rx = true;
                CAN_R24.objNum = priority;
                CAN_R24.messageObject.ui32MsgID = filter;
                CAN_R24.messageObject.ui32MsgIDMask = mask;
                CAN_R24.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R24.messageObject.pui8MsgData = CAN_R24.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R24.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R24;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 25:
                #ifdef CAN_SEND_25
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_25
                CAN_R25.messageID = filter;
                CAN_R25.messageObject = tCANMsgObject();
                CAN_R25.rx = true;
                CAN_R25.objNum = priority;
                CAN_R25.messageObject.ui32MsgID = filter;
                CAN_R25.messageObject.ui32MsgIDMask = mask;
                CAN_R25.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R25.messageObject.pui8MsgData = CAN_R25.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R25.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R25;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 26:
                #ifdef CAN_SEND_26
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_26
                CAN_R26.messageID = filter;
                CAN_R26.messageObject = tCANMsgObject();
                CAN_R26.rx = true;
                CAN_R26.objNum = priority;
                CAN_R26.messageObject.ui32MsgID = filter;
                CAN_R26.messageObject.ui32MsgIDMask = mask;
                CAN_R26.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R26.messageObject.pui8MsgData = CAN_R26.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R26.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R26;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 27:
                #ifdef CAN_SEND_27
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_27
                CAN_R27.messageID = filter;
                CAN_R27.messageObject = tCANMsgObject();
                CAN_R27.rx = true;
                CAN_R27.objNum = priority;
                CAN_R27.messageObject.ui32MsgID = filter;
                CAN_R27.messageObject.ui32MsgIDMask = mask;
                CAN_R27.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R27.messageObject.pui8MsgData = CAN_R27.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R27.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R27;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 28:
                #ifdef CAN_SEND_28
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_28
                CAN_R28.messageID = filter;
                CAN_R28.messageObject = tCANMsgObject();
                CAN_R28.rx = true;
                CAN_R28.objNum = priority;
                CAN_R28.messageObject.ui32MsgID = filter;
                CAN_R28.messageObject.ui32MsgIDMask = mask;
                CAN_R28.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R28.messageObject.pui8MsgData = CAN_R28.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R28.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R28;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 29:
                #ifdef CAN_SEND_29
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_29
                CAN_R29.messageID = filter;
                CAN_R29.messageObject = tCANMsgObject();
                CAN_R29.rx = true;
                CAN_R29.objNum = priority;
                CAN_R29.messageObject.ui32MsgID = filter;
                CAN_R29.messageObject.ui32MsgIDMask = mask;
                CAN_R29.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R29.messageObject.pui8MsgData = CAN_R29.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R29.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R29;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 30:
                #ifdef CAN_SEND_30
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_30
                CAN_R30.messageID = filter;
                CAN_R30.messageObject = tCANMsgObject();
                CAN_R30.rx = true;
                CAN_R30.objNum = priority;
                CAN_R30.messageObject.ui32MsgID = filter;
                CAN_R30.messageObject.ui32MsgIDMask = mask;
                CAN_R30.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R30.messageObject.pui8MsgData = CAN_R30.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R30.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R30;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 31:
                #ifdef CAN_SEND_31
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_31
                CAN_R31.messageID = filter;
                CAN_R31.messageObject = tCANMsgObject();
                CAN_R31.rx = true;
                CAN_R31.objNum = priority;
                CAN_R31.messageObject.ui32MsgID = filter;
                CAN_R31.messageObject.ui32MsgIDMask = mask;
                CAN_R31.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R31.messageObject.pui8MsgData = CAN_R31.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R31.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R31;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            case 32:
                #ifdef CAN_SEND_32
                Serial.println("Can't allocate same object for both sending and receving");
                return FAILURE;
                #endif

                #ifdef CAN_RECEIVE_32
                CAN_R32.messageID = filter;
                CAN_R32.messageObject = tCANMsgObject();
                CAN_R32.rx = true;
                CAN_R32.objNum = priority;
                CAN_R32.messageObject.ui32MsgID = filter;
                CAN_R32.messageObject.ui32MsgIDMask = mask;
                CAN_R32.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                                MSG_OBJ_RX_INT_ENABLE |
                                                MSG_OBJ_USE_EXT_FILTER;
                CAN_R32.messageObject.pui8MsgData = CAN_R32.buffer;
                obj_callback[priority - 1] = callback;
                CANMessageSet(CAN0_BASE, priority, &CAN_R32.messageObject, MSG_OBJ_TYPE_RX);

                canObjects[priority - 1] = &CAN_R32;
                canObjectsRx[priority - 1] = true;
                return SUCCESS;
                #else
                Serial.println("Define the correct Object");
                return FAILURE;
                #endif            
            break;
            default:
                Serial.println("Invalid object number");
                return FAILURE;


        }

    }

    /**
     * @param messageID ID to send messag     * Create a sender for a specific message ID
     *
     * @param priority Priority of the CAN Message Object
es in
     * @returns An object that you can use to send messages in the specified ID
     */
    CANSenderObject* createSender(int priority, int messageID)
    {

        switch(priority){
            case 1:
                #ifdef CAN_RECEIVE_1
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_1
                CAN_S1.messageID = messageID;
                CAN_S1.messageObject = tCANMsgObject();
                CAN_S1.rx = false;
                CAN_S1.objNum = priority;
                CAN_S1.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S1.messageObject.ui32MsgLen = 8u;
                CAN_S1.messageObject.ui32MsgID = messageID;
                CAN_S1.messageObject.pui8MsgData = CAN_S1.buffer;
                CAN_S1.msg_completed = true;
                CAN_S1.no_of_frames = 0;
                CAN_S1.which_frame = 0;
                CAN_S1.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S1;
                canObjectsRx[priority - 1] = false;
                return &CAN_S1; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 2:
                #ifdef CAN_RECEIVE_2
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_2
                CAN_S2.messageID = messageID;
                CAN_S2.messageObject = tCANMsgObject();
                CAN_S2.rx = false;
                CAN_S2.objNum = priority;
                CAN_S2.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S2.messageObject.ui32MsgLen = 8u;
                CAN_S2.messageObject.ui32MsgID = messageID;
                CAN_S2.messageObject.pui8MsgData = CAN_S2.buffer;
                CAN_S2.msg_completed = true;
                CAN_S2.no_of_frames = 0;
                CAN_S2.which_frame = 0;
                CAN_S2.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S2;
                canObjectsRx[priority - 1] = false;
                return &CAN_S2; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 3:
                #ifdef CAN_RECEIVE_3
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_3
                CAN_S3.messageID = messageID;
                CAN_S3.messageObject = tCANMsgObject();
                CAN_S3.rx = false;
                CAN_S3.objNum = priority;
                CAN_S3.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S3.messageObject.ui32MsgLen = 8u;
                CAN_S3.messageObject.ui32MsgID = messageID;
                CAN_S3.messageObject.pui8MsgData = CAN_S3.buffer;
                CAN_S3.msg_completed = true;
                CAN_S3.no_of_frames = 0;
                CAN_S3.which_frame = 0;
                CAN_S3.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S3;
                canObjectsRx[priority - 1] = false;
                return &CAN_S3; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 4:
                #ifdef CAN_RECEIVE_4
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_4
                CAN_S4.messageID = messageID;
                CAN_S4.messageObject = tCANMsgObject();
                CAN_S4.rx = false;
                CAN_S4.objNum = priority;
                CAN_S4.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S4.messageObject.ui32MsgLen = 8u;
                CAN_S4.messageObject.ui32MsgID = messageID;
                CAN_S4.messageObject.pui8MsgData = CAN_S4.buffer;
                CAN_S4.msg_completed = true;
                CAN_S4.no_of_frames = 0;
                CAN_S4.which_frame = 0;
                CAN_S4.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S4;
                canObjectsRx[priority - 1] = false;
                return &CAN_S4; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 5:
                #ifdef CAN_RECEIVE_5
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_5
                CAN_S5.messageID = messageID;
                CAN_S5.messageObject = tCANMsgObject();
                CAN_S5.rx = false;
                CAN_S5.objNum = priority;
                CAN_S5.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S5.messageObject.ui32MsgLen = 8u;
                CAN_S5.messageObject.ui32MsgID = messageID;
                CAN_S5.messageObject.pui8MsgData = CAN_S5.buffer;
                CAN_S5.msg_completed = true;
                CAN_S5.no_of_frames = 0;
                CAN_S5.which_frame = 0;
                CAN_S5.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S5;
                canObjectsRx[priority - 1] = false;
                return &CAN_S5; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 6:
                #ifdef CAN_RECEIVE_6
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_6
                CAN_S6.messageID = messageID;
                CAN_S6.messageObject = tCANMsgObject();
                CAN_S6.rx = false;
                CAN_S6.objNum = priority;
                CAN_S6.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S6.messageObject.ui32MsgLen = 8u;
                CAN_S6.messageObject.ui32MsgID = messageID;
                CAN_S6.messageObject.pui8MsgData = CAN_S6.buffer;
                CAN_S6.msg_completed = true;
                CAN_S6.no_of_frames = 0;
                CAN_S6.which_frame = 0;
                CAN_S6.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S6;
                canObjectsRx[priority - 1] = false;
                return &CAN_S6; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 7:
                #ifdef CAN_RECEIVE_7
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_7
                CAN_S7.messageID = messageID;
                CAN_S7.messageObject = tCANMsgObject();
                CAN_S7.rx = false;
                CAN_S7.objNum = priority;
                CAN_S7.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S7.messageObject.ui32MsgLen = 8u;
                CAN_S7.messageObject.ui32MsgID = messageID;
                CAN_S7.messageObject.pui8MsgData = CAN_S7.buffer;
                CAN_S7.msg_completed = true;
                CAN_S7.no_of_frames = 0;
                CAN_S7.which_frame = 0;
                CAN_S7.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S7;
                canObjectsRx[priority - 1] = false;
                return &CAN_S7; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 8:
                #ifdef CAN_RECEIVE_8
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_8
                CAN_S8.messageID = messageID;
                CAN_S8.messageObject = tCANMsgObject();
                CAN_S8.rx = false;
                CAN_S8.objNum = priority;
                CAN_S8.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S8.messageObject.ui32MsgLen = 8u;
                CAN_S8.messageObject.ui32MsgID = messageID;
                CAN_S8.messageObject.pui8MsgData = CAN_S8.buffer;
                CAN_S8.msg_completed = true;
                CAN_S8.no_of_frames = 0;
                CAN_S8.which_frame = 0;
                CAN_S8.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S8;
                canObjectsRx[priority - 1] = false;
                return &CAN_S8; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 9:
                #ifdef CAN_RECEIVE_9
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_9
                CAN_S9.messageID = messageID;
                CAN_S9.messageObject = tCANMsgObject();
                CAN_S9.rx = false;
                CAN_S9.objNum = priority;
                CAN_S9.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S9.messageObject.ui32MsgLen = 8u;
                CAN_S9.messageObject.ui32MsgID = messageID;
                CAN_S9.messageObject.pui8MsgData = CAN_S9.buffer;
                CAN_S9.msg_completed = true;
                CAN_S9.no_of_frames = 0;
                CAN_S9.which_frame = 0;
                CAN_S9.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S9;
                canObjectsRx[priority - 1] = false;
                return &CAN_S9; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 10:
                #ifdef CAN_RECEIVE_10
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_10
                CAN_S10.messageID = messageID;
                CAN_S10.messageObject = tCANMsgObject();
                CAN_S10.rx = false;
                CAN_S10.objNum = priority;
                CAN_S10.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S10.messageObject.ui32MsgLen = 8u;
                CAN_S10.messageObject.ui32MsgID = messageID;
                CAN_S10.messageObject.pui8MsgData = CAN_S10.buffer;
                CAN_S10.msg_completed = true;
                CAN_S10.no_of_frames = 0;
                CAN_S10.which_frame = 0;
                CAN_S10.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S10;
                canObjectsRx[priority - 1] = false;
                return &CAN_S10; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 11:
                #ifdef CAN_RECEIVE_11
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_11
                CAN_S11.messageID = messageID;
                CAN_S11.messageObject = tCANMsgObject();
                CAN_S11.rx = false;
                CAN_S11.objNum = priority;
                CAN_S11.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S11.messageObject.ui32MsgLen = 8u;
                CAN_S11.messageObject.ui32MsgID = messageID;
                CAN_S11.messageObject.pui8MsgData = CAN_S11.buffer;
                CAN_S11.msg_completed = true;
                CAN_S11.no_of_frames = 0;
                CAN_S11.which_frame = 0;
                CAN_S11.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S11;
                canObjectsRx[priority - 1] = false;
                return &CAN_S11;
                #else
                Serial.println("Object not defined");
                return nullptr; 
                #endif
            break;
            case 12:
                #ifdef CAN_RECEIVE_12
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_12
                CAN_S12.messageID = messageID;
                CAN_S12.messageObject = tCANMsgObject();
                CAN_S12.rx = false;
                CAN_S12.objNum = priority;
                CAN_S12.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S12.messageObject.ui32MsgLen = 8u;
                CAN_S12.messageObject.ui32MsgID = messageID;
                CAN_S12.messageObject.pui8MsgData = CAN_S12.buffer;
                CAN_S12.msg_completed = true;
                CAN_S12.no_of_frames = 0;
                CAN_S12.which_frame = 0;
                CAN_S12.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S12;
                canObjectsRx[priority - 1] = false;
                return &CAN_S12; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 13:
                #ifdef CAN_RECEIVE_13
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_13
                CAN_S13.messageID = messageID;
                CAN_S13.messageObject = tCANMsgObject();
                CAN_S13.rx = false;
                CAN_S13.objNum = priority;
                CAN_S13.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S13.messageObject.ui32MsgLen = 8u;
                CAN_S13.messageObject.ui32MsgID = messageID;
                CAN_S13.messageObject.pui8MsgData = CAN_S13.buffer;
                CAN_S13.msg_completed = true;
                CAN_S13.no_of_frames = 0;
                CAN_S13.which_frame = 0;
                CAN_S13.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S13;
                canObjectsRx[priority - 1] = false;
                return &CAN_S13; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 14:
                #ifdef CAN_RECEIVE_14
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_14
                CAN_S14.messageID = messageID;
                CAN_S14.messageObject = tCANMsgObject();
                CAN_S14.rx = false;
                CAN_S14.objNum = priority;
                CAN_S14.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S14.messageObject.ui32MsgLen = 8u;
                CAN_S14.messageObject.ui32MsgID = messageID;
                CAN_S14.messageObject.pui8MsgData = CAN_S14.buffer;
                CAN_S14.msg_completed = true;
                CAN_S14.no_of_frames = 0;
                CAN_S14.which_frame = 0;
                CAN_S14.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S14;
                canObjectsRx[priority - 1] = false;
                return &CAN_S14; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 15:
                #ifdef CAN_RECEIVE_15
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_15
                CAN_S15.messageID = messageID;
                CAN_S15.messageObject = tCANMsgObject();
                CAN_S15.rx = false;
                CAN_S15.objNum = priority;
                CAN_S15.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S15.messageObject.ui32MsgLen = 8u;
                CAN_S15.messageObject.ui32MsgID = messageID;
                CAN_S15.messageObject.pui8MsgData = CAN_S15.buffer;
                CAN_S15.msg_completed = true;
                CAN_S15.no_of_frames = 0;
                CAN_S15.which_frame = 0;
                CAN_S15.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S15;
                canObjectsRx[priority - 1] = false;
                return &CAN_S15; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 16:
                #ifdef CAN_RECEIVE_16
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_16
                CAN_S16.messageID = messageID;
                CAN_S16.messageObject = tCANMsgObject();
                CAN_S16.rx = false;
                CAN_S16.objNum = priority;
                CAN_S16.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S16.messageObject.ui32MsgLen = 8u;
                CAN_S16.messageObject.ui32MsgID = messageID;
                CAN_S16.messageObject.pui8MsgData = CAN_S16.buffer;
                CAN_S16.msg_completed = true;
                CAN_S16.no_of_frames = 0;
                CAN_S16.which_frame = 0;
                CAN_S16.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S16;
                canObjectsRx[priority - 1] = false;
                return &CAN_S16;
                #else
                Serial.println("Object not defined");
                return nullptr; 
                #endif
            break;
            case 17:
                #ifdef CAN_RECEIVE_17
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_17
                CAN_S17.messageID = messageID;
                CAN_S17.messageObject = tCANMsgObject();
                CAN_S17.rx = false;
                CAN_S17.objNum = priority;
                CAN_S17.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S17.messageObject.ui32MsgLen = 8u;
                CAN_S17.messageObject.ui32MsgID = messageID;
                CAN_S17.messageObject.pui8MsgData = CAN_S17.buffer;
                CAN_S17.msg_completed = true;
                CAN_S17.no_of_frames = 0;
                CAN_S17.which_frame = 0;
                CAN_S17.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S17;
                canObjectsRx[priority - 1] = false;
                return &CAN_S17; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 18:
                #ifdef CAN_RECEIVE_18
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_18
                CAN_S18.messageID = messageID;
                CAN_S18.messageObject = tCANMsgObject();
                CAN_S18.rx = false;
                CAN_S18.objNum = priority;
                CAN_S18.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S18.messageObject.ui32MsgLen = 8u;
                CAN_S18.messageObject.ui32MsgID = messageID;
                CAN_S18.messageObject.pui8MsgData = CAN_S18.buffer;
                CAN_S18.msg_completed = true;
                CAN_S18.no_of_frames = 0;
                CAN_S18.which_frame = 0;
                CAN_S18.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S18;
                canObjectsRx[priority - 1] = false;
                return &CAN_S18; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 19:
                #ifdef CAN_RECEIVE_19
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_19
                CAN_S19.messageID = messageID;
                CAN_S19.messageObject = tCANMsgObject();
                CAN_S19.rx = false;
                CAN_S19.objNum = priority;
                CAN_S19.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S19.messageObject.ui32MsgLen = 8u;
                CAN_S19.messageObject.ui32MsgID = messageID;
                CAN_S19.messageObject.pui8MsgData = CAN_S19.buffer;
                CAN_S19.msg_completed = true;
                CAN_S19.no_of_frames = 0;
                CAN_S19.which_frame = 0;
                CAN_S19.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S19;
                canObjectsRx[priority - 1] = false;
                return &CAN_S19; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 20:
                #ifdef CAN_RECEIVE_20
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_20
                CAN_S20.messageID = messageID;
                CAN_S20.messageObject = tCANMsgObject();
                CAN_S20.rx = false;
                CAN_S20.objNum = priority;
                CAN_S20.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S20.messageObject.ui32MsgLen = 8u;
                CAN_S20.messageObject.ui32MsgID = messageID;
                CAN_S20.messageObject.pui8MsgData = CAN_S20.buffer;
                CAN_S20.msg_completed = true;
                CAN_S20.no_of_frames = 0;
                CAN_S20.which_frame = 0;
                CAN_S20.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S20;
                canObjectsRx[priority - 1] = false;
                return &CAN_S20; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 21:
                #ifdef CAN_RECEIVE_21
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_21
                CAN_S21.messageID = messageID;
                CAN_S21.messageObject = tCANMsgObject();
                CAN_S21.rx = false;
                CAN_S21.objNum = priority;
                CAN_S21.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S21.messageObject.ui32MsgLen = 8u;
                CAN_S21.messageObject.ui32MsgID = messageID;
                CAN_S21.messageObject.pui8MsgData = CAN_S21.buffer;
                CAN_S21.msg_completed = true;
                CAN_S21.no_of_frames = 0;
                CAN_S21.which_frame = 0;
                CAN_S21.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S21;
                canObjectsRx[priority - 1] = false;
                return &CAN_S21;
                #else
                Serial.println("Object not defined");
                return nullptr; 
                #endif
            break;
            case 22:
                #ifdef CAN_RECEIVE_22
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_22
                CAN_S22.messageID = messageID;
                CAN_S22.messageObject = tCANMsgObject();
                CAN_S22.rx = false;
                CAN_S22.objNum = priority;
                CAN_S22.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S22.messageObject.ui32MsgLen = 8u;
                CAN_S22.messageObject.ui32MsgID = messageID;
                CAN_S22.messageObject.pui8MsgData = CAN_S22.buffer;
                CAN_S22.msg_completed = true;
                CAN_S22.no_of_frames = 0;
                CAN_S22.which_frame = 0;
                CAN_S22.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S22;
                canObjectsRx[priority - 1] = false;
                return &CAN_S22; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 23:
                #ifdef CAN_RECEIVE_23
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_23
                CAN_S23.messageID = messageID;
                CAN_S23.messageObject = tCANMsgObject();
                CAN_S23.rx = false;
                CAN_S23.objNum = priority;
                CAN_S23.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S23.messageObject.ui32MsgLen = 8u;
                CAN_S23.messageObject.ui32MsgID = messageID;
                CAN_S23.messageObject.pui8MsgData = CAN_S23.buffer;
                CAN_S23.msg_completed = true;
                CAN_S23.no_of_frames = 0;
                CAN_S23.which_frame = 0;
                CAN_S23.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S23;
                canObjectsRx[priority - 1] = false;
                return &CAN_S23; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 24:
                #ifdef CAN_RECEIVE_24
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_24
                CAN_S24.messageID = messageID;
                CAN_S24.messageObject = tCANMsgObject();
                CAN_S24.rx = false;
                CAN_S24.objNum = priority;
                CAN_S24.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S24.messageObject.ui32MsgLen = 8u;
                CAN_S24.messageObject.ui32MsgID = messageID;
                CAN_S24.messageObject.pui8MsgData = CAN_S24.buffer;
                CAN_S24.msg_completed = true;
                CAN_S24.no_of_frames = 0;
                CAN_S24.which_frame = 0;
                CAN_S24.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S24;
                canObjectsRx[priority - 1] = false;
                return &CAN_S24; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 25:
                #ifdef CAN_RECEIVE_25
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_25
                CAN_S25.messageID = messageID;
                CAN_S25.messageObject = tCANMsgObject();
                CAN_S25.rx = false;
                CAN_S25.objNum = priority;
                CAN_S25.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S25.messageObject.ui32MsgLen = 8u;
                CAN_S25.messageObject.ui32MsgID = messageID;
                CAN_S25.messageObject.pui8MsgData = CAN_S25.buffer;
                CAN_S25.msg_completed = true;
                CAN_S25.no_of_frames = 0;
                CAN_S25.which_frame = 0;
                CAN_S25.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S25;
                canObjectsRx[priority - 1] = false;
                return &CAN_S25; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 26:
                #ifdef CAN_RECEIVE_26
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_26
                CAN_S26.messageID = messageID;
                CAN_S26.messageObject = tCANMsgObject();
                CAN_S26.rx = false;
                CAN_S26.objNum = priority;
                CAN_S26.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S26.messageObject.ui32MsgLen = 8u;
                CAN_S26.messageObject.ui32MsgID = messageID;
                CAN_S26.messageObject.pui8MsgData = CAN_S26.buffer;
                CAN_S26.msg_completed = true;
                CAN_S26.no_of_frames = 0;
                CAN_S26.which_frame = 0;
                CAN_S26.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S26;
                canObjectsRx[priority - 1] = false;
                return &CAN_S26; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 27:
                #ifdef CAN_RECEIVE_27
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_27
                CAN_S27.messageID = messageID;
                CAN_S27.messageObject = tCANMsgObject();
                CAN_S27.rx = false;
                CAN_S27.objNum = priority;
                CAN_S27.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S27.messageObject.ui32MsgLen = 8u;
                CAN_S27.messageObject.ui32MsgID = messageID;
                CAN_S27.messageObject.pui8MsgData = CAN_S27.buffer;
                CAN_S27.msg_completed = true;
                CAN_S27.no_of_frames = 0;
                CAN_S27.which_frame = 0;
                CAN_S27.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S27;
                canObjectsRx[priority - 1] = false;
                return &CAN_S27; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 28:
                #ifdef CAN_RECEIVE_28
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_28
                CAN_S28.messageID = messageID;
                CAN_S28.messageObject = tCANMsgObject();
                CAN_S28.rx = false;
                CAN_S28.objNum = priority;
                CAN_S28.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S28.messageObject.ui32MsgLen = 8u;
                CAN_S28.messageObject.ui32MsgID = messageID;
                CAN_S28.messageObject.pui8MsgData = CAN_S28.buffer;
                CAN_S28.msg_completed = true;
                CAN_S28.no_of_frames = 0;
                CAN_S28.which_frame = 0;
                CAN_S28.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S28;
                canObjectsRx[priority - 1] = false;
                return &CAN_S28;
                #else
                Serial.println("Object not defined");
                return nullptr; 
                #endif
            break;
            case 29:
                #ifdef CAN_RECEIVE_29
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_29
                CAN_S29.messageID = messageID;
                CAN_S29.messageObject = tCANMsgObject();
                CAN_S29.rx = false;
                CAN_S29.objNum = priority;
                CAN_S29.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S29.messageObject.ui32MsgLen = 8u;
                CAN_S29.messageObject.ui32MsgID = messageID;
                CAN_S29.messageObject.pui8MsgData = CAN_S29.buffer;
                CAN_S29.msg_completed = true;
                CAN_S29.no_of_frames = 0;
                CAN_S29.which_frame = 0;
                CAN_S29.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S29;
                canObjectsRx[priority - 1] = false;
                return &CAN_S29; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 30:
                #ifdef CAN_RECEIVE_30
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_30
                CAN_S30.messageID = messageID;
                CAN_S30.messageObject = tCANMsgObject();
                CAN_S30.rx = false;
                CAN_S30.objNum = priority;
                CAN_S30.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S30.messageObject.ui32MsgLen = 8u;
                CAN_S30.messageObject.ui32MsgID = messageID;
                CAN_S30.messageObject.pui8MsgData = CAN_S30.buffer;
                CAN_S30.msg_completed = true;
                CAN_S30.no_of_frames = 0;
                CAN_S30.which_frame = 0;
                CAN_S30.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S30;
                canObjectsRx[priority - 1] = false;
                return &CAN_S30;
                #else
                Serial.println("Object not defined");
                return nullptr; 
                #endif
            break;
            case 31:
                #ifdef CAN_RECEIVE_31
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_31
                CAN_S31.messageID = messageID;
                CAN_S31.messageObject = tCANMsgObject();
                CAN_S31.rx = false;
                CAN_S31.objNum = priority;
                CAN_S31.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S31.messageObject.ui32MsgLen = 8u;
                CAN_S31.messageObject.ui32MsgID = messageID;
                CAN_S31.messageObject.pui8MsgData = CAN_S31.buffer;
                CAN_S31.msg_completed = true;
                CAN_S31.no_of_frames = 0;
                CAN_S31.which_frame = 0;
                CAN_S31.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S31;
                canObjectsRx[priority - 1] = false;
                return &CAN_S31; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            case 32:
                #ifdef CAN_RECEIVE_32
                Serial.println("Can't allocate same object for both sending and receving");
                return nullptr;
                #endif

                #ifdef CAN_SEND_32
                CAN_S32.messageID = messageID;
                CAN_S32.messageObject = tCANMsgObject();
                CAN_S32.rx = false;
                CAN_S32.objNum = priority;
                CAN_S32.messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
                CAN_S32.messageObject.ui32MsgLen = 8u;
                CAN_S32.messageObject.ui32MsgID = messageID;
                CAN_S32.messageObject.pui8MsgData = CAN_S32.buffer;
                CAN_S32.msg_completed = true;
                CAN_S32.no_of_frames = 0;
                CAN_S32.which_frame = 0;
                CAN_S32.rndm_msg_identifier = 0;    
                canObjects[priority - 1] = &CAN_S32;
                canObjectsRx[priority - 1] = false;
                return &CAN_S32; 
                #else
                Serial.println("Object not defined");
                return nullptr;
                #endif
            break;
            default:
                Serial.println("Invalid object number");
                return nullptr;
        }
        return nullptr;
        
    }

    CANCommon() 
    {
        for(int i =0;i<10;i++){
            canReceiveBufs[i] = Receive_Message_Buffer();
        }
    }
};

CANCommon* CANCommon::baseCommunicator = nullptr;
CANCommon CAN0 = CANCommon();

void tmr_callback(int id , uint8_t buf[])
{
    CAN_tmr.CAN_timer_callback(id , buf);
}
#endif
