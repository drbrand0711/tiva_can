
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
#include <CANObject.h>
#include <CANVars.h>
#include <can.h>
#include <interrupt.h>
#include <pin_map.h>
#include <sysctl.h>
#include <vector>

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
    void createReceiver(int priority, int messageID,
                        void (*callback)(int id, uint8_t* buf))
    {
        // create a new CANObject, set up the tCANMsgObject, add it to the
        // array of objects, and update the Rx true array of objects
        auto obj = new CANReceiverObject(messageID, priority, callback);
        canObjects[priority - 1] = obj;
        canObjectsRx[priority - 1] = true;
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
    void createReceiver(int priority, int filter, int mask,
                        void (*callback)(int id, uint8_t* buf))
    {
        // create a new CANObject, set up the tCANMsgObject,add it to the
        // array of objects, and update the Rx true array of objects
        auto obj = new CANReceiverObject(filter, mask, priority, callback);
        canObjects[priority - 1] = obj;
        canObjectsRx[priority - 1] = true;
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
        auto obj = new CANSenderObject(messageID, priority);
        canObjects[priority - 1] = obj;
        canObjectsRx[priority - 1] = false;
        return obj;
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
