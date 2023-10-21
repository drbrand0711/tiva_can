#ifndef CAN_COMMON_CAN_OBJECT_H
#define CAN_COMMON_CAN_OBJECT_H

#include <Arduino.h>
#include <CANVars.h>
#include <can.h>

/**
 * Represents a CAN Message Object in the TIVA
 */
class CANObject
{
public:
    /**
     * A message object in the CAN controller
     */
    tCANMsgObject messageObject;

    /**
     * Number of the CAN message object
     */
    int objNum;

    /**
     * Whether this object is configured to be rx or tx
     */
    bool rx;

    /**
     * Message ID associated with this message object
     */
    int messageID;

    /**
     * Buffer used for storing the data in one CAN frame
     */
    uint8_t buffer[8];

    /**
     * Buffer used for storing the data of all 8 frames of CAN message
    */
    uint8_t* msg_buf[8];

    CANObject(int messageID, bool rx, int objNum)
    {
        this->messageObject = tCANMsgObject();
        this->messageID = messageID;
        this->rx = rx;
        this->objNum = objNum;
        
    }

    /**
     * Change the message ID of this CAN object
     *
     * @param newID ID to change to
     */
    void setID(int newID)
    {
        this->messageID = newID;
        this->messageObject.ui32MsgID = newID;
    }
};

/**
 * CAN Message Object which is configured for receiving messages with a
 * callback
 */
class CANReceiverObject : public CANObject
{
public:
    /**
     * Callback for when a CAN message is received
     */
    void (*callback)(int id, uint8_t buf[]) = nullptr;

    CANReceiverObject(int messageID, int objNum,
                      void (*callback)(int id, uint8_t buf[]))
        : CANObject(messageID, objNum, true)
    {
        this->messageObject.ui32MsgID = messageID;
        this->messageObject.ui32MsgIDMask = 0x1FFFFFFF;
        this->messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                        MSG_OBJ_RX_INT_ENABLE |
                                        MSG_OBJ_USE_EXT_FILTER;
        this->messageObject.pui8MsgData = this->buffer;
        this->callback = callback;
        CANMessageSet(CAN0_BASE, objNum, &this->messageObject, MSG_OBJ_TYPE_RX);

#if CAN_COMMON_DEBUG_SERIAL
        Serial.println("set can message object " + String(objNum));
#endif
    }

    CANReceiverObject(int messageID, int mask, int objNum,
                      void (*callback)(int id, uint8_t buf[]))
        : CANObject(messageID, objNum, true)
    {
        this->messageObject.ui32MsgID = messageID;
        this->messageObject.ui32MsgIDMask = mask;
        this->messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID |
                                        MSG_OBJ_RX_INT_ENABLE |
                                        MSG_OBJ_USE_EXT_FILTER | CAN_INT_MASTER;
        this->messageObject.pui8MsgData = this->buffer;
        this->callback = callback;
        CANMessageSet(CAN0_BASE, objNum, &this->messageObject, MSG_OBJ_TYPE_RX);

#if CAN_COMMON_DEBUG_SERIAL
        Serial.println("set can message object " + String(objNum));
#endif
    }
};

/**
 * CAN Message Object which is configured for transmitting messages
 */
class CANSenderObject : public CANObject
{
public:


    uint8_t no_of_frames;

    uint8_t which_frame;

    /**
     * Send a speific number of bytes with this sender's ID
     *
     * @param buf Array of bytes to send
     * @param bytes Number of bytes to send (should be <= 8)
     */
    int send(uint8_t* buf, int bytes)
    {
        //if (bytes > 8)
          //  return;  // TODO throw errors
        memset(this->buffer, 0, 16);
        memcpy(this->buffer, buf, bytes);
        CANMessageSet(CAN0_BASE, this->objNum, &this->messageObject,
                      MSG_OBJ_TYPE_TX);
        if(this->which_frame == this->no_of_frames){
            return -1;
        }
        this->which_frame = this->which_frame + 1;
        return 0;
    }

    template<typename T>
    void send_many(T value){
        uint8_t* data_ptr = (uint8_t*)(&value);
        uint8_t data_size = sizeof(T);
        this->no_of_frames = (data_size/6 + 1) ? (data_size%6) : (data_size/6);
        for(int i = 0;i < this->no_of_frames;i++)
        {
            this->msg_buf[i] = new uint8_t[16];
            this->msg_buf[i] = data_ptr + i;
        }
        
        this->which_frame = 0;
        this->send(this->msg_buf[which_frame] , 8);
    }
    /**
     * Send a value with this sender's ID
     *
     * @param value Value to send (size should be <= 8 bytes)
     */
    template <typename T>
    void send(T value)
    {
#if CAN_COMMON_DEBUG_SERIAL
        Serial.println("sending size: " + String(sizeof(value)));
#endif
        this->send((uint8_t*)&value, sizeof(value));
    }

    CANSenderObject(int messageID, int objNum)
        : CANObject(messageID, false , objNum)
    {
        this->messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
        this->messageObject.ui32MsgLen = 8u;
        this->messageObject.ui32MsgID = messageID;
        this->messageObject.pui8MsgData = this->buffer;
    }
};

#endif
