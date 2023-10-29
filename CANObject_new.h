#ifndef CAN_COMMON_CAN_OBJECT_H
#define CAN_COMMON_CAN_OBJECT_H

#define NO_OF_FRAMES_OFFSET 5
#define WHICH_FRAME_OFFSET 2

#include <Arduino.h>
#include <CANVars.h>
#include <can.h>

// Callbacks for the 32 CAN message objects
void (*obj_callback[32])(int , uint8_t*);

/**
 * Represents a Buffer where all the incoming frames get processed
*/
class Receive_Message_Buffer
{
public:
    /**
     * Message identifier of this message 
    */
    uint8_t msg_id;

    /**
     * Number of frames associated with this message
    */
    uint8_t no_of_frames;

    /**
     * Number of frames received for this message
    */
    uint8_t which_frame;

    /**
     * The random message identifier for this message
    */
    uint8_t rndm_msg_identifier;

    /**
     * The number of the CAN Message object associated with this message
    */
    uint8_t obj_num;

    /**
     * The data contained in the CAN Message
    */
    uint8_t *msg_buf;

    /**
     * The callback of the CAN Message object associated with this message
    */
    void (*callback)(int , uint8_t*);

    /**
     * Denotes whether receive_message_buffer object is empty to accept a new message
    */
    bool empty;

    /**
     * Initialise empty receive_message_buffer object
    */
    Receive_Message_Buffer(){
        this->empty = true;
    }



    /**
     * Get the total number of frames associated with this message
     * 
     * @param buf data of a single frame of this message
    */
    uint8_t get_no_of_frames(uint8_t *buf)
    {
        return (buf[0] & 0b11100000) >> NO_OF_FRAMES_OFFSET;
    }



    /**
     * Get the random message identifier of this message
     * 
     * @param buf data of a single frame of this message
    */
    uint8_t get_rndm_msg_id(uint8_t *buf)
    {
        return (buf[1] & 0b00111111);
    }



    /**
     * Get the frame number of this frame
     * 
     * @param buf data of a single frame of this message
    */
    uint8_t get_which_frame(uint8_t *buf)
    {
        return (buf[0] & 0b00011100) >> WHICH_FRAME_OFFSET;
    }



    /**
     * Set this receive_message_buffer object for a message
     * 
     * @param InterruptCause Relates to the priority of the CAN message object
     * @param CAN_msg_obj The first frame of this message
    */
    void set_msg_buffer_obj(uint8_t InterruptCause , tCANMsgObject CAN_msg_obj)
    {
        if(this->empty == false)
        {
            Serial.println("Non empty message buffer");
            return;
        }

        else
        {
#if CAN_COMMON_DEBUG_SERIAL
            Serial.println("New message set associated with CAN Message object " + String(InterruptCause - 1));
#endif
            this->empty = false;

            this->msg_id = CAN_msg_obj.ui32MsgID;

            this->rndm_msg_identifier = get_rndm_msg_id(CAN_msg_obj.pui8MsgData);

            this->no_of_frames = get_no_of_frames(CAN_msg_obj.pui8MsgData);

            this->which_frame = 0;

            this->obj_num = InterruptCause - 1;

            this->callback = obj_callback[this->obj_num];

            //Dynamically allocated, deleted upon calling  the callback
            this-> msg_buf = new uint8_t[6*(this->no_of_frames)];

            memset(this->msg_buf , 0 ,6*(this->no_of_frames) );
        }
    }



    /**
    * Entered upon receiving a frame
    * 
    * @param buf Data of the frame received
    */
    void frame_received(uint8_t *buf)
    {
        this->which_frame += 1;

        if(this->which_frame != get_which_frame(buf))
            Serial.println("Error in number of frame received gotten " + String(get_which_frame(buf)) + ",Expected " + String(this->which_frame));

        else if(this->rndm_msg_identifier != get_rndm_msg_id(buf))
            Serial.println("Error in random message identifier");

        else
        {
            memcpy(msg_buf + 6*(this->which_frame - 1), buf + 2, 6);
            if(this->which_frame == this->no_of_frames)
            {
                this->call_receiver_obj();
            }
        }
    }



    /**
     * Call the callback of the CAN Message Object
     *
    */
    void call_receiver_obj()
    {
        this->callback(this->msg_id , this->msg_buf);
        this->empty = true;
        //Deallocate memory to the msg_buf;
        delete this->msg_buf;
    }
};


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
    uint8_t buffer[8] ;

    /**
     * Buffer used for storing the data of all 8 frames of CAN message
    */
    uint8_t msg_buf[8][8];

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
        obj_callback[objNum - 1] = callback;
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
        obj_callback[objNum - 1] = callback;
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

    bool msg_completed;
  
    uint8_t rndm_msg_identifier;

    void set_msg_buf(uint8_t* buf, int bytes)
    {       
	
	    this->rndm_msg_identifier = (this->rndm_msg_identifier + 1)%64;
        this->which_frame = 0;
        this->no_of_frames = int(ceil(bytes/6.0));
        Serial.println("no of frames is "+ String(this->no_of_frames));
        for(int i=0; i<8; i++)
            memset(this->msg_buf[i], 0, 8);

        for(int i =0; i < this->no_of_frames - 1; i++)
        {
	    msg_buf[i][0] = ((i+1)<<2) + ((this->no_of_frames)<<5) + ((1)<<1);
	    msg_buf[i][1] = this->rndm_msg_identifier;
        
        memcpy(this->msg_buf[i] + 2, buf+6*i, 6);
        }

        msg_buf[this->no_of_frames][0] = ((this->no_of_frames)<<2) +((this->no_of_frames)<<5) + ((1)<<1);
        msg_buf[this->no_of_frames][1] = this->rndm_msg_identifier;
        if (bytes % 6)
            memcpy(msg_buf[this->no_of_frames] , buf + 6*(this->no_of_frames), bytes%6);
        else
            memcpy(msg_buf[this->no_of_frames] , buf + 6*(this->no_of_frames), 6);

    }


    void __send(uint8_t* buf, int bytes)
    {
        if (bytes > 8)
            return;  // TODO throw errors

        memset(this->buffer, 0, 8);
        memcpy(this->buffer, buf, bytes);
        Serial.println("Message set ");
        CANMessageSet(CAN0_BASE, this->objNum, &this->messageObject,
                      MSG_OBJ_TYPE_TX);
    }


    void _send()
    {
        if (!this->are_all_frames_sent())
        {
            __send(this->msg_buf[this->which_frame], 8);
            this->which_frame++;
        }
        else
            this->msg_completed = true;
    }

    int are_all_frames_sent()
    {
        Serial.println("Which frame is " + String(this->which_frame));
        if (this->which_frame >= this->no_of_frames)
            this->msg_completed = true;
        else
            this->msg_completed = false;
        return (this->which_frame >= this->no_of_frames);
    }

    /**
     * Send a speific number of bytes with this sender's ID
     *
     * @param buf Array of bytes to send
     * @param bytes Number of bytes to send (should be <= 8)
     */
    int send(uint8_t* buf, int bytes)
    {

        if (this->msg_completed != true){
#if CAN_COMMON_DEBUG_SERIAL
            Serial.println("Wait while current message gets transmitted");
#endif
            return -1;
        }

        this->msg_completed = false;
        this->set_msg_buf(buf , bytes);
        this->_send();

        return 0;
    }

    /**
     * Send a value with this sender's ID
     *
     * @param value Value to send (size should be <= 8 bytes)
     */
    template <typename T>
    int send(T value)
    {
#if CAN_COMMON_DEBUG_SERIAL
        Serial.println("sending size: " + String(sizeof(value)));
#endif
        return this->send((uint8_t*)&value, sizeof(value));
    }

    CANSenderObject(int messageID, int objNum)
        : CANObject(messageID, false , objNum)
    {
        this->messageObject.ui32Flags = MSG_OBJ_EXTENDED_ID | MSG_OBJ_TX_INT_ENABLE | CAN_INT_MASTER;
        this->messageObject.ui32MsgLen = 8u;
        this->messageObject.ui32MsgID = messageID;
        this->messageObject.pui8MsgData = this->buffer;
        this->msg_completed = true;
        this->no_of_frames = 0;
        this->which_frame = 0;
	    this->rndm_msg_identifier = 0;
    }
};

#endif
