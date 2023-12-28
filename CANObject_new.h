
#ifndef CAN_COMMON_CAN_OBJECT_H
#define CAN_COMMON_CAN_OBJECT_H

#define NO_OF_FRAMES_OFFSET 5
#define WHICH_FRAME_OFFSET 2

#define SUCCESS 0
#define FAILURE -1

#include <Arduino.h>
#include <CANVars.h>
#include <can.h>
#include "timestamp.h"

//Timer associated with CAN Messages
extern time_keeper CAN_tmr;

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
    uint32_t msg_id;

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
    uint8_t msg_buf[48];

    /**
     * The callback of the CAN Message object associated with this message
    */
    void (*callback)(int , uint8_t*);

    /**
     * Denotes whether receive_message_buffer object is empty to accept a new message
    */
    bool empty;

    /**
     * Denotes whether frame is empty to accept a CAN message
    */
    bool empty_frame[8];


    /**
     * Initialise empty receive_message_buffer object
    */
    Receive_Message_Buffer()
    {
        this->empty = true;
        memset(this->empty_frame, 0, 8);
    }


    /**
     * Get the total number of frames associated with this message
     * 
     * @param buf data of a single frame of this message
    */
    uint8_t get_no_of_frames(uint8_t *buf)
    {
        return ((buf[0] & 0b11100000) >> NO_OF_FRAMES_OFFSET) + 1;
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


    bool is_message_buffer_available()
    {
        return this->empty;
    }

    bool is_msg_id_rndm_msg_identifier_matching(uint32_t msg_id, uint8_t rndm_msg_identifier)
    {
        if (msg_id != this->msg_id)
            return FAILURE;
        if (rndm_msg_identifier != this->rndm_msg_identifier)
            return FAILURE;

        return SUCCESS;
    }


    /**
     * Set this receive_message_buffer object for a message
     * 
     * @param InterruptCause Relates to the priority of the CAN message object
     * @param CAN_msg_obj The first frame of this message
    */
    int set_msg_buffer_obj(uint8_t InterruptCause , uint32_t msg_id, uint8_t* buf)
    {
        if(!this->is_message_buffer_available())
        {
            Serial.println("Non empty message buffer");
            return FAILURE;
        }

        else
        {
#if CAN_COMMON_DEBUG_SERIAL
            Serial.println("New message set associated with CAN Message object " + String(InterruptCause - 1));
#endif
            this->empty = false;
            this->msg_id = msg_id;
            this->rndm_msg_identifier = this->get_rndm_msg_id(buf);
#if CAN_COMMON_DEBUG_SERIAL
            Serial.println("Set msg with rndm msg id "+String(this->rndm_msg_identifier));
#endif            
            this->no_of_frames = this->get_no_of_frames(buf);
            this->obj_num = InterruptCause - 1;
            this->callback = obj_callback[this->obj_num];
            memset(this->msg_buf , 0, 48);
            memset(this->empty_frame, 0, 8);
        }
        return SUCCESS;
    }


    /**
    * Entered upon receiving a frame
    * 
    * @param buf Data of the frame received
    */
    int frame_received(uint32_t msg_id, uint8_t *buf)
    {

        if (this->empty)
        {
            Serial.println("Message not allocated");
            return FAILURE;
        }

        if(this->msg_id != msg_id)
        {
            Serial.println("Error in message ID");
            return FAILURE;
        }

        if(this->rndm_msg_identifier != this->get_rndm_msg_id(buf))
        {
            Serial.println("Error in random message identifier");
            return FAILURE;
        }

        if(this->empty_frame[this->get_which_frame(buf)] == 1)
        {    
            Serial.println("Error in number of frame received gotten");
            return FAILURE;
        }

        if(this->get_no_of_frames(buf) != this->no_of_frames)
        {
            Serial.println("Error in number of frames");
            return FAILURE;
        }

        else
        {
#if CAN_COMMON_DEBUG_SERIAL
            Serial.println("Frame Received");
#endif
            this->which_frame = this->get_which_frame(buf);  
            this->empty_frame[this->which_frame] = 1;
            memcpy(msg_buf + 6*this->which_frame, buf + 2, 6);

            for (int i = 0; i < this->no_of_frames; i++)
                if (this->empty_frame[i] == 0)
                    return SUCCESS; 

            this->call_receiver_obj();
            return SUCCESS;
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
        memset(this->empty_frame, 0, 8);
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


    CANObject()
    {

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

    CANReceiverObject()
        : CANObject()
    {
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

    uint32_t timestamp;

    /**
     * Buffer used for storing the data of all 8 frames of CAN message
    */
    uint8_t msg_buf[8][8];


    int set_msg_buf(uint8_t* buf, int bytes)
    {       
	
        
	    this->rndm_msg_identifier = (this->rndm_msg_identifier + 1) % 64;
        this->no_of_frames = int(ceil((bytes+4)/6.0));
        this->timestamp = CAN_tmr.get_time_of_day();
        CAN_tmr.print_IST_Time(this->timestamp);

        if (this->no_of_frames > 8)
            return FAILURE;

        this->which_frame = 0;
#if CAN_COMMON_DEBUG_SERIAL
        Serial.println("no of frames is "+ String(this->no_of_frames));
#endif    
        for(int i = 0; i < 8; i++)
            memset(this->msg_buf[i], 0, 8);

        this->msg_buf[0][0] = ((this->no_of_frames - 1) << NO_OF_FRAMES_OFFSET) | ( 0 << WHICH_FRAME_OFFSET) | (1 << 1) ;
	    this->msg_buf[0][1] = this->rndm_msg_identifier;

        memcpy(this->msg_buf[0]+2 ,(uint8_t *)&timestamp , 4);
        memcpy(this->msg_buf[0]+6 , buf  , 2);

        for(int i = 1; i < this->no_of_frames - 1; i++)
        {
	        msg_buf[i][0] = ((this->no_of_frames - 1) << NO_OF_FRAMES_OFFSET) | ( i << WHICH_FRAME_OFFSET) | (1 << 1) ;
	        msg_buf[i][1] = this->rndm_msg_identifier;
            if(i == 0)

            memcpy(this->msg_buf[i] + 2, buf-4+6*i, 6);
            
        }

        if(this->no_of_frames != 1){
            this->msg_buf[this->no_of_frames - 1][0] = ((this->no_of_frames - 1) << NO_OF_FRAMES_OFFSET) | ((this->no_of_frames - 1) << WHICH_FRAME_OFFSET) |  (1 << 1);
            this->msg_buf[this->no_of_frames - 1][1] = this->rndm_msg_identifier;

            if (bytes % 6)
                memcpy(this->msg_buf[this->no_of_frames - 1] +2, buf + 6*(this->no_of_frames - 1), bytes%6);
            else
                memcpy(this->msg_buf[this->no_of_frames - 1] +2, buf + 6*(this->no_of_frames - 1), 6);

            for(int i=0;i<8;i++)
                for(int j=0;j<8;j++){
                    Serial.print(this->msg_buf[i][j]);
                    Serial.print(" ");
                }
            Serial.println("");
            return SUCCESS;

        }
        
        for(int i=0;i<8;i++){
            Serial.print(this->msg_buf[0][i]);
            Serial.print(" ");
        }
        Serial.println("");
        Serial.println(this->msg_buf[0][0] + 256*this->msg_buf[0][1] + 256*256*this->msg_buf[0][2] + 256*256*256*this->msg_buf[0][3]);

        return SUCCESS;
    } 

    int __send(uint8_t* buf, int bytes)
    {
        if (bytes > 8)
            return FAILURE; 

        memset(this->buffer, 0, 8);
        memcpy(this->buffer, buf, bytes);
        CANMessageSet(CAN0_BASE, this->objNum, &this->messageObject,
                      MSG_OBJ_TYPE_TX);
        
        return SUCCESS;
    }


    int _send()
    {
        if (!this->are_all_frames_sent())
        {
            if ( __send(this->msg_buf[this->which_frame], 8) == FAILURE)
                return FAILURE;
            this->which_frame++;
        }
        else
            this->msg_completed = true;
        
        return SUCCESS;
    }

    int are_all_frames_sent()
    {
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

        if(bytes > 48 || bytes <= 0){
#if CAN_COMMON_DEBUG_SERIAL
            Serial.println("Invalid number of bytes to be sent");
#endif
            return FAILURE;
        }

        if (this->msg_completed != true){
#if CAN_COMMON_DEBUG_SERIAL
            Serial.println("Wait while current message gets transmitted");
#endif
            return FAILURE;
        }

        this->msg_completed = false;

        if (this->set_msg_buf(buf , bytes) == FAILURE) 
            return FAILURE;
        if (this->_send() == FAILURE)
            return FAILURE;

        return SUCCESS;
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

    CANSenderObject()
        : CANObject()
    {

    }
};



#endif
