#! /usr/bin/env python3

# This file interfaces between CAN messages and messages in other ROS topics.
# It takes setpoints/params and converts them to CAN messages, and reads
# received CAN messages and publishes them in feedback topics.

import time
import rospy
import math
from can_rpi.msg import CANmessage
from std_msgs.msg import UInt16, Int16, Int32, Float64, Char, UInt64
import struct


#stores pending and completed can messages , completed ones is overwritten
can_receive_buffers = []


#for publishing received can messages to ros topics
receive_message_objects = {}


#maps the message id of message in can_receiver_topic to the topic , datatype to publish to 
map_id_topic_datatype = {}

HOURS = 24
MINUTES = 60
SECONDS = 60
MILLISECONDS = 1000

NUMBER_OF_UNIQUE_MESSAGE_IDENTIFIERS = 64

NUMBER_OF_BYTES_PER_FRAME = 6

NUMBER_OF_FRAMES_OFFSET = 5
WHICH_FRAME_OFFSET = 2
TIMESTAMP_INCLUDED_OR_NOT_OFFSET = 1

MAX_NO_OF_FRAMES_PER_MESSAGE = 8


class HandleMessage:
    """
    General message handler base class
    """
    def __init__(self, topic, datatype):
        """
        Initialize the topic and datatype related parameters corresponding to the message
        """
        self.topic = topic
        self.datatype = datatype
        data_type = type(self.datatype().data)
        
        # based on the datatype, store some values to use when packing/unpacking the value
        if data_type == UInt16:
            self.size = 2
            self.struct_format = "H"
        elif data_type == Int16:
            self.size = 2
            self.struct_format = "h"
        elif data_type == Float64:
            self.size = 8
            self.struct_format = "d"
        elif data_type == Char:
            self.size = 1
            self.struct_format = "c"
        elif data_type == Int32:
            self.size = 4
            self.struct_format = "i"
        elif data_type == UInt64:
            self.size = 8
            self.struct_format = "Q"
        else:
            rospy.logerr("you forgot to implement " + str(datatype))


class Publisher_Details(HandleMessage):
    """
    Holds the details of publishing of the can message stored in receive_message_class object
    """

    def __init__(self,topic,datatype):
        """
        Initialize the topic and datatype wrt publishing
        """
        super().__init__(topic,datatype)
        try:
            self.publisher = rospy.Publisher(self.topic,self.datatype,queue_size = 1)
        except :
            rospy.logerr("Invalid topic:"+ self.topic+ " or datatype:"+ self.datatype)
    def unpack(self,data):
        """
        Unpacks the CANmessage data into datatype of publishing
        """
        return  struct.unpack(self.struct_format , bytes(data))[0]
        
    def publish(self ,timestamp , data):
        """
        Publishes data and timestamp to the topic
        """
        published_data = self.datatype()
        published_data.data.data = self.unpack(data[:self.size])
        published_data.timestamp.data = timestamp
        self.publisher.publish(published_data)


class receive_message_class():
    """
    Receive message class
    """

    #the topic and datatype that this object has to publish to
    def __init__(self):
        """
        Initialize an empty receive message object
        """
        self.available = True
        self.message_id = 0
        self.rndm_message_identifier = 0

        self.received_frames = 0
        self.number_of_frames = MAX_NO_OF_FRAMES_PER_MESSAGE
        self.incld_tmsp = 0

    #sets a new can message into the object
    def set_message(self, CANmessage_object):
        """
        Sets the receive message object to hold a new can message
        """

        if not self.available:
            rospy.logwarn(CANmessage_object.id +" , the message with this id could not be set") 

        self.available = False
        self.message_id = CANmessage_object.id
        self.received_frames = 0
        self.incld_tmsp = self.get_incld_timestamp(CANmessage_object.data)
        self.number_of_frames = self.get_no_of_frames(CANmessage_object.data)
        self.rndm_message_identifier = self.get_rndm_msg_id(CANmessage_object.data)
        self.frames = []
        for _ in range(0,self.number_of_frames):
            frame = {}
            frame['frame'] = []
            frame['valid'] = False
            self.frames.append(frame)
        self.store_frame(CANmessage_object.data)


    def get_no_of_frames(self, can_frame_data):
        """
        Gets No.of frames in the current can message
        """
        first_byte = can_frame_data[0]
        no_of_frames = (first_byte & 0xE0) >> NUMBER_OF_FRAMES_OFFSET
        return no_of_frames

    def get_which_frame(self, can_frame_data):
        """
        Gets the number of the current frame in the can message
        """
        first_byte = can_frame_data[0]
        which_frame = (first_byte & 0x1C) >> WHICH_FRAME_OFFSET
        return which_frame

    def get_incld_timestamp(self, can_frame_data):
        """
        Whether the can message has timestamp included in it or not
        """
        first_byte = can_frame_data[0]
        incl_timestamp = (first_byte & 0x02) >> TIMESTAMP_INCLUDED_OR_NOT_OFFSET
        return incl_timestamp

    def get_rndm_msg_id(self, can_frame_data):
        """
        Gets the random message identifier of a can message from the can frame
        """
        second_byte = can_frame_data[1]
        rndm_msg_id = (second_byte & 0x3F)
        return rndm_msg_id

    def store_frame(self , can_frame_data):
        """
        Stores data and timestamp if present from a can frame of a can message
        """
        if self.available:
            rospy.logerr("You are trying to store data in an already complete message")
            return

        can_frame_data = list(can_frame_data)
        which_frame = self.get_which_frame(can_frame_data)

        if which_frame > self.number_of_frames:
            rospy.logwarn("Invalid frame with frame no "+which_frame)
            return

        if self.frames[which_frame]['valid'] == True:
            rospy.logwarn("Duplicate frame with frame no "+which_frame)
            return
        
        self.frames[which_frame]['valid'] = True
        self.frames[which_frame]['frame'] = can_frame_data

        self.received_frames += 1

        if self.received_frames == self.number_of_frames:
            publish_message = True
            for each_frame in self.frames:
                if each_frame['valid'] == False:
                    publish_message = False
                    break
            if publish_message:
                self.publish_message()


    def publish_message(self):
        """
        Extracts data and timestamp from all the frames
        Followed by publishing a rosmessage
        """
        message_id = self.message_id

        data = []

        if self.incld_tmsp:
            timestamp = self.frames[0]['frame'][2:6]
            timestamp = self.frames[0]['frame']
            timestamp = (timestamp[0] << 24) | (timestamp[1] << 16) | (timestamp[2] << 8) | (timestamp[3])
            data += self.frames[0]['frame'][6:]
        else:
            timestamp = 0
            data += self.frames[0]['frame'][2:]

        for frame_number in range(1,self.number_of_frames):
            data += self.frames[frame_number]['frame'][2:]

        publish_ros_message(message_id, timestamp, data)


def publish_ros_message(message_id , timestamp,data):
    """
    Publishes a rosmessage from a given CANmessage and timestamp
    """
    receive_message_objects[message_id].publish(timestamp,data)


def receiver_callback(data):
    """
    Callback for when a can frame is received 
    """

    def get_rndm_message_identifier(data):
        """
        returns random message identifier.
        Present in second byte: byte[5:0] - LSB 6 bits
        """
        second_byte_in_frame = data.data[1]
        return 0x3F & second_byte_in_frame

    for message_object in can_receive_buffers:
        if message_object.available == False:
            if message_object.message_id == data.id and message_object.rndm_message_identifier == get_rndm_message_identifier(data) :
                message_object.store_frame(data.data)
                return
            
    for message_object in can_receive_buffers[::-1]:
        if message_object.available == True:
            message_object.set_message(data) 
            return

    #if not both the above make a new message object
    create_new_receive_msg_object = receive_message_class()
    can_receive_buffers.append(create_new_receive_msg_object)
    can_receive_buffers[-1].set_message(data) 


class SendMessage(HandleMessage):
    def __init__(
            self,
            message_id,
            max_value,
            min_value,
            default_value,
            subscriber_topic,
            incld_tmstmp,
            publisher,
            datatype
    ):
        """
        Initialize a new send message object
        """
        super().__init__(subscriber_topic, datatype)
        self.message_id = message_id
        self.max_value = max_value
        self.min_value = min_value
        self.default_value = default_value
        self.sub = rospy.Subscriber(self.topic, datatype, self.publish)
        self.pub = publisher
        self.incld_tmstmp = incld_tmstmp 

        self.rndm_msg_identifier = 0

    def pack(self, data):
        """
        Packs a ROS message into bytes
        """
        d = data.data

        if d > self.max_value:
            d = self.max_value
        elif d < self.min_value:
            d = self.min_value

        return list(struct.pack(self.struct_format, data.data))


    def modified_timestamp(self, timestamp):
        """
        Receives the timestamp and modifies it to a day's time so that it can be sent
        """
        return (timestamp % (HOURS * MINUTES * SECONDS * MILLISECONDS))

    def get_msg_identifier(self):
        """
        Increment the random message identifier and return it
        """
        self.rndm_msg_identifier = (self.rndm_msg_identifier + 1) % (NUMBER_OF_UNIQUE_MESSAGE_IDENTIFIERS)
        return self.rndm_msg_identifier

    def publish(self, data):
        """
        Send a ROS message over CAN
        """

        # get the data from the message
        actual_data = self.pack(data.data)

        # If time stamp is included, it must be sent with the data
        if self.incld_tmstmp:
            timestamp = data.timestamp.data
            timestamp = self.modified_timestamp(timestamp)
            timestamp = list(struct.pack("i", timestamp))
            actual_data = timestamp + actual_data

        #Find out the number of can frames
        len_data = len(actual_data)
        number_of_can_frames = math.ceil(len_data / NUMBER_OF_BYTES_PER_FRAME)

        if number_of_can_frames > MAX_NO_OF_FRAMES_PER_MESSAGE:
            rospy.logerr("Message with length "+len_data+" is too big" )
            return

        # Pad zeros
        if len_data % NUMBER_OF_BYTES_PER_FRAME != 0:
            actual_data.extend([0] * (NUMBER_OF_BYTES_PER_FRAME - len_data % NUMBER_OF_BYTES_PER_FRAME))

        #Get the rndm msg identifier of this message
        m_i = self.get_msg_identifier()

        for frame_no in range(0, number_of_can_frames):

            # first byte contains
            # [7:5] bits contains number of frames
            # [4:2] bits contain which frame
            # [1]   bit contains whether timestamp is included or not
            # [0]   Reserved bit: 0

            first_byte = 0
            first_byte += (1 << NUMBER_OF_FRAMES_OFFSET) * number_of_can_frames
            first_byte += (1 << WHICH_FRAME_OFFSET) * frame_no
            first_byte += (1 << TIMESTAMP_INCLUDED_OR_NOT_OFFSET) * self.incld_tmstmp

            # second byte contains
            # [7:6] Reserved bits : 0
            # [5:0] Message Unique number
            second_byte = m_i

            remaining_bytes = actual_data[frame_no * NUMBER_OF_BYTES_PER_FRAME: (frame_no+1) * NUMBER_OF_BYTES_PER_FRAME]

            # Creating the frame
            frame = []
            frame.append(first_byte)
            frame.append(second_byte)
            frame.extend(remaining_bytes)

            c = CANmessage()
            c.id = self.message_id
            c.data = frame
            self.pub.publish(c)
            time.sleep(0.002)



def create_receiver(name , datatype , kind):
    """
    Shortand for creating a ReceiveMessage object using YAML params
    This publishes to the Rostopic
    """
    receive_message_objects[rospy.get_param(f"{name}/can/{kind}_message_id")] = Publisher_Details(rospy.get_param(f"{name}/{kind}_topic"),datatype)



def create_sender(name, pub, datatype):
    """
    Shorthand for creating a SendMessage object using YAML params
    """
    return SendMessage(
        message_id=rospy.get_param(f"{name}/can/data_message_id"),
        max_value=rospy.get_param(f"{name}/max_value"),
        min_value=rospy.get_param(f"{name}/min_value"),
        default_value=rospy.get_param(f"{name}/default_value"),
        subscriber_topic=rospy.get_param(f"{name}/setpoint_topic"),
        # Include timestamp or not - get it from ROS param {name}/include_timestamp
        incld_tmstmp = rospy.get_param(f"{name}/incld_tmstmp"),
        publisher=pub,
        datatype=datatype
    )


if __name__ == "__main__":
    rospy.init_node("topic_joiner", anonymous=True)
    throttle_check = rospy.get_param("enable/throttle", False)
    brake_check = rospy.get_param("enable/brake", False)
    steering_check = rospy.get_param("enable/steering", False)
    encoder_check = rospy.get_param("enable/encoder", False)

    can_sender_publisher = rospy.Publisher(rospy.get_param("can/sender_topic"), CANmessage, queue_size=50)

    can_receiver_subscriber = rospy.Subscriber(
        rospy.get_param("can/receiver_topic"), CANmessage, receiver_callback
    )

    if throttle_check:
        create_sender("throttle", can_sender_publisher, throttle_message_object)
        create_receiver("throttle", throttle_feedback_msg_obj, "feedback")
        create_receiver("throttle", throttle_hearbeat_msg_obj, "heartbeat")

    if brake_check:
        create_sender("brake", can_sender_publisher, brake_message_object)   

    if steering_check:
        create_sender("steering", can_sender_publisher, steering_message_object)
        create_receiver("steering", steering_feedback_msg_object, "feedback")
        create_receiver("steering", steering_heartbeat_msg_object, "heartbeat")

    if encoder_check:
        create_receiver("encoder", encoder_feedback_msg_object, "feedback")
        create_receiver("encoder", encoder_heartbeat_msg_object, "heartbeat")

    rospy.spin()
