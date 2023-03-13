#!/usr/bin/env python3

#use service or actions 
#service is blocking 
#actions are non-blocking

import rospy
import time
from sensor_msgs.msg import Image
import ros_numpy

# OpenCV2 for saving an image
import cv2


class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        #waitformessage will work here 
        self.sub = rospy.Subscriber(topic, Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        print("Received an image!")

        # Convert your ROS Image message to OpenCV2 
        cv2_img = ros_numpy.numpify(data)
        # cv2_img = bridge.imgmsg_to_cv2(data, "bgr8") #do this for once
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite("camera_image.jpg", cv2_img) #for once
        print("saved image on program startup")


def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
  
    topic = '/camera/color/image_raw'  # check the depth image topic in your Gazebo environmemt and replace this with your
    
    
    ImageListener(topic)
    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped 

if __name__ == '__main__':
    listener()




