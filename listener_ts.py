#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
#wait_for_message

import rospy
import time
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os

#date = time.strftime("%H:%M:%S %d %b", time.gmtime())
#filename = "data " + date + ".txt"
#text_file = open(filename, "x")

y = [] #to save one value of ts
count = 0 #data saving purposes

# Function to get the next available index for the file
def get_next_filename(dir_and_filename_base):
    next_file_ind_to_check = 0
    while True: 
          filename = dir_and_filename_base +str(next_file_ind_to_check).zfill(5) + '.npy'
          if not os.path.isfile(filename):
               break
          next_file_ind_to_check += 1
    return filename

class TouchSensorData():
    def __init__(self):
        self.touch_sensor_data = []
        self.store = False #flag turned on and off to record touch sensor data 
        rospy.Subscriber('chatter', Float32MultiArray, self.touch_sensor_cb) 
        rospy.Subscriber('store_data', Bool, self.store_cb) 



    def touch_sensor_cb(self, data):
        #rospy.loginfo(data.data)
        # saveData = np.array(x)
        # # File prefix and suffix
        # prefix = ''
        # suffix = '.npy'
        # dir_path = './'
        # # Get the next available index for the file
        # next_index = get_next_index(dir_path, prefix, suffix)

        # # Save the array to a .npy file with the next available index
        # np.save(os.path.join(dir_path, prefix + str(next_index).zfill(5) + suffix), x)

        if self.store:
            # dataraw = [float(x) for x in data.data]
            # print(dataraw)
            # for i in dataraw:
            #     if dataraw[i]<50:
            #         dataraw.pop[i]
            # avg = sum(data) / len(data)
            # y.append(count, int(avg))
            self.touch_sensor_data.append(data.data)

    def store_cb(self, data):
        # data should be boolean
        if data.data:
            self.store = True
            rospy.loginfo("saving data")

        else:
            self.store = False
            # save data
            print(self.touch_sensor_data)
            forcesave = '/home/neha/Desktop/fyp_2223_sam/catkin_ws/src/fyp_code/data/force/'
            forcefilename = get_next_filename(forcesave)
            np.save(forcefilename, self.touch_sensor_data)
            rospy.loginfo("saved data")
            rospy.loginfo(forcefilename)
            
            # clear data
            self.touch_sensor_data.clear()





if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True) 

    TouchSensorData()
    while not rospy.is_shutdown():
        rospy.spin()



