#!/usr/bin/env python3
import rospy
import ros_numpy
import sys
import os
import time
from xarm_sdk.xarm.wrapper import XArmAPI
import cv2
import numpy as np

from depth import ImageListener
from armmove import RobotMain
from pytorch_retinanet_master.visualize_single_image import detect_image 
#from listener_ts import callback

#can only be rosrun 

from sensor_msgs.msg import Image as msg_Image
from std_msgs.msg import Bool

# move down 
fork = 290 #measured, bottom of fork is roughly 30 cm below camera 
forkvals = 150

#to keep moving up in filenames
def get_next_filename(dir_and_filename_base, jpg=True):
     next_file_ind_to_check = 0
     while True: 
          if jpg:
               filename = dir_and_filename_base +str(next_file_ind_to_check).zfill(5)+ '.jpg' 
          else:
               # assume it will be npy
               filename = dir_and_filename_base +str(next_file_ind_to_check).zfill(5)+ '.npy' 
          if not os.path.isfile(filename):
               break
          next_file_ind_to_check += 1
     return filename

def leastdepth(data, x1, x2, y1, y2):
     try:
#bridge.imgmsg_to_cv2(data, data.encoding)
          for x in range(x1, x2):            
               for y in range(y1, y2):    
                    if (data[x][y] < fork):
                         data[x][y] = 1000     #not working 
                    

          #sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))

          #sys.stdout.flush()
          
     except CvBridgeError as e:
          print(e)
     BBsubset = data[x1:x2,y1:y2] #to return value from the subset
     return BBsubset.min() 

if __name__ == '__main__':
     print("moving robot to starting pos")
     rospy.loginfo("arm moving now")
     arm = XArmAPI('192.168.1.201', baud_checkset=False)
     robot_main = RobotMain(arm)
     robot_main.setpos()

     print("Initializing node training_master")
     #initial photo for retinanet
     rospy.init_node("training_master")
     pub = rospy.Publisher('store_data', Bool, queue_size=10)
     rate = rospy.Rate(100) 

     #image for retinanet
     phototopic = '/camera/color/image_raw' 
     dataretphoto = rospy.wait_for_message(phototopic, msg_Image)
     rospy.loginfo("Obtained color image")
     
     #putting into jpg for retinanet
     cv2_img_ret = ros_numpy.numpify(dataretphoto)
     cv2_img_ret = cv2.cvtColor(cv2_img_ret, cv2.COLOR_BGR2RGB)
     rospy.loginfo("put image for retinanet into jpg")
     # cv2_img = bridge.imgmsg_to_cv2(data, "bgr8") 

     #generation of bounding boxes via retinanet
     bbimage_folder = '/home/neha/Desktop/fyp_2223_sam/catkin_ws/src/fyp_code/data/bbimages/'
     ret_image = get_next_filename(bbimage_folder, jpg=True)
     rospy.loginfo("just saved")
     rospy.loginfo(ret_image)
     cv2.imwrite(ret_image, cv2_img_ret) #image is opened from within detect_image function
     model_path = '/home/neha/Desktop/fyp_2223_sam/catkin_ws/src/fyp_code/src/fyp_code/pytorch_retinanet_master/resnet50-19c8e357.pth'
     class_list='/home/neha/Desktop/fyp_2223_sam/catkin_ws/src/fyp_code/src/fyp_code/pytorch_retinanet_master/csvdataset/class_list.csv'
     
     BBlist = detect_image(cv2_img_ret, model_path, class_list) 
     rospy.loginfo("generated BBs via retnet")
     print(BBlist)
     rospy.loginfo("opened the photo with BBs")

     #handling depth image
     depthtopic = '/camera/depth/image_rect_raw'
     datadepth = rospy.wait_for_message(depthtopic, msg_Image)
     cv2_img_depth = ros_numpy.numpify(datadepth)
     rospy.loginfo("got depth image")
     
     # check that bblist is not None
     if BBlist[0] is None:
          rospy.logwarn("No bounding box found!")
          BBlist = [290,380,327,450] #fixed for override from opencv image 
          min = leastdepth(cv2_img_depth,BBlist[0],BBlist[1],BBlist[2],BBlist[3])
          
     else:
          min = leastdepth(cv2_img_depth,BBlist[0],BBlist[1],BBlist[2],BBlist[3])
          #move to over where where the BB center is 
     print("min is", min)

     lengthtomove = min - fork
     print("will move down", lengthtomove)


     #move down in x direction

     robot_main = RobotMain(arm)
     robot_main.run(depth=-lengthtomove)


     rospy.loginfo("taking photo for training")
     datahvn = rospy.wait_for_message(phototopic, msg_Image)
     image_folder = '/home/neha/Desktop/fyp_2223_sam/catkin_ws/src/fyp_code/data/images/'
     store_image = get_next_filename(image_folder, jpg=True)
     rospy.loginfo("just saved")
     rospy.loginfo(store_image)
     cv2_img_hvn = ros_numpy.numpify(datahvn)
     cv2_img_hvn = cv2.cvtColor(cv2_img_hvn, cv2.COLOR_BGR2RGB)
     cv2.imwrite(store_image , cv2_img_hvn) 


     oldtime = time.time()

     rospy.loginfo("touch sensor data collection")
     while (time.time()-oldtime < 0.5): #collect data for 300 milliseconds
          #must run listener.py independently
     # 1. publish to store_data (True)
          msg_tsdatabool = Bool()
          msg_tsdatabool.data = True
          rospy.loginfo(msg_tsdatabool)
          pub.publish(msg_tsdatabool)
          rate.sleep()
# 2. move arm
          robot_main.run(depth=-15)

     # 3. publish to stop storing (False)
     msg_tsdatabool = Bool()
     msg_tsdatabool.data = False
     rospy.loginfo(msg_tsdatabool)
     pub.publish(msg_tsdatabool)
     rate.sleep()

     rospy.loginfo("done with touch sensor data collection")

     #user input for ground truth label :) 

     skewertype = int(input("Enter 1 for hard food, 2 for soft food"))
     label_folder = '/home/neha/Desktop/fyp_2223_sam/catkin_ws/src/fyp_code/data/labels/'
     store_labels = get_next_filename(label_folder, jpg=False)
     rospy.loginfo("just saved")
     rospy.loginfo(store_labels)
     np.save(store_labels,skewertype)

     robot_main.setpos()

     rospy.loginfo("training data successfully collected!")

#     things todo:
#          depth.py sortout x
#          store data for 200 ms x
#          input gtruth label x
#          two more xarm things: set posn at the start, camera intrinsic move in the middle (optional first)
#         save listener_ts.py stuff in .npy x
#         make imagedepthcallback a fn not a callback
#         change all to numpify
#         youre collecting 3 tings so differentiate x