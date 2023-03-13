import rospy
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os


forkvals = 300


class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data, x1, x2, y1, y2):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            for x in range(x1, x2):            
                for y in range(y1, y2):    
                    depthtest = cv_image[x, y]
                    if (cv_image[x, y] < forkvals):
                        cv_image[x, y] = 1000     
                        

            #sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))

            #sys.stdout.flush()
            
        except CvBridgeError as e:
            print(e)
        return cv_image.min() #wont work 





if __name__ == '__main__':
    rospy.init_node("depth_image_processor")
    topic = '/camera/depth/image_rect_raw'  # check the depth image topic in your Gazebo environmemt and replace this with your
    listener = ImageListener(topic)
    print(listener)
    rospy.spin()