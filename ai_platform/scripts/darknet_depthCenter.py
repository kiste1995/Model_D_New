#! /usr/bin/env python

import rospy, time
import numpy as np
import actionlib
import sys, os, cv2, time
from cv_bridge import CvBridge, CvBridgeError
# from skimage import img_as_ubyte, img_as_float


    

from sensor_msgs.msg import Image as msg_Image
from std_msgs.msg import String as string
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
#sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from darknet_ros_msgs.msg import BoundingBoxes, BoundingBoxesCenter, BoundingBoxCenter

import threading

bridge = CvBridge()
boundingBoxCenter = BoundingBoxCenter()
object_list = BoundingBoxesCenter()

class Darknet:
    minimum = []
    object_class = []
    def __init__(self, darknet_topic, depth_topic):

        self.darknet_sub = rospy.Subscriber(darknet_topic, BoundingBoxes, self.callback_darknet)
        self.depth_sub = rospy.Subscriber(depth_topic, msg_Image, self.callback_depth)
        self.pub_darknet_depth = rospy.Publisher('darknet_depth', BoundingBoxesCenter, queue_size = 10)
        rospy.spin()

    def callback_darknet(self, darknet_topic):
        
        self.box_list = darknet_topic.bounding_boxes
        self.center_list = []
        self.object_class= []
     
        for i in range(len(self.box_list)):
            self.center_point = []     
            x_min = self.box_list[i].xmin
            y_min = self.box_list[i].ymin
            x_max = self.box_list[i].xmax
            y_max = self.box_list[i].ymax
            d_Class = self.box_list[i].Class
            self.x = (x_max - x_min)/2 + x_min
            self.y = (y_max - y_min)/2 + y_min

            self.center_point.append(self.center_function(self.x, x_min, self.y, y_min))
            self.center_point.append(self.center_function(x_max, self.x, self.y, y_min))
            self.center_point.append(self.center_function(self.x, x_min, y_max, self.y))
            self.center_point.append(self.center_function(x_max, self.x, y_max, self.y))
            self.center_point.append([self.x, self.y])
            self.center_list.append(self.center_point)
            self.object_class.append(d_Class)

            # print(Class)
            # print(self.center_list)
            
            
    def callback_depth(self, depth_topic):
        cv_image = bridge.imgmsg_to_cv2(depth_topic, depth_topic.encoding)
        # depth image
        # img_array = np.array(cv_image, dtype = np.dtype('f8'))
        # img_norm = cv2.normalize(img_array, img_array, 0, 1, cv2.NORM_MINMAX)
        # src = img_as_ubyte(img_norm)
        # cv2.imshow('image', cv_image)
        # cv2.waitKey(3)
        # print(img_array.shape) = 480, 640
        # depth data = center_list[i][c1,2,3,4][dx,dy]
        self.minimum=[]
        for i in range(len(self.box_list)):
            depth_center = []
            for j in range(5):
                dx = self.center_list[i][j]
                dy = self.center_list[i][j]

                # print('i : %d c%d = %5.1f mm'% (i, j, cv_image[dy[1], dx[0]]))
                depth_center.append(cv_image[dy[1], dx[0]])

        # depth min
            self.minimum.append(depth_center[0])
            for a in range(len(depth_center)):
                if depth_center[a] <= self.minimum[-1]:
                    if depth_center[a] != 0 :
                        self.minimum[-1] = depth_center[a]

            print("-"*15+"minimum"+"-"*15)
            print(self.object_class)
            print(self.minimum)   
	# publish

        for i in range(len(self.box_list)):
            msg_center = BoundingBoxCenter()
            msg_center.minimum  = self.minimum[i] 
            msg_center.Class = self.object_class[i]
            object_list.bounding_boxes_center.append(msg_center)
            
            boundingBoxCenter.Class = self.object_class[i]
            boundingBoxCenter.minimum = self.minimum[i]
        
        self.pub_darknet_depth.publish(object_list)
        object_list.bounding_boxes_center = []
        self.box_list = []
        
    # center calculate
    def center_function(self, a, b, c, d):
        center_x = (a - b)/2 + b 
        center_y = (c - d)/2 + d
        center = [center_x,center_y]
       
        return center

def main():
# darknet, depth rostopic

    darknet_topic = '/darknet_ros/bounding_boxes'
    depth_topic = '/camera/depth/image_rect_raw'

    Darknet_Listener = Darknet(darknet_topic, depth_topic)
    
    while not rospy.is_shutdown():

        rospy.Rate(10).sleep()


if __name__ == '__main__' :
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
    
