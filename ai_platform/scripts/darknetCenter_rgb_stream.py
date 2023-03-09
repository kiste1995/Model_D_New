#!/usr/bin/env python
import rospy, time
import numpy as np
import actionlib
import sys, os, cv2, time, random
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
    
    def __init__(self, darknet_topic, depth_topic, rgb_topic):

        self.darknet_sub = rospy.Subscriber(darknet_topic, BoundingBoxes, self.callback_darknet)
        self.depth_sub = rospy.Subscriber(depth_topic, msg_Image, self.callback_depth)
        self.pub_darknet_depth = rospy.Publisher('darknet_depth', BoundingBoxesCenter, queue_size = 10)


        # self.rgb_sub = rospy.Subscriber(rgb_topic, msg_Image, self.callback_rgb)

        rospy.spin()

    def callback_darknet(self, darknet_topic):
        
        self.box_list = darknet_topic.bounding_boxes
        self.center_list = []
        self.object_class= []
        self.rgb_list = darknet_topic.bounding_boxes

        self.header = darknet_topic.header
        

        ###################### 3x3 ######################
        for i in range(len(self.box_list)):
            self.center_point = []     
            x_min = self.box_list[i].xmin
            y_min = self.box_list[i].ymin
            x_max = self.box_list[i].xmax
            y_max = self.box_list[i].ymax

            d_Class = self.box_list[i].Class
            self.x = (x_max - x_min)/2 + x_min
            self.y = (y_max - y_min)/2 + y_min

            x1 = (x_max - x_min)/3 + x_min
            y1 = (y_max - y_min)/3 + y_min

            x2 = (x_max - x_min)*2/3 + x_min
            y2 = (y_max - y_min)/3 + y_min

            x3 = (x_max - x_min)/3 + x_min
            y3 = (y_max - y_min)*2/3 + y_min

            x4 = (x_max - x_min)*2/3 + x_min
            y4 = (y_max - y_min)*2/3 + y_min

            self.center_point.append([self.x, self.y])
            self.center_point.append([x1, y1])
            self.center_point.append([x2, y2])
            self.center_point.append([x3, y3])
            self.center_point.append([x4, y4])
            self.center_list.append(self.center_point)
            self.object_class.append(d_Class)

        self.pub_darknet_depth.publish(object_list)
        
        # self.box_list = []
    ###################### 2x2 ######################
        # for i in range(len(self.box_list)):
        #     self.center_point = []     
        #     self.x_min = self.box_list[i].xmin
        #     self.y_min = self.box_list[i].ymin
        #     self.x_max = self.box_list[i].xmax
        #     self.y_max = self.box_list[i].ymax
        #     d_Class = self.box_list[i].Class
        #     self.x = (self.x_max - self.x_min)/2 + self.x_min
        #     self.y = (self.y_max - self.y_min)/2 + self.y_min

        #     self.center_point.append(self.center_function(self.x, self.x_min, self.y, self.y_min))
        #     self.center_point.append(self.center_function(self.x_max, self.x, self.y, self.y_min))
        #     self.center_point.append(self.center_function(self.x, self.x_min, self.y_max, self.y))
        #     self.center_point.append(self.center_function(self.x_max, self.x, self.y_max, self.y))
        #     self.center_point.append([self.x, self.y])
        #     self.center_list.append(self.center_point)
        #     self.object_class.append(d_Class)       
            
    def callback_depth(self, depth_topic):
        cv_image = bridge.imgmsg_to_cv2(depth_topic, depth_topic.encoding)
    # depth image stream
        # img_array = np.array(cv_image, dtype = np.dtype('f8'))
        # img_norm = cv2.normalize(img_array, img_array, 0, 1, cv2.NORM_MINMAX)
        # src = img_as_ubyte(img_norm)
        # cv2.imshow('image', cv_image)
        # cv2.waitKey(3)
        # print(img_array.shape) = 480, 640
        # depth data = center_list[i][c1,2,3,4][dx,dy]
    # receive box depth
        self.minimum=[]
        object_list.bounding_boxes_center = []
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
            
            #5people
            # for j in range(0,5) :
            #     object_list.bounding_boxes_center.append(msg_center)

            #solo
            object_list.bounding_boxes_center.append(msg_center)
            
            boundingBoxCenter.Class = self.object_class[i]
            boundingBoxCenter.minimum = self.minimum[i]


        
    def callback_rgb(self, rgb_topic):
        global point_list1, point_list2, text_list, count, temp, nex, t
      
        try:
            #color
           
            red_color = (0,0,255)
            blue_color = (255,0,0)
            green_color = (0,255,0)
            black_color = (255,200,200)
            
            color = [red_color, blue_color, green_color]
            choice_color = random.choice(color)

            
            base_rgb_list = []
            pt1 = ()
            pt2 = ()
            centerPoint = ()    
            centerPoint_list = []
            text = ""
            point_list1 = []
            point_list2 = []
            text_list = []
            image = np.zeros((640,480,3),np.uint8)
            #font
            fonts = cv2.FONT_HERSHEY_PLAIN

            image = bridge.imgmsg_to_cv2(rgb_topic, "bgr8")

            # A = base_rgb_list 
            # B = self.rgb_list
            
            # if base_rgb_list == self.rgb_list:
                
            #     t = time.time()
            # else:
            #     base_rgb_list = self.rgb_list
               
            # if time.time() - t >= 1:
            #     base_rgb_list = []
            #     cv2.rectangle(image)
            #     cv2.putText(image)  
            # print(self.rgb_list)
            
            if count == 0:
                temp = self.header.seq            
                nex = temp+1
            
            count = count + 1
   
            if nex == self.header.seq:
                temp = temp + 1
                nex = temp + 1 
                t= time.time()
            elif time.time() - t > 1:
                self.rgb_list = [] 
                cv2.rectangle(image, (0,0), (0,0), red_color, 0)
                cv2.putText(image, text, (0,0), fonts, 0, red_color, 0, cv2.LINE_AA)  

            for i in range(len(self.rgb_list)):
                
                text = self.rgb_list[i].Class
                x_min = self.rgb_list[i].xmin
                y_min = self.rgb_list[i].ymin
                x_max = self.rgb_list[i].xmax
                y_max = self.rgb_list[i].ymax

                x = (x_max - x_min)/2 + x_min
                y = (y_max - y_min)/2 + y_min

                pt1 = (x_min, y_min)
                pt2 = (x_max, y_max)
                centerPoint = (x,y)

                point_list1.append(pt1)
                point_list2.append(pt2)
                centerPoint_list.append(centerPoint)
                text_list.append(text)

                for j in range(i):
                    if text_list[j] == text_list[i]:
                        if (centerPoint_list[j][0]-30 <= centerPoint_list[i][0] <= centerPoint_list[j][0]+30) and (centerPoint_list[j][1]-30 <= centerPoint_list[i][1] <= centerPoint_list[j][1]+30) :
                            hypotenuse1 = self.trangle_function(point_list1[i], point_list2[i])
                            hypotenuse2 = self.trangle_function(point_list1[j], point_list2[j])

                            if hypotenuse1 > hypotenuse2:
                                point_list1[i] = point_list1[j]
                                point_list2[i] = point_list2[j]
                            elif hypotenuse1 < hypotenuse2:
                                point_list1[i] = point_list1[j]
                                point_list2[i] = point_list2[j]  
                            else:
                                point_list1[i] = point_list1[j]
                                point_list2[i] = point_list2[j]

                cv2.rectangle(image, point_list1[i], point_list2[i], red_color, 2)
                cv2.putText(image, text, point_list1[i], fonts, 2, red_color, 2, cv2.LINE_AA)
                
            #cv2.imshow('image', image)
            cv2.waitKey(1)     

        except CvBridgeError as e:
            print(e)

    # center calculate
    def center_function(self, a, b, c, d):
        center_x = (a - b)/2 + b 
        center_y = (c - d)/2 + d
        center = [center_x,center_y]
       
        return center
    def trangle_function(self, tuple1, tuple2):
        x1 = tuple2[0] - tuple1[0]
        y1 = tuple2[1] - tuple1[1]
        c = x1^2 + y1^2

        return c
def main():
# darknet, depth rostopic
    global count
    count = 0
    
    darknet_topic = '/darknet_ros/bounding_boxes'
    depth_topic = '/camera/depth/image_rect_raw'
    rgb_topic = '/camera/color/image_raw'

    Darknet_Listener = Darknet(darknet_topic, depth_topic, rgb_topic)

    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()

if __name__ == '__main__' :
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
    
