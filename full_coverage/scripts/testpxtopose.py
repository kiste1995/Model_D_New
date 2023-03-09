#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from full_coverage.srv import Fullpath
from full_coverage.msg import Pxmsg

g_map =None
qq =None

def calcPixel2Pos(data):
    global g_map
    tfs=TransformStamped()
    tfs.header.frame_id = 'map'
    tfs.child_frame_id = 'goal'
    tfs.transform.translation.x = g_map.origin.position.x + data.pose_x * g_map.resolution
    tfs.transform.translation.y = g_map.origin.position.y + g_map.height * g_map.resolution - data.pose_y * g_map.resolution
    tfs.transform.translation.z = data.pose_theta

    print('x:',tfs.transform.translation.x,'y:',tfs.transform.translation.y)

    quaternion = tf.transformations.quaternion_from_euler(0, 0, data.pose_theta)
    tfs.transform.rotation.x = quaternion[0]
    tfs.transform.rotation.y = quaternion[1]
    tfs.transform.rotation.z = quaternion[2]
    tfs.transform.rotation.w = quaternion[3]


def mapcb(data):
    global g_map
    g_map = data
    print ("dddddddddddddd",g_map)

if __name__ == '__main__':
    try:
        rospy.init_node('test_pxtopose_py')
        rospy.Subscriber("/map_metadata", MapMetaData ,mapcb)
        rospy.Subscriber("/test_px", Pxmsg ,calcPixel2Pos)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    rospy.spin()
