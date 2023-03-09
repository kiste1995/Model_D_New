#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

from std_srvs.srv import Empty

g_map =None
qq =None

def movebase_client():
    clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
    clear_costmaps_srv()

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        # rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def calcPixel2Pos(x,y,th):
    tfs=TransformStamped()
    tfs.header.frame_id = 'map'
    tfs.child_frame_id = 'goal'
    tfs.transform.translation.x = g_map.origin.position.x + x * g_map.resolution 
    tfs.transform.translation.y =  g_map.origin.position.y + g_map.height * g_map.resolution - y * g_map.resolution 
    tfs.transform.translation.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, th)
    tfs.transform.rotation.x = quaternion[0]
    tfs.transform.rotation.y = quaternion[1]
    tfs.transform.rotation.z = quaternion[2]
    tfs.transform.rotation.w = quaternion[3]
    return tfs

def mapcb(data):
    global g_map
    g_map = data
    print "mapcb~!!!!!!!"
    


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')

        rospy.Subscriber("/map_metadata", MapMetaData ,mapcb)
        rospy.sleep(2)
        print calcPixel2Pos(500,800,0)
        #result = movebase_client()
        #if result:
        #  rospy.loginfo("Goal execution done!")
        

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")




'''

   5 def callback(data):
   6     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   7     
   8 def listener():
   9 
  10     # In ROS, nodes are uniquely named. If two nodes with the same
  11     # name are launched, the previous one is kicked off. The
  12     # anonymous=True flag means that rospy will choose a unique
  13     # name for our 'listener' node so that multiple listeners can
  14     # run simultaneously.
  15     rospy.init_node('listener', anonymous=True)
  16 
  17     rospy.Subscriber("chatter", String, callback)
  nav_msgs::MapMetaData g_map_data; 





mapSubscriber = nh_.subscribe("/map_metadata", 1000, &ZetabotControl::mapInfoCb, this);


geometry_msgs::TransformStamped ZetabotControl::calcPixel2Pos(float goal_x, float goal_y, float goal_theta)
{
  geometry_msgs::TransformStamped transformStamped;

  //set a tf inheriting map.
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "goal";
  // origin(0, 0) = (g_map_data.origin.position.x,  g_map_data.origin.position.y + g_map_data.height *g_map_data.resolution)
  transformStamped.transform.translation.x =  g_map_data.origin.position.x + goal_x * g_map_data.resolution ;
  transformStamped.transform.translation.y =  g_map_data.origin.position.y + g_map_data.height *g_map_data.resolution - goal_y * g_map_data.resolution ;
  transformStamped.transform.translation.z = 0.0;

  q.setRPY(0, 0, goal_theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  
  transformStamped.header.stamp = ros::Time::now();

  return transformStamped;
}
'''