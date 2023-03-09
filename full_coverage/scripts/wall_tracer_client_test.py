#! /usr/bin/env python

import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import full_coverage.msg

def wall_tracer_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('wall_tracer', full_coverage.msg.Wall_tracerAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = full_coverage.msg.Wall_tracerGoal(command = "hi")

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('wall_tracer_client_py')
        result = wall_tracer_client()
        print("Result :    ", result.result)
        print("Result:", ', '.join([str(n) for n in result.result]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")