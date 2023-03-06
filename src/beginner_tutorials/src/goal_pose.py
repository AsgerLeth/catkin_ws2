#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Callbacks definition

def active_cb():
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")
    

rospy.init_node('goal_pose')
name = rospy.get_name()

navclient = actionlib.SimpleActionClient(name + '/move_base',MoveBaseAction)
navclient.wait_for_server()

# Example of navigation goal
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x = 3.16
goal.target_pose.pose.position.y = 3.764
goal.target_pose.pose.position.z = 0.0
goal.target_pose.pose.orientation.x = 0.0
goal.target_pose.pose.orientation.y = 0.0
goal.target_pose.pose.orientation.z = 0.662
goal.target_pose.pose.orientation.w = 0.750

navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
finished1 = navclient.wait_for_result()

if finished1:
    rospy.sleep(10)
    goal2 = MoveBaseGoal()
    goal2.target_pose.header.frame_id = "map"
    goal2.target_pose.header.stamp = rospy.Time.now()

    goal2.target_pose.pose.position.x = 5.16
    goal2.target_pose.pose.position.y = 5.764
    goal2.target_pose.pose.position.z = 0.0
    goal2.target_pose.pose.orientation.x = 0.0
    goal2.target_pose.pose.orientation.y = 0.0
    goal2.target_pose.pose.orientation.z = 0.662
    goal2.target_pose.pose.orientation.w = 0.750
    navclient.send_goal(goal2, done_cb, active_cb, feedback_cb)
    finished2 = navclient.wait_for_result()
else :
    rospy.logerr("Action server not available!")


if not finished1:
    rospy.logerr("Action server not available!")
else:
    rospy.loginfo ( navclient.get_result())
