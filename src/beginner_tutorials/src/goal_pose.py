#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math

robot_pose = None
combined_scan = None
# Callbacks definition
def combined_scan_callback(scan_msg):
    global combined_scan
    combined_scan = scan_msg



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
# Map callback
def map_callback(data):
    global map_data
    map_data = data

# Pose callback
def pose_callback(pose_stamped):
    global robot_pose
    robot_pose = pose_stamped.pose.pose

def bresenham_line(x1, y1, x2, y2):
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x, y = x1, y1
    sx = -1 if x1 > x2 else 1
    sy = -1 if y1 > y2 else 1

    if dx > dy:
        err = dx / 2.0
        while x != x2:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y2:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    points.append((x, y))
    return points

def world_to_map(x, y, map_info):
    mx = int((x - map_info.origin.position.x) / map_info.resolution)
    my = int((y - map_info.origin.position.y) / map_info.resolution)
    return mx, my

def map_to_world(mx, my, map_info):
    x = mx * map_info.resolution + map_info.origin.position.x
    y = my * map_info.resolution + map_info.origin.position.y
    return x, y

def has_obstacle_between(robot_pose, goal_pose, map_data):
    map_info = map_data.info
    robot_mx, robot_my = world_to_map(robot_pose.position.x, robot_pose.position.y, map_info)
    goal_mx, goal_my = world_to_map(goal_pose.position.x, goal_pose.position.y, map_info)

    # Generate points using Bresenham's line algorithm
    points = bresenham_line(robot_mx, robot_my, goal_mx, goal_my)

    # Check for obstacles
    for mx, my in points:
        index = my * map_info.width + mx
        if 0 <= index < len(map_data.data) and map_data.data[index] > 0:
            return True

    return False

rospy.init_node('goal_pose')
name = rospy.get_name()
map_sub = rospy.Subscriber(name + "/map", OccupancyGrid, map_callback)
pose_sub = rospy.Subscriber(name + "/amcl_pose",  PoseWithCovarianceStamped, pose_callback)
rospy.Subscriber('/shared_scans', LaserScan, combined_scan_callback)
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

while not rospy.is_shutdown():
    if map_data and robot_pose:
        break
    rospy.sleep(0.5)

if not has_obstacle_between(robot_pose, goal.target_pose.pose, map_data):
    print("No obstacle between the robot and the goal pose")
    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished1 = navclient.wait_for_result()
else:
    rospy.loginfo("Obstacle detected between the robot and the goal pose")
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
