#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import rosnode

# Callbacks definition

def active_cb():
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    a = 1
    #rospy.loginfo("Current location: " + str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
        #pub.publish(name + " has reached its goal and is now sleeping")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")
    

def callback(data):
    #rospy.loginfo(name + " heard \"%s\"", data.data)
    print(name, "heard: i = %d, x = %g, y = %g, sleep_time = %g" % (data.data[0], data.data[1], data.data[2], data.data[3]))
    print(data.data)
    task = [int(data.data[0]), round(data.data[1], 2), round(data.data[2], 2), int(data.data[3])]
    print(task)
    current_task(task[0], task[1], task[2], task[3])
    #print(name, "heard: data[0]=", data[0], "data.data[1]=", data[1])


rospy.init_node('goal_pose')
name = rospy.get_name()

navclient = actionlib.SimpleActionClient(name + '/move_base', MoveBaseAction)
navclient.wait_for_server()
#node_names = rosnode.get_node_names()
turtlebot_names = ["tb3_0", "tb3_1", "tb3_2"]


for i in range (len(turtlebot_names)):
    #print(turtlebot_names[i])
    if turtlebot_names[i] not in name:
        rospy.Subscriber(turtlebot_names[i], Float32MultiArray, callback)
        print("Subscribed to", turtlebot_names[i])


pub = rospy.Publisher(name, Float32MultiArray, queue_size = 10)

def current_task(i, x_coordinate, y_coordinate, sleep_time):
    
    print("\nGoing to task", i,": x_coordinate:", x_coordinate, "y_coordinate:", y_coordinate, "sleep_time:", sleep_time)

    # Example of navigation goal
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.seq = i

    goal.target_pose.pose.position.x = x_coordinate
    goal.target_pose.pose.position.y = y_coordinate
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0


    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = navclient.wait_for_result(rospy.Duration(35))


    if not finished:
        rospy.logerr("Can't reach goal")
        #pub.publish(name + " can't reach the goal")
        task = Float32MultiArray()
        task.data = [i, x_coordinate, y_coordinate, sleep_time]
        pub.publish(task)
    else:
        rospy.loginfo(navclient.get_result())
    
    print("The goal of task ", i, " was:    x_coordinate:", x_coordinate, "y_coordinate:", y_coordinate, "sleep_time:", sleep_time)
    
    
    rospy.sleep(sleep_time)

def task_list(tasks_seed):
    
    random.seed(str(tasks_seed) + name)

    no_of_tasks = 4

    for i in range (1, no_of_tasks+1):
        x_coordinate = round(random.uniform(0.5, 9.5), 1)
        y_coordinate = round(random.uniform(0.5, 9.5), 1)
        sleep_time = random.randint(3, 8)
        
        current_task(i, x_coordinate, y_coordinate, sleep_time)

task_list(1)