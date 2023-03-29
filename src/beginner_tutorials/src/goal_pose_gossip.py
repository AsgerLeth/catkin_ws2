#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
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
    print("data 0 =",data.data[0])
    if (int(data.data[0]) == 0):
        print("responder")
        responder(data.data[1])
    
    elif (int(data.data[0]) == 1):
        print("check_range")
        check_range(data.data[1], data.data[2], data.data[3])

    elif (int(data.data[0]) == 2):
        print("append task")
        task = [round(data.data[1], 2), round(data.data[2], 2), int(data.data[3])]
        print(tasks)
        tasks.data.append(task)
        print(tasks)



rospy.init_node('goal_pose')
name = rospy.get_name()
odom_msg = rospy.wait_for_message(name + '/odom', Odometry)

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

tasks = Float32MultiArray()

def check_range(other_x, other_y, failed_task_index):
    odom_msg = rospy.wait_for_message(name + '/odom', Odometry)
    print("other_x=%f, other_y=%f, my_x=%f, my_y=%f" % (other_x, other_y, odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y))
    if (abs(other_x - odom_msg.pose.pose.position.x)**2 + abs(other_y - odom_msg.pose.pose.position.y)**2 < 2 ):
        print("ROBOT IS WITHIN RANGE!")
        failed_task = Float32MultiArray()
        failed_task.data = tasks.data[int(failed_task_index)]
        print(failed_task.data)
        failed_task.data.insert(0, 2)
        print(failed_task.data)
        pub.publish(failed_task)
    else:
        print("No robots in range!")

def responder(failed_task_index):
    odom_msg = rospy.wait_for_message(name + '/odom', Odometry)
    x_current = odom_msg.pose.pose.position.x
    y_current = odom_msg.pose.pose.position.y

    print("x_current =", x_current, "y_current =", y_current)
    pos = Float32MultiArray()
    pos.data = [1, x_current, y_current, failed_task_index]
    pub.publish(pos)

def do_task(tasks):
    
    print("len tasks = ", len(tasks.data))
    i = 0

    while i < (len(tasks.data)):

        print("\ntask", i, ": task_x:", tasks.data[i][0], "task_y:", tasks.data[i][1], "hover_duration:", tasks.data[i][2])

        odom_msg = rospy.wait_for_message(name + '/odom', Odometry)
        x_start = odom_msg.pose.pose.position.x
        y_start = odom_msg.pose.pose.position.y

        print("x_start =", x_start, "y_start =", y_start)

        # Example of navigation goal
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.seq = i

        goal.target_pose.pose.position.x = tasks.data[i][0]
        goal.target_pose.pose.position.y = tasks.data[i][1]
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
            task_failed = Float32MultiArray()
            task_failed.data = [0, i]
            
            pub.publish(task_failed)
        else:
            rospy.loginfo(navclient.get_result())
        
        print("The goal of tasks ", i, " was:    task_x:", tasks.data[i][0], "task_y:", tasks.data[i][1], "hover_duration:", tasks.data[i][2])
        
        rospy.sleep(tasks.data[i][2])
        i += 1

def task_list(tasks_seed):
    
    random.seed(str(tasks_seed) + name)

    no_of_tasks = 4

    for i in range (no_of_tasks):
        task_x = round(random.uniform(0.5, 9.5), 1)
        task_y = round(random.uniform(0.5, 9.5), 1)
        hover_duration = random.randint(3, 8)
        
        tasks.data.append([task_x, task_y, hover_duration])

    do_task(tasks)

task_list(1)