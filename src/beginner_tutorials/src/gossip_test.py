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
        check_range(data.data[1], data.data[2])

    elif (int(data.data[0]) == 2):
        print("append task")
        if (data.data[4] == turtlebot_names.index(name)):
            task = [round(data.data[1], 2), round(data.data[2], 2), int(data.data[3])]
            print(tasks)
            not_in_list = True
            for i in range(len(tasks.data)):
                if (task == tasks.data[i]):
                    not_in_list = False
            if not_in_list == True:
                tasks.data.append(task)
            print(tasks)

    elif (int(data.data[0]) == 3):
        if (data.data[4] == 0):
            completed_task = [round(data.data[1], 2), round(data.data[2], 2), int(data.data[3])]
            tb3_0_completed_tasks.data.append(completed_task)
            print("tasks completed by tb3_0 = ", tb3_0_completed_tasks.data)

        elif (data.data[4] == 1):
            completed_task = [round(data.data[1], 2), round(data.data[2], 2), int(data.data[3])]
            tb3_1_completed_tasks.data.append(completed_task)
            print("tasks completed by tb3_1 = ", tb3_1_completed_tasks.data)

        elif (data.data[4] == 2):
            completed_task = [round(data.data[1], 2), round(data.data[2], 2), int(data.data[3])]
            tb3_2_completed_tasks.data.append(completed_task)
            print("tasks completed by tb3_2 = ", tb3_2_completed_tasks.data)

        elif (data.data[4] == 3):
            completed_task = [round(data.data[1], 2), round(data.data[2], 2), int(data.data[3])]
            tb3_3_completed_tasks.data.append(completed_task)
            print("tasks completed by tb3_3 = ", tb3_3_completed_tasks.data)

        elif (data.data[4] == 4):
            completed_task = [round(data.data[1], 2), round(data.data[2], 2), int(data.data[3])]
            tb3_4_completed_tasks.data.append(completed_task)
            print("tasks completed by tb3_4 = ", tb3_4_completed_tasks.data)


rospy.init_node('goal_pose')
name = rospy.get_name()
odom_msg = rospy.wait_for_message(name + '/odom', Odometry)

navclient = actionlib.SimpleActionClient(name + '/move_base', MoveBaseAction)
navclient.wait_for_server()
#node_names = rosnode.get_node_names()
turtlebot_names = ["/tb3_0", "/tb3_1", "/tb3_2"]

for i in range (len(turtlebot_names)):
    #print(turtlebot_names[i])
    if turtlebot_names[i] not in name:
        rospy.Subscriber(turtlebot_names[i], Float32MultiArray, callback)
        print("Subscribed to", turtlebot_names[i])

no_of_tasks = 5

pub = rospy.Publisher(name, Float32MultiArray, queue_size = 10)

tasks = Float32MultiArray()

tb3_0_completed_tasks = Float32MultiArray()

tb3_1_completed_tasks = Float32MultiArray()

tb3_2_completed_tasks = Float32MultiArray()

tb3_3_completed_tasks = Float32MultiArray()

tb3_4_completed_tasks = Float32MultiArray()

def check_range(robot_number, failed_task_index):
    failed_task = Float32MultiArray()
    failed_task.data = [tasks.data[int(failed_task_index)][0], tasks.data[int(failed_task_index)][1], tasks.data[int(failed_task_index)][2]]
    print(failed_task.data)
    failed_task.data.insert(0, 2)
    print(failed_task.data)
    failed_task.data.append(robot_number)
    print(failed_task.data)
    pub.publish(failed_task)

def responder(failed_task_index):
    pos = Float32MultiArray()
    name_to_float = turtlebot_names.index(name)
    print("name to float: ", name_to_float)
    pos.data = [1, name_to_float, failed_task_index]
    pub.publish(pos)

def do_task(tasks):
    
    print("len tasks = ", len(tasks.data))
    i = 0

    while i < (len(tasks.data)):

        print("\ntask", i+1, ": task_x:", tasks.data[i][0], "task_y:", tasks.data[i][1], "hover_duration:", tasks.data[i][2])

        if (tasks.data[i] in tb3_0_completed_tasks.data):
            print(name + "Skipped task, since it was already completed by tb_0")
            i += 1
            continue
        if (tasks.data[i] in tb3_1_completed_tasks.data):
            print(name + "Skipped task, since it was already completed by tb_1")
            i += 1
            continue
        if (tasks.data[i] in tb3_2_completed_tasks.data):
            print(name + "Skipped task, since it was already completed by tb_2")
            i += 1
            continue
        if (tasks.data[i] in tb3_3_completed_tasks.data):
            print(name + "Skipped task, since it was already completed by tb_3")
            i += 1
            continue
        if (tasks.data[i] in tb3_4_completed_tasks.data):
            print(name + "Skipped task, since it was already completed by tb_4")
            i += 1
            continue

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
            name_to_float = turtlebot_names.index(name)
            finished_task = Float32MultiArray()
            finished_task.data = [3, tasks.data[i][0], tasks.data[i][1], tasks.data[i][2], name_to_float]
            pub.publish(finished_task)
        
        print("The goal of tasks ", i+1, " was:    task_x:", tasks.data[i][0], "task_y:", tasks.data[i][1], "hover_duration:", tasks.data[i][2])

        rospy.sleep(tasks.data[i][2])
        i += 1


def task_list(tasks_seed):
    
    random.seed(str(tasks_seed) + name)

    for i in range (no_of_tasks):
        task_x = round(random.uniform(0.5, 9.5), 1)
        task_y = round(random.uniform(0.5, 9.5), 1)
        hover_duration = random.randint(3, 8)
        
        tasks.data.append([task_x, task_y, hover_duration])

    do_task(tasks)

    print(name + " is done")
    print("tb_0 completed", len(tb3_0_completed_tasks.data), "tasks:")
    print(tb3_0_completed_tasks.data)

    print("tb_1 completed", len(tb3_1_completed_tasks.data), "tasks:")
    print(tb3_1_completed_tasks.data)
    
    print("tb_2 completed", len(tb3_2_completed_tasks.data), "tasks:")
    print(tb3_2_completed_tasks.data)

    print("tb_3 completed", len(tb3_3_completed_tasks.data), "tasks:")
    print(tb3_3_completed_tasks.data)
    
    print("tb_4 completed", len(tb3_4_completed_tasks.data), "tasks:")
    print(tb3_4_completed_tasks.data)

task_list(1)