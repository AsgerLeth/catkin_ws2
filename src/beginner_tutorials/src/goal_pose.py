#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
from std_msgs.msg import String
import rosnode
import csv
import time

# Callbacks definition
def log_to_csv():
    # Check if the log file exists, and if not, create it with header row
    try:
        with open('log.csv') as f:
            pass
    except FileNotFoundError:
        with open('log.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Run', 'Time (s)'])

    # Read the latest run number from the log file and increment it by one
    with open('log.csv', 'r') as f:
        reader = csv.reader(f)
        next(reader, None)  # Skip the header row
        runs = [int(row[0]) for row in reader]
        if runs:
            latest_run = max(runs)
        else:
            latest_run = 0
        run_number = latest_run + 1

    # Run the script and time it
    start_time = time.time()
    # ... Your script code goes here ...
    end_time = time.time()
    run_time = round(end_time - start_time, 2)

    # Write the run number and time to the log file
    with open('log.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([run_number, run_time])
    
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
        pub.publish(name + " can't reach the goal")
    

def callback(data):
    rospy.loginfo(name + " heard \"%s\"", data.data)



rospy.init_node('goal_pose')
name = rospy.get_name()

navclient = actionlib.SimpleActionClient(name + '/move_base', MoveBaseAction)
navclient.wait_for_server()
#node_names = rosnode.get_node_names()
turtlebot_names = ["tb3_0", "tb3_1", "tb3_2"]


for i in range (len(turtlebot_names)):
    #print(turtlebot_names[i])
    if turtlebot_names[i] not in name:
        rospy.Subscriber(turtlebot_names[i], String, callback)
        print("Subscribed to", turtlebot_names[i])


pub = rospy.Publisher(name, String, queue_size = 10)

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
    finished = navclient.wait_for_result(rospy.Duration(40))


    if not finished:
        rospy.logerr("Can't reach goal")
        pub.publish(name + " can't reach the goal")
    else:
        rospy.loginfo(navclient.get_result())
    
    print("The goal of task ", i, " was:    x_coordinate:", x_coordinate, "y_coordinate:", y_coordinate, "sleep_time:", sleep_time)
    
    
    rospy.sleep(sleep_time)

def task_list(tasks_seed):
    log_to_csv()
    random.seed(str(tasks_seed) + name)

    no_of_tasks = 4
    start_time = time.time()
    for i in range (1, no_of_tasks+1):
        x_coordinate = round(random.uniform(0.5, 9.5), 1)
        y_coordinate = round(random.uniform(0.5, 9.5), 1)
        sleep_time = random.randint(3, 8)
        
        current_task(i, x_coordinate, y_coordinate, sleep_time)
    end_time = time.time()
    run_time = round(end_time - start_time, 2)
    print("Run completed in", run_time, "seconds")
    log_to_csv()
    

# Call the log_to_csv() function to log the completion time and run number to a CSV file



task_list(1)