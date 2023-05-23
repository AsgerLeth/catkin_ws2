#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import random
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import rosnode
import csv
import time
import math

# Callbacks definition

def active_cb():
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    a = 1
    #rospy.loginfo("Current location: " + str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
        
        #Publishing the completed task to other robots
        finished_task = Float32MultiArray(data = [3, tasks.data[task_index][0], tasks.data[task_index][1], tasks.data[task_index][2], turtlebot_names.index(name)])
        pub.publish(finished_task)

        #Saving the completed task in own storage
        completed_tasks.data[turtlebot_names.index(name)].append(tasks.data[task_index])

    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")
        print("name, aborted", name)
        print("robot_number, aborted", turtlebot_names.index(name))
        task_failed = Float32MultiArray(data = [0, turtlebot_names.index(name), task_index])
        pub.publish(task_failed)
        impossible_tasks.data.append(tasks.data[task_index])

def callback(message: Float32MultiArray):
    if (message.data[0] == 0):
        responder(message.data[1], message.data[2])
    
    elif (message.data[0] == 1):
        check_range(message.data[1], message.data[2])

    elif (message.data[0] == 2):
        put_task_in_list(round(message.data[1], 2), round(message.data[2], 2), int(message.data[3]))

    elif (message.data[0] == 3):
        finished_task = Float32MultiArray(data = [round(message.data[1], 2), round(message.data[2], 2), int(message.data[3])])
        if finished_task.data not in completed_tasks.data[int(message.data[4])]:
            completed_tasks.data[int(message.data[4])].append(finished_task.data)
            print("tasks completed by", turtlebot_names[int(message.data[4])], "= ", completed_tasks.data[int(message.data[4])])

    elif (int(message.data[0]) == 4):
        update_ongoing_tasks(round(message.data[1], 2), round(message.data[2], 2), int(message.data[3]), int(message.data[4]))
    
    elif (message.data[0] == 5):
        done.data.append(message.data[1])

rospy.init_node('goal_pose')
name = rospy.get_name()
odom_msg = rospy.wait_for_message(name + '/odom', Odometry)

navclient = actionlib.SimpleActionClient(name + '/move_base', MoveBaseAction)
navclient.wait_for_server()
#node_names = rosnode.get_node_names()
turtlebot_names = ["/tb3_0", "/tb3_1", "/tb3_2", "/tb3_3", "/tb3_4"]

no_of_tasks = 5

task_index = -1

pub = rospy.Publisher(name, Float32MultiArray, queue_size = 10)

tasks = Float32MultiArray()

impossible_tasks = Float32MultiArray()

ongoing_tasks = Float32MultiArray(data = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])

completed_tasks = Float32MultiArray()
completed_tasks.data = [[], [], [], [], []]

print(completed_tasks)
print(turtlebot_names.index(name))
print(completed_tasks.data[turtlebot_names.index(name)])
print(len(completed_tasks.data[turtlebot_names.index(name)]))

done = Float32MultiArray()

def update_ongoing_tasks(x, y, h, robot_number):
    ongoing_tasks.data[robot_number] = [x, y, h]

def put_task_in_list(x, y, h):
    task = Float32MultiArray(data = [x, y, h])
    not_in_list = True
    for i in range(len(tasks.data)):
        if (task.data == tasks.data[i]):
            not_in_list = False
    if not_in_list == True:
        #print(name)
        print("Putting task into list")
        tasks.data.append(task.data)
    
def check_range(robot_number, failed_task_index):
    if turtlebot_names.index(name) == robot_number:
        #print(name)
        print("Failed task index =", failed_task_index)
        print("Task list length =", len(tasks.data))
        failed_task = Float32MultiArray(data = [2, tasks.data[int(failed_task_index)][0], tasks.data[int(failed_task_index)][1], tasks.data[int(failed_task_index)][2]])
        pub.publish(failed_task)

def responder(robot_number, failed_task_index):
    pos = Float32MultiArray(data = [1, robot_number, failed_task_index])
    pub.publish(pos)

def log_to_csv(run_time):
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

    # Write the run number and time to the log file
    with open('log.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([run_number, run_time])

def closest_task():
    min_dist = 13.0
    global task_index
    print(len(tasks.data))
    skip = False

    odom_msg = rospy.wait_for_message(name + '/odom', Odometry)
    x_start = odom_msg.pose.pose.position.x
    y_start = odom_msg.pose.pose.position.y
    print("Starting position:", x_start, y_start)

    for n in range(len(tasks.data)):
        if tasks.data[n] in impossible_tasks.data:
            print(name + " skipped task, since it was impossible")
            continue
        
        skip_task = False
        for i in range(len(turtlebot_names)):
            for j in range(1, len(completed_tasks.data[i])):
                if (turtlebot_names.index(name) != i):
                    if (math.dist([tasks.data[n][0], tasks.data[n][1]], [completed_tasks.data[i][j][0], completed_tasks.data[i][j][1]]) < 2):
                        skip_task = True
                else:
                    if (tasks.data[n] == completed_tasks.data[i][j]):
                        skip_task = True
        if (skip_task):
            continue

        if(tasks.data[n] in ongoing_tasks.data):
            print("Another robot is doing that task!")
            continue

        distance = math.dist([x_start, y_start], [tasks.data[n][0], tasks.data[n][1]])
        print("Distance:", distance)
        
        if(distance < min_dist):
            min_dist = distance
            min_dist_index = n
        
    if(min_dist == 13.0):
        skip = True
        return min_dist, skip
    
    task_index = min_dist_index

    ongoing_task = Float32MultiArray(data = [4, tasks.data[task_index][0], tasks.data[task_index][1], tasks.data[task_index][2], turtlebot_names.index(name)])
    ongoing_tasks.data[turtlebot_names.index(name)] = [tasks.data[task_index][0], tasks.data[task_index][1], tasks.data[task_index][2]]
    pub.publish(ongoing_task)

    return min_dist, skip

def do_tasks():
    
    print("len tasks = ", no_of_tasks)
    i = 0

    while i < (len(tasks.data)):

        rospy.sleep(random.uniform(0.0, 1.0))

        min_dist, skip = closest_task()

        if(skip):
            break

        print("\nThe", i+1, "th task is task", task_index,
              ": task_x:", tasks.data[task_index][0],
              "task_y:", tasks.data[task_index][1],
              "hover_duration:", tasks.data[task_index][2])

        # print("x_start =", x_start, "y_start =", y_start)

        # Example of navigation goal
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.seq = 0

        goal.target_pose.pose.position.x = tasks.data[task_index][0]
        goal.target_pose.pose.position.y = tasks.data[task_index][1]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        navclient.send_goal(goal, done_cb, active_cb, feedback_cb)

        print("Distance =", min_dist)

        max_task_duration = min_dist * 10 + 15
        print("Max task duration", max_task_duration)

        finished = True
        state = navclient.get_state()
        print("state:", state)


        print("ongoing tasks:", ongoing_tasks.data)

        finished = navclient.wait_for_result(rospy.Duration(max_task_duration))

        print("finished:",finished)

        if not finished:
            rospy.logerr("Can't reach goal")
            #pub.publish(name + " can't reach the goal")
            print("name", name)
            print("robot_number", turtlebot_names.index(name))
            task_failed = Float32MultiArray(data = [0, turtlebot_names.index(name), task_index])
            pub.publish(task_failed)
            impossible_tasks.data.append(tasks.data[task_index])

        else:
            rospy.loginfo(navclient.get_result())
            
        
        print("The goal of the", i+1, "th task, task", task_index,
              ", was:    task_x:", tasks.data[task_index][0],
              "task_y:", tasks.data[task_index][1],
              "hover_duration:", tasks.data[task_index][2])

        rospy.sleep(tasks.data[task_index][2])
        i += 1

def generate_spawn_positions(num_positions, allowed_areas, seed):
    if seed is not None:
        random.seed(str(seed) + name)

    for _ in range(num_positions):
        allowed_area = random.choice(allowed_areas)
        x_range, y_range = allowed_area

        x = round(random.uniform(x_range[0], x_range[1]), 2)
        y = round(random.uniform(y_range[0], y_range[1]), 2)
        hover_duration = random.randint(1, 5)

        tasks.data.append([x, y, hover_duration])

# Function to generate a random float between min_val and max_val
def random_float(min_val, max_val):
    return random.uniform(min_val, max_val)

def task_list(tasks_seed):
    start_time = time.time()

    for i in range (len(turtlebot_names)):
        if turtlebot_names[i] not in name:
            rospy.Subscriber(turtlebot_names[i], Float32MultiArray, callback)
            print("Subscribed to", turtlebot_names[i])
    

    allowed_areas_Middle_wall = [
        ((  0,  10), (  0, 4.7)),
        ((  0,  10), (5.3,  10))]

    allowed_areas_Blocks = [
        ((  0, 1.3), (  0,  10)),
        ((1.3,  10), (8.8,  10)),
        ((8.7,  10), (1.2,  10)),
        ((2.6, 8.7), (  0, 1.8)),
        ((1.3, 2.5), (  0, 1.2)),
        ((1.3, 8.7), (5.7, 7.3)),
        ((4.5, 6.1), (7.3, 8.8)),
        ((6.2, 8.8), (3.2, 5.7)),
        ((2.6, 4.8), (1.8, 5.7)),
        ((4.8,   7), (1.8, 4.1))]

    allowed_areas_Maze = [
        ((  0, 1.5), (  0,  10)),
        ((  2, 3.7), (  0, 6.2)),
        ((  2, 3.7), (6.9,  10)),
        ((3.7,  10), (  0, 1.8)),
        ((3.7, 7.9), (2.1, 4.2)),
        ((  4, 5.7), (4.9,  10)),
        ((6.3, 7.9), (2.3,  10)),
        ((8.4,  10), (2.3,  10))]

    generate_spawn_positions(no_of_tasks, allowed_areas_Middle_wall, tasks_seed)
    do_tasks()

    print(name, "is done")
    done.data.append(turtlebot_names.index(name))
    done_message = Float32MultiArray(data = [5, turtlebot_names.index(name)])
    pub.publish(done_message)

    ongoing_task = Float32MultiArray(data = [4, 0, 0, 0, turtlebot_names.index(name)])
    ongoing_tasks.data[turtlebot_names.index(name)] = [0, 0, 0]
    print("ongoing tasks:", ongoing_tasks.data)
    pub.publish(ongoing_task)
    
    while len(done.data) < 5:
        do_tasks()
        print("In loop")
        print("done:", done.data)
        
    print(name, " tasks were: ", tasks.data)
    for i in range(len(completed_tasks.data)):
        print(turtlebot_names[i], "completed", len(completed_tasks.data[i]), "tasks:", completed_tasks.data[i])

    end_time = time.time()
    run_time = round(end_time - start_time, 2)
    print("Run completed in", run_time, "seconds")
    log_to_csv(run_time)


task_list(1)