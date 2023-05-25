import os
import random
import subprocess
import rospy
import time
from std_msgs.msg import String

allowed_areas_Middle_wall = [
    ((0, 10), (0, 4.7)),
    ((0, 10), (5.3, 10)),
    # Add more allowed areas as needed
]

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
    ((4.8,   7), (1.8, 4.1)),
    # Add more allowed areas as needed
]
allowed_areas_Maze = [
    ((  0, 1.5), (  0,  10)),
    ((  2, 3.7), (  0, 6.2)),
    ((  2, 3.7), (6.9,  10)),
    ((3.7,  10), (  0, 1.8)),
    ((3.7, 7.9), (2.1, 4.2)),
    ((  4, 5.7), (4.9,  10)),
    ((6.3, 7.9), (2.3,  10)),
    ((8.4,  10), (2.3,  10)),
    # Add more allowed areas as needed
]

def generate_spawn_positions(num_positions, allowed_areas, seed):
    if seed is not None:
        random.seed(seed)

    spawn_positions = []
    for _ in range(num_positions):
        allowed_area = random.choice(allowed_areas)
        x_range, y_range = allowed_area

        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])

        spawn_positions.append((x, y))

    return spawn_positions

class TurtlebotController:
    def __init__(self):
        self.tasks_completed = {}
        self.num_seeds = 30
        self.current_seed = 0
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Subscribe to the 'tasks_finished' topic
        rospy.Subscriber('tasks_finished', String, self.tasks_finished_callback)

    def tasks_finished_callback(self, msg):
        msg_parts = msg.data.split(" ")
        turtlebot_name = msg_parts[0]
        seed = int(msg_parts[4])
        self.tasks_completed[turtlebot_name] = seed
        print(f"{turtlebot_name} done with seed {seed}")
        print("----------------------------------------------------------------------------------------------------len", len(self.tasks_completed))

        if len(self.tasks_completed) >= 5 and all(seed == self.current_seed for seed in self.tasks_completed.values()):
            time.sleep(30)
            self.restart()

    def restart(self):
        # Terminate the currently running processes
        self.terminate_processes()

        # Check if all seeds have been processed
        if self.current_seed < self.num_seeds:
            self.current_seed += 1
            time.sleep(2)
            self.run_seed(self.current_seed)

    def terminate_processes(self):
        # Terminate the 'roslaunch' processes
        subprocess.run(['pkill', '-f', 'roslaunch'])

        # Terminate the 'smart_alloc_test.py' processes
        subprocess.run(['pkill', '-f', 'smart_alloc_test.py'])
        time.sleep(30)

    def run_seed(self, seed):
        # Generate spawn positions for Middle_wall.launch
        spawn_positions = generate_spawn_positions(5, allowed_areas_Maze, seed)

        # Start the 'roslaunch' processes
        subprocess.Popen(['roslaunch', 'turtlebot3_navigation', 'multi_nav_bringup4.launch'])
        subprocess.Popen(['roslaunch', 'glocal_package', 'Maze.launch', 'spawn_positions:={}'.format(spawn_positions)])

        # Run the bash script to start the python code on each turtlebot
        current_dir = os.getcwd()
        script_path = os.path.join(current_dir, 'smart.sh')
        subprocess.run([script_path, str(seed)])

    def run(self):
        self.run_seed(self.current_seed)
        rospy.spin()

if __name__ == '__main__':
    controller = TurtlebotController()
    controller.run()
