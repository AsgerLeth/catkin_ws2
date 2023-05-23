import random
import subprocess
import os
import atexit
import signal
import psutil

def close_gazebo():
    for process in psutil.process_iter(['name', 'pid']):
        if process.info['name'] in ['gzserver', 'gzclient']:
            try:
                process.terminate()
                print(f"{process.info['name']} has been closed.")
            except psutil.NoSuchProcess:
                print(f"{process.info['name']} is not running.")

atexit.register(close_gazebo)

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


allowed_areas_Middle_wall = [
    ((  0,  10), (  0, 4.7)),
    ((  0,  10), (5.3,  10))]

allowed_areas_Blocks = [
    ((  0,  10), (  0, 1.3)),
    ((8.8,  10), (1.3,  10)),
    ((1.2,  10), (8.7,  10)),
    ((  0, 1.8), (2.6, 8.7)),
    ((  0, 1.2), (1.3, 2.5)),
    ((5.7, 7.3), (1.3, 8.7)),
    ((7.3, 8.8), (4.5, 6.1)),
    ((3.2, 5.7), (6.2, 8.8)),
    ((1.8, 5.7), (2.6, 4.8)),
    ((1.8, 4.1), (4.8,   7))]

allowed_areas_Maze = [
    ((  0,  10), (  0, 1.5)),
    ((  0, 6.2), (  2, 3.7)),
    ((6.9,  10), (  2, 3.7)),
    ((  0, 1.8), (3.7,  10)),
    ((2.1, 4.2), (3.7, 7.9)),
    ((4.9,  10), (  4, 5.7)),
    ((2.3,  10), (6.3, 7.9)),
    ((2.3,  10), (8.4,  10))]

# Function to generate a random float between min_val and max_val
def random_float(min_val, max_val):
    return random.uniform(min_val, max_val)

# Define the limits for the random spawn positions
x_min, x_max = 0.0, 10.0
y_min, y_max = 0.0, 10.0

# Generate random spawn positions for the 5 turtlebots
spawn_positions = generate_spawn_positions(5, allowed_areas_Middle_wall, seed=1)
turtlebot_names = ["first_tb3", "second_tb3", "third_tb3", "fourth_tb3", "fifth_tb3"]
# Create the roslaunch command with the random spawn positions
cmd = [
    "roslaunch",
    "glocal_package",
    "Middle_wall.launch",
]
nav_cmd = ["gnome-terminal", "--", "roslaunch", "turtlebot3_navigation", "multi_nav_bringup4.launch"]


for i, (x, y) in enumerate(spawn_positions):
    cmd += [f"{turtlebot_names[i]}_x_pos:={x}", f"{turtlebot_names[i]}_y_pos:={y}"]


subprocess.run(nav_cmd)

subprocess.run(cmd)



