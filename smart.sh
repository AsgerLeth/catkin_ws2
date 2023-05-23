#!/bin/bash

# declare an array with the turtlebots names
declare -a turtlebots=("tb3_0" "tb3_1" "tb3_2" "tb3_3" "tb3_4")

# loop through the array and open a new terminal for each turtlebot
for bot in "${turtlebots[@]}"
do
  gnome-terminal --tab --title="$bot" -- bash -c "rosrun beginner_tutorials init_pose.py __name:=$bot && rosrun beginner_tutorials smart_alloc_test.py __name:=$bot; exec bash"
done

