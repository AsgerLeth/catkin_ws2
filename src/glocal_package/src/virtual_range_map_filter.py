#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

class VirtualRangeMapFilter:
    def __init__(self):
        # Define the virtual range, e.g., 5 meters
        self.virtual_range = 5.0

        # Dictionary to store map data and pose information for each TurtleBot
        self.robot_maps = {}

        # Subscribers and publishers for each TurtleBot
        number_of_turtlebots = 3
        for i in range(number_of_turtlebots):
            robot_name = f"tb3_{i}"
            rospy.Subscriber(f'/{robot_name}/map', OccupancyGrid, self.map_callback, robot_name)
            rospy.Subscriber(f'/{robot_name}/amcl_pose', PoseWithCovarianceStamped, self.pose_callback, robot_name)
            self.robot_maps[robot_name] = {'map_pub': rospy.Publisher(f'/{robot_name}/filtered_map', OccupancyGrid, queue_size=10)}

    def pose_callback(self, msg, robot_name):
        self.robot_maps[robot_name]['pose'] = msg.pose

    def map_callback(self, msg, robot_name):
        # Update map data
        self.robot_maps[robot_name]['map'] = msg

        # Filter maps based on the virtual range
        for robot, data in self.robot_maps.items():
            if robot != robot_name and 'pose' in data:
                distance = ((msg.info.origin.position.x - data['pose'].position.x) ** 2 + (msg.info.origin.position.y - data['pose'].position.y) ** 2) ** 0.5 * msg.info.resolution
                if distance <= self.virtual_range:
                    data['map_pub'].publish(msg)

if __name__ == '__main__':
    rospy.init_node('virtual_range_map_filter')
    vr_map_filter = VirtualRangeMapFilter()
    rospy.spin()
