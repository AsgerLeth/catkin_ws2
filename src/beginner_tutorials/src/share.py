#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_py
import math
from sensor_msgs.msg import LaserScan
import numpy as np
from tf.transformations import quaternion_matrix

def transform_scan(scan_msg, robot_id):
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    source_frame = f"{robot_id}/base_scan"
    target_frame = "map"

    tf_buffer.can_transform(target_frame, source_frame, rospy.Time(0), timeout=rospy.Duration(3.0))
    transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))

    transformed_scan = LaserScan()
    transformed_scan.header = scan_msg.header
    transformed_scan.header.frame_id = target_frame
    transformed_scan.angle_min = scan_msg.angle_min
    transformed_scan.angle_max = scan_msg.angle_max
    transformed_scan.angle_increment = scan_msg.angle_increment
    transformed_scan.time_increment = scan_msg.time_increment
    transformed_scan.scan_time = scan_msg.scan_time
    transformed_scan.range_min = scan_msg.range_min
    transformed_scan.range_max = scan_msg.range_max

    transform_matrix = quaternion_matrix([transform.transform.rotation.x,
                                          transform.transform.rotation.y,
                                          transform.transform.rotation.z,
                                          transform.transform.rotation.w])
    transform_matrix[:3, 3] = [transform.transform.translation.x,
                               transform.transform.translation.y,
                               transform.transform.translation.z]

    for i in range(len(scan_msg.ranges)):
        angle = scan_msg.angle_min + i * scan_msg.angle_increment
        range_ = scan_msg.ranges[i]
        # Ignore invalid range values
        if math.isnan(range_) or math.isinf(range_):
            continue



        source_point = [range_ * math.cos(angle), range_ * math.sin(angle), 0.0, 1.0]
        target_point = transform_matrix @ source_point

        transformed_angle = math.atan2(target_point[1], target_point[0])
        transformed_range = math.sqrt(target_point[0] ** 2 + target_point[1] ** 2)

        transformed_scan.ranges.append(transformed_range)

    return transformed_scan

def scan_callback(scan_msg, robot_id):
    transformed_scan = transform_scan(scan_msg, robot_id)
    scan_pub.publish(transformed_scan)

rospy.init_node('sensor_data_sharing')
scan_pub = rospy.Publisher('/shared_scans', LaserScan, queue_size=10)

robot_ids = ['tb3_0', 'tb3_1', 'tb3_2']
for robot_id in robot_ids:
    rospy.Subscriber(f'{robot_id}/scan', LaserScan, scan_callback, robot_id)

rospy.spin()
