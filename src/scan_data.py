#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import csv
import os

# Set file path
log_dir = os.path.expanduser('~/WS/SLAM_simulation/src/RB-Y1_SLAM_Simulation/result/')  
file_path = os.path.join(log_dir, 'scan_data.csv')

# Open the file once, outside the callback
file = open(file_path, mode='w')
writer = csv.writer(file)

def scan_callback(msg):
    writer.writerow(msg.ranges)

def shutdown_hook():
    rospy.loginfo("Shutting down scan logger, closing file.")
    file.close()

if __name__ == '__main__':
    rospy.init_node('scan_logger', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


