#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import tf_conversions
import csv
import datetime
import os
import rospkg
import numpy as np


class RecordInputAndSensorData:
    def __init__(self, car_number):

        self.folder_name = '/home/domenico/NOISE_PATH' # Percorso assoluto
        base_path = self.folder_name


        self.car_number = car_number
        self.file_name = 'car_' + str(car_number) + '_Data'

        # Initialize the ROS node
        rospy.init_node('data_recording' + str(car_number), anonymous=True)

        # Initialize variables
        self.sensors_and_input_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vicon_state = [0.0, 0.0, 0.0, 0.0]

        # Find the full path and create the folder if it doesn't exist
        self.rospack = rospkg.RosPack()
        # base_path = self.rospack.get_path('racecar_pkg') + self.folder_name
        self.ensure_directory_exists(base_path)

        # Create the file
        date_time = datetime.datetime.now()
        date_time_str = date_time.strftime("%m_%d_%Y_%H_%M_%S")
        file_name = 'dritto_2.5'
        # file_name = base_path + self.file_name + 'recording_' + date_time_str + '.csv'
        file = open(file_name, 'w+')
        print(file_name)

        # Write the header
        self.writer = csv.writer(file)
        self.writer.writerow(['elapsed time sensors', 'current', 'voltage', 'acc x (IMU)', 'acc y (IMU)', 'W (IMU)',
                              'vel encoder', 'safety_value', 'throttle', 'steering', 'vicon time', 'vicon x', 
                              'vicon y', 'vicon yaw'])

        # ROS topic subscriptions
        rospy.Subscriber('sensors_and_input_' + str(car_number), Float32MultiArray, self.callback_sensors_and_input)
        rospy.Subscriber('/vicon/jetracer' + str(car_number), PoseStamped, self.odom_callback)

        rospy.spin()

    def ensure_directory_exists(self, path):
        """Create the directory if it doesn't exist"""
        if not os.path.exists(path):
            os.makedirs(path)
            rospy.loginfo(f"Directory {path} created.")
        else:
            rospy.loginfo(f"Directory {path} already exists.")

    def callback_sensors_and_input(self, sensors_and_input_data):
        """Callback for sensors and inputs"""
        self.sensors_and_input_data = np.array(sensors_and_input_data.data)

    def odom_callback(self, msg):
        """Callback per il Vicon topic"""
        self.pos_x = msg.pose.position.x 
        self.pos_y = msg.pose.position.y
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y, 
                    msg.pose.orientation.z, msg.pose.orientation.w])
        self.zyx_euler_angles = tf_conversions.transformations.euler_from_quaternion(q, 'rzyx')
        self.yaw = self.zyx_euler_angles[0]

        time_now = msg.header.stamp.to_sec()
        self.vicon_state = [time_now, self.pos_x, self.pos_y, self.yaw]
        data_line = [*self.sensors_and_input_data, *self.vicon_state]
        self.writer.writerow(data_line)


if __name__ == '__main__':
    try:
        car_number = 2
        recording = RecordInputAndSensorData(car_number)

    except rospy.ROSInterruptException:
        print('Something went wrong with the topic configuration.')
