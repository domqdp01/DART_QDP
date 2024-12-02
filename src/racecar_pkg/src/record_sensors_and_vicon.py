#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_conversions
import csv
import datetime
import os
import rospkg
import numpy as np

class record_input_and_sensor_data:
    def __init__(self, car_number):

        self.folder_name = '/src/Data/NEW_FOLDER/'

        self.car_number = car_number
        self.file_name = 'car_' + str(car_number) + '_Data'

        # Inizializza il nodo ROS
        rospy.init_node('data_recording' + str(car_number), anonymous=True)

        # Inizializza variabili
        self.sensors_and_input_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vicon_state = [0.0, 0.0, 0.0, 0.0]

        # Trova il percorso completo e crea la cartella, se necessario
        self.rospack = rospkg.RosPack()
        base_path = self.rospack.get_path('racecar_pkg') + self.folder_name
        self.ensure_directory_exists(base_path)

        # Crea il file
        date_time = datetime.datetime.now()
        date_time_str = date_time.strftime("%m_%d_%Y_%H_%M_%S")
        file_name = base_path + self.file_name + 'recording_' + date_time_str + '.csv'
        file = open(file_name, 'w+')
        print(file_name)

        # Scrivi l'intestazione
        self.writer = csv.writer(file)
        self.writer.writerow(['elapsed time sensors', 'current', 'voltage', 'acc x (IMU)', 'acc y (IMU)', 'W (IMU)',
                              'vel encoder', 'safety_value', 'throttle', 'steering', 'vicon time', 'vicon x', 
                              'vicon y', 'vicon yaw'])

        # Sottoscrizioni ai topic ROS
        rospy.Subscriber('sensors_and_input_' + str(car_number), Float32MultiArray, self.callback_sensors_and_input)
        rospy.Subscriber('/vicon/jetracer' + str(car_number), PoseWithCovarianceStamped, self.odom_callback)

        rospy.spin()

    def ensure_directory_exists(self, path):
        """Crea la directory se non esiste"""
        if not os.path.exists(path):
            os.makedirs(path)
            rospy.loginfo(f"Directory {path} creata.")
        else:
            rospy.loginfo(f"Directory {path} già esistente.")

    def callback_sensors_and_input(self, sensors_and_input_data):
        """Callback per i sensori e gli input"""
        self.sensors_and_input_data = np.array(sensors_and_input_data.data)

    def odom_callback(self, msg):
        """Callback per il topic del Vicon"""
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.zyx_euler_angles = tf_conversions.transformations.euler_from_quaternion(q, 'rzyx')
        self.yaw = self.zyx_euler_angles[0]

        time_now = msg.header.stamp.to_sec()
        self.vicon_state = [time_now, msg.pose.pose.position.x, msg.pose.pose.position.y, self.zyx_euler_angles[0]]
        data_line = [*self.sensors_and_input_data, *self.vicon_state]
        self.writer.writerow(data_line)


if __name__ == '__main__':
    try:
        car_number = 1
        recording = record_input_and_sensor_data(car_number)

    except rospy.ROSInterruptException:
        print('Qualcosa è andato storto con la configurazione dei topic.')
