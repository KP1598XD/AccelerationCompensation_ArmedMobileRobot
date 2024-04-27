#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import csv
from geometry_msgs.msg import Twist
import os
from termcolor import colored

def get_file_name(base_path, extension='csv'):
    # Function to get a valid file name from the user
    while True:
        filename = input("Enter the desired file name: ")
        full_path = os.path.join(base_path, f"{filename}.{extension}")
        if not os.path.exists(full_path):
            return full_path
        print("File already exists. Please choose a different name.")

def get_positive_float(prompt):
    # Function to get a positive float from the user
    while True:
        try:
            value = float(input(prompt))
            if value <= 0:
                raise ValueError
            return value
        except ValueError:
            print("Please enter a positive number.")

class ImuDataCollector:
    def __init__(self):
        # Initialize the data collector
        warning_msg = "WARNING: Please ensure all data entered is accurate as it will directly influence robot movement and data collection (do not establish high accelerations/deceleration, because the robot can stumble down)"
        print(colored(warning_msg, 'light_yellow'))
        rospy.init_node('imu_data_collector', anonymous=True)
        self.base_path = os.path.join(os.getenv('ROS_PACKAGE_PATH').split(':')[0], 'robot_model', 'data', 'imu_data')
        if not os.path.exists(self.base_path):
            os.makedirs(self.base_path)
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.imu_data = []
        self.collecting_data = False

        # Set up movement parameters
        self.acceleration_rate = get_positive_float("Enter acceleration rate (m/s^2): ")
        self.deceleration_rate = get_positive_float("Enter deceleration rate (m/s^2): ")
        self.duration = get_positive_float("Enter duration of the movement (seconds): ")
        self.max_speed = get_positive_float("Enter maximum speed (m/s): ")

        self.collecting_data = True
        self.filename_imu = get_file_name(self.base_path, 'csv')  # Ask user for filename

    def imu_callback(self, data):
        # Callback function to collect IMU data
        if self.collecting_data:
            self.imu_data.append((rospy.get_time(), data.linear_acceleration.x))

    def execute_trajectory(self):
        # Execute the predefined trajectory
        rospy.sleep(2)  # Initial wait before starting motion
        speed = 0
        step_time = 0.1
        ramp_up_duration = int(self.max_speed / self.acceleration_rate / step_time)
        ramp_down_duration = int(self.max_speed / self.deceleration_rate / step_time)

        # Ramp up acceleration
        for _ in range(ramp_up_duration):
            speed += self.acceleration_rate * step_time
            self.publish_speed(speed)

        # Constant speed phase
        constant_speed_duration = self.duration - (ramp_up_duration + ramp_down_duration) * step_time
        for _ in range(int(constant_speed_duration / step_time)):
            self.publish_speed(self.max_speed)

        # Ramp down deceleration
        for _ in range(ramp_down_duration):
            speed -= self.deceleration_rate * step_time
            self.publish_speed(speed)

        self.publish_speed(0)  # Stop
        rospy.sleep(2)  # Final pause after stopping
        self.collecting_data = False
        self.save_data()

    def publish_speed(self, speed):
        # Publish speed to the robot
        move_cmd = Twist()
        move_cmd.linear.x = speed
        self.cmd_vel_publisher.publish(move_cmd)
        rospy.sleep(0.1)

    def save_data(self):
        # Save collected IMU data to a CSV file
        with open(self.filename_imu, 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['TimeStamps', 'Linear acceleration X'])
            for data in self.imu_data:
                writer.writerow([data[0], data[1]])
        print(f"Data saved to {self.filename_imu}")

if __name__ == '__main__':
    collector = ImuDataCollector()
    collector.execute_trajectory()

