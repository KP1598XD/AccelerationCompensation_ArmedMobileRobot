#!/usr/bin/env python3
import rospy
import csv
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
import os

class JointController():
    def __init__(self):
        # Initialize the JointController class
        test_folder_name = rospy.get_param('/test_folder_name')
        self.joint_positions = []  # List to store desired joint positions
        self.current_joint_positions = [0] * 6  # List to store current joint positions
        self.orientation_data = None  # Variable to store orientation data
        self.pub = rospy.Publisher('/arm_robot_controller/command', JointTrajectory, queue_size=10)  # Publisher for joint positions
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)  # Subscriber for current joint states
        rospy.Subscriber('/orientation_ee', Vector3, self.orientation_callback)  # Subscriber for end-effector orientation
        rospy.sleep(1)  # Wait for initialization

        # Create a CSV file to store real orientations
        self.csv_file = os.path.join(os.getenv('ROS_PACKAGE_PATH').split(':')[0], 'robot_model', 'data', test_folder_name, 'real_orientations.csv')
        with open(self.csv_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Roll', 'Pitch', 'Yaw'])

    def joint_state_callback(self, msg):
        # Callback function to update current joint positions
        self.current_joint_positions = msg.position

    def orientation_callback(self, msg):
        # Callback function to update orientation data
        self.orientation_data = (-msg.x, -msg.y, msg.z)

    def read_joint_positions(self, filename):
        # Function to read desired joint positions from a CSV file
        with open(filename, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header
            self.joint_positions = [(row[0], [float(pos) for pos in row[1:7]]) for row in reader]

    def save_orientation(self, timestamp):
        # Function to save orientation data to the CSV file
        with open(self.csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            if self.orientation_data:
                writer.writerow([timestamp] + [f"{x:.2f}" for x in self.orientation_data])

    def send_joint_positions(self):
        # Function to send desired joint positions
        rate = rospy.Rate(10)  # 10 Hz
        trajectory = JointTrajectory()
        trajectory.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        for timestamp, desired_positions in self.joint_positions:
            if not rospy.is_shutdown():
                trajectory.points = [
                    JointTrajectoryPoint(positions=desired_positions, time_from_start=rospy.Duration(0.1))
                ]
                self.pub.publish(trajectory)  # Publish joint positions
                rospy.loginfo(f"Sending joint positions: {desired_positions}")
                rospy.sleep(0.1)

                self.save_orientation(timestamp)  # Save orientation data
                rate.sleep()
                
def trim_path_for_output(full_path):
    # Function to trim the path for output
    parts = full_path.split(os.sep)
    if 'robot_model' in parts:
        return os.sep.join(parts[parts.index('robot_model'):])
    return full_path

def main():
    # Main function
    rospy.init_node('joint_trajectory_sender')  # Initialize ROS node
    test_folder_name = rospy.get_param('/test_folder_name')
    controller = JointController()  # Create instance of JointController class
    test_folder_name = rospy.get_param('/test_folder_name')
    package_path = os.path.join(os.getenv('ROS_PACKAGE_PATH').split(':')[0], 'robot_model', 'data', test_folder_name)
    filename = os.path.join(package_path, 'joint_positions.csv')  # Path to CSV file containing desired joint positions
    
    if not os.path.exists(filename):
        # Check if input file exists
        rospy.logerr(f"Input file does not exist: {trim_path_for_output(filename)}")
    else:
        rospy.loginfo(f"Joint positions file loaded")
        controller.read_joint_positions(filename)  # Read joint positions from CSV file
        controller.send_joint_positions()  # Send desired joint positions
        rospy.loginfo(f"Real orientations file saved: {trim_path_for_output(filename)} ")
    
if __name__ == '__main__':
    main()

