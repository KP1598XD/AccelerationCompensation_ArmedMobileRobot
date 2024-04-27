#!/usr/bin/env python3

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def get_user_input(prompt, min_val, max_val):
    # Function to get user input within a specified range
    while True:
        try:
            value = float(input(prompt))
            if min_val <= value <= max_val:
                return value
            else:
                print(f"Error: value must be between {min_val} and {max_val}.")
        except ValueError:
            print("Please, introduce a valid number")


def send_initial_position(Joint1):
    # Send initial position command to the robot controller
    rospy.init_node('initial_pose_2')
    
    # Initialize publisher
    pub = rospy.Publisher('/arm_robot_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Wait for publisher to initialize

    # Create JointTrajectory message
    trajectory = JointTrajectory()
    trajectory.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
    
    # Define trajectory point with specified Joint1 position
    point = JointTrajectoryPoint()
    point.positions = [math.radians(Joint1), -0.43482197643969117, -1.2531849125741297, -3.7026623672755852e-06, 1.4535931104459971, 0.04267949895626001]
    rospy.set_param('/Initial_joints_positions', point.positions)
    
    # Set time from start for trajectory point
    point.time_from_start = rospy.Duration(1)
    
    # Add trajectory point to trajectory
    trajectory.points.append(point)

    # Publish trajectory
    pub.publish(trajectory)
    rospy.sleep(1)  # Wait for message to be published

if __name__ == '__main__':
    try:
        # Get user input for Joint1 position
        Joint1 = get_user_input("Introduce position for Joint1 [-180º, 180º]: ", -180, 180)
        # Set Joint1 orientation parameter
        rospy.set_param('/Joint1_orientation', Joint1)
        # Send initial position command
        send_initial_position(Joint1)
    except rospy.ROSInterruptException:
        pass

