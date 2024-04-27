#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_initial_position():
    # Initialize ROS node
    rospy.init_node('initial_pose_1')
    
    # Publisher for sending joint trajectory commands
    pub = rospy.Publisher('/arm_robot_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Wait for publisher to initialize

    # Create a JointTrajectory message
    trajectory = JointTrajectory()
    trajectory.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
    
    # Define the joint positions for the initial pose
    point = JointTrajectoryPoint()
    point.positions = [0.0, -0.43482197643969117, -1.2531849125741297, -3.7026623672755852e-06, 1.4535931104459971, 0.04267949895626001]
    
    # Set the time from start for the trajectory point
    point.time_from_start = rospy.Duration(1)
    
    # Append the trajectory point to the trajectory
    trajectory.points.append(point)

    # Publish the trajectory
    pub.publish(trajectory)
    rospy.sleep(1)  # Wait for the message to be published

if __name__ == '__main__':
    try:
        send_initial_position()
    except rospy.ROSInterruptException:
        pass

