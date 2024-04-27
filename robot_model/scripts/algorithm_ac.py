#!/usr/bin/env python3

# Import necessary libraries
import rospy
import csv
import os
import math
import geometry_msgs.msg
import tf.transformations as transformations
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped

# Class to calculate joint positions
class JointPositionCalculator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('joint_position_calculator')
        
        # Initialize current pose variable
        self.current_pose = PoseStamped()
        
        # Subscribe to TCP pose topic
        rospy.Subscriber('/pose_tcp', PoseStamped, self.pose_callback)
        
        # Wait for IK service to become available
        rospy.wait_for_service('/compute_ik')
        self.compute_ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    # Callback function for TCP pose
    def pose_callback(self, msg):
        self.current_pose = msg

    # Function to calculate joint positions
    def calculate_joint_positions(self, input_file, output_file):
        # Open input and output CSV files
        with open(input_file, 'r') as infile, open(output_file, 'w', newline='') as outfile:
            reader = csv.reader(infile)
            writer = csv.writer(outfile)
            
            # Skip header row
            next(reader)
            
            # Write header row for output file
            writer.writerow(['TimeStamp', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
            
            # Initialize last joints with initial positions
            last_joints = rospy.get_param('/Initial_joints_positions')

            # Iterate over each row in input file
            for row in reader:
                timestamp = row[0]
                inclination_radians_x = math.radians(float(row[2]))
                
                # Create IK request
                ik_request = GetPositionIKRequest()
                ik_request.ik_request.group_name = "arm_robot"
                pose = geometry_msgs.msg.PoseStamped()
                pose.header.frame_id = "base_link"
                pose.pose.position.x = self.current_pose.pose.position.x
                pose.pose.position.y = self.current_pose.pose.position.y
                pose.pose.position.z = self.current_pose.pose.position.z

                # Calculate quaternion for orientation
                quaternion = transformations.quaternion_from_euler(math.radians(180), inclination_radians_x, 0.0)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                ik_request.ik_request.pose_stamped = pose

                # Call IK service
                response = self.compute_ik_service(ik_request)
                
                # Check if IK solution is successful
                if response.error_code.val == response.error_code.SUCCESS:
                    joints = response.solution.joint_state.position
                    writer.writerow([timestamp] + list(joints[:6]))
                    last_joints = joints
                else:
                    # Log warning if IK solution not found and use previous solution
                    rospy.logwarn("IK solution not found, using previous solution for timestamp: {}".format(timestamp))
                    writer.writerow([timestamp] + list(last_joints))

# Function to trim file path for output
def trim_path_for_output(full_path):
    parts = full_path.split(os.sep)
    if 'robot_model' in parts:
        return os.sep.join(parts[parts.index('robot_model'):])
    return full_path
    
# Main function
def main():
    # Initialize joint position calculator
    calculator = JointPositionCalculator()
    
    # Get test folder name from ROS parameter
    test_folder_name = rospy.get_param('/test_folder_name')
    
    # Set base directory path
    base_dir = os.path.join(os.getenv('ROS_PACKAGE_PATH').split(':')[0], 'robot_model', 'data', test_folder_name)
    
    # Set input and output file paths
    input_file = os.path.join(base_dir, 'shifted_filtered_calculated_inclinations.csv')
    output_file = os.path.join(base_dir, 'joint_positions.csv')
    
    # Check if input file exists
    if not os.path.exists(input_file):
        rospy.logerr(f"Input file does not exist: {trim_path_for_output(input_file)}")
    else:
        # Calculate joint positions and log info
        calculator.calculate_joint_positions(input_file, output_file)
        rospy.loginfo(f"Joint positions calculations are completed and saved to {trim_path_for_output(output_file)}")
        
# Execute main function if script is run directly
if __name__ == '__main__':
    main()

