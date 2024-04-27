#!/usr/bin/env python3

# Import necessary libraries
import rospy
import roslaunch
import time
from termcolor import colored

# Function to launch a ROS node from a given package and script name
def launch_node(package, script_name):
    # Define the node to be launched
    node = roslaunch.core.Node(package=package, node_type=script_name, output='screen')

    # Initialize ROS launch
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # Launch the node
    process = launch.launch(node)
    print(colored(f"--------------Starting {script_name} from package {package}... --------------", 'light_yellow'))

    # Wait for the node to finish executing
    while process.is_alive():
        if rospy.is_shutdown():
            process.stop()
            break
        time.sleep(1)

    print(colored(f"-------------- {script_name} has finished. --------------",'light_yellow'))

# Main function
if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('sequential_node_launcher', anonymous=True)

    try:
        print("Starting launch process...")

        # List of nodes to be launched sequentially
        nodes_to_launch = [
            ("robot_model", "initialization.py"),
            ("robot_model", "initial_pose_2.py"),
            ("robot_model", "orientation_planning.py"),
            ("robot_model", "algorithm_ac.py"),
            ("robot_model", "move_joints.py"),
            ("robot_model", "graphs_generator.py")
        ]

        # Launch each node in the list
        for package, script_name in nodes_to_launch:
            if rospy.is_shutdown():
                break
            launch_node(package, script_name)

        print("Launch process completed.")
    except rospy.ROSInterruptException:
        print("Launch process was interrupted.")

