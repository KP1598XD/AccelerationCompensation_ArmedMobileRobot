#!/usr/bin/env python3
import rospy
import os
import shutil

def find_lowest_available_index(directory, prefix):
    """ Finds the lowest available index for a new directory with a specified prefix """
    max_index = 0
    existing_directories = [d for d in os.listdir(directory) if os.path.isdir(os.path.join(directory, d)) and d.startswith(prefix)]
    existing_indices = sorted([int(d[len(prefix):]) for d in existing_directories if d[len(prefix):].isdigit()])

    if existing_indices:
        last_index = existing_indices[-1]
        for index in range(1, last_index):
            if index not in existing_indices:
                return index
        return last_index + 1
    else:
        return 1

def ensure_directory_exists(directory):
    """ Ensure that the directory exists, create it if it does not """
    if not os.path.exists(directory):
        os.makedirs(directory)
        print(f"Directory created: {directory}")
    else:
        print(f"Directory already exists: {directory}")

def list_files(directory):
    """ Lists all files in the specified directory """
    try:
        files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
        if not files:
            print("No files found in the directory.")
            return None
        else:
            return files
    except FileNotFoundError:
        print("The specified directory does not exist.")
        return None

def select_file(files, directory):
    """ Allows the user to select a file from the list """
    for idx, file in enumerate(files, start=1):
        print(f"{idx}. {file}")

    while True:
        try:
            choice = int(input("Enter the number of the file you want to use: "))
            if 1 <= choice <= len(files):
                return os.path.join(directory, files[choice - 1])
            else:
                print("Invalid selection. Please choose a valid file number.")
        except ValueError:
            print("Please enter a valid integer.")
            
def trim_path_for_output(full_path):
    parts = full_path.split(os.sep)
    if 'robot_model' in parts:
        return os.sep.join(parts[parts.index('robot_model'):])
    return full_path

def main():
    rospy.init_node('file_selector_node')

    # Define the directory using ROS_PACKAGE_PATH
    base_path = os.getenv('ROS_PACKAGE_PATH').split(':')[0]
    data_directory = os.path.join(base_path, 'robot_model', 'data')
    imu_data_directory = os.path.join(data_directory, 'imu_data')

    # List files in the IMU data directory
    files = list_files(imu_data_directory)
    if files:
        # Allow the user to select a file
        selected_file = select_file(files, imu_data_directory)
        if selected_file:
            # Find the lowest available index for a new test folder
            index = find_lowest_available_index(data_directory, 'test_')
            test_folder_name = f'test_{index}'
            test_directory = os.path.join(data_directory, test_folder_name)
            ensure_directory_exists(test_directory)
            
            # Copy the selected file to the new test folder
            destination_file = os.path.join(test_directory, os.path.basename(selected_file))
            shutil.copy(selected_file, destination_file)
            destination_file = trim_path_for_output(destination_file)
            
            # Set ROS parameters for the selected file and test folder name
            rospy.set_param('/test_file_name', os.path.basename(selected_file))
            rospy.set_param('/test_folder_name', test_folder_name)
            print(f"Selected file '{os.path.basename(selected_file)}' copied to '{trim_path_for_output(destination_file)}'. Saved as ROS parameter '/test_file_name'.")
            print(f"File and folder names saved as ROS parameters '/test_file_name' and '/test_folder_name'.")
        else:
            print("No valid file selected.")
    else:
        print("No files available for selection.")

if __name__ == '__main__':
    main()

