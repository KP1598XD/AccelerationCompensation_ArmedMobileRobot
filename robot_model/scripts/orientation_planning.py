#!/usr/bin/env python3
import rospy
import csv
import math
import os

def calculate_rotation(acc_x, Joint1_angle):
    # Function to calculate the rotation of the accelerometer reading based on the angle of Joint1
    angle_rad = math.radians(Joint1_angle)
    cos_angle = math.cos(angle_rad)
    acc_x_rotated = acc_x * cos_angle
    return acc_x_rotated

def calculate_inclination(acc_x, g=9.81):
    # Function to calculate the inclination angle from the rotated accelerometer reading
    angle_x = math.atan2(acc_x, g)
    degrees_x = math.degrees(angle_x)
    return angle_x, degrees_x
    
def get_valid_window_size():
    # Function to get a valid window size for the moving average filter
    while True:
        try:
            window_size = int(input("Please, introduce the window size for the moving average filter: "))
            if window_size > 0:
                return window_size
            else:
                print("Error: The parameter must be an integer positive number.")
        except ValueError:
            print("Error: Please, introduce an integer number.")

def shift_backward(values, shift_amount):
    # Function to shift values backward in a list
    return values[shift_amount:]

def moving_average_filter(values, window_size):
    # Function to apply a moving average filter to a list of values
    filtered_values = []
    for i in range(len(values)):
        if i < window_size - 1:
            filtered_values.append(values[i])
        else:
            window = values[i - window_size + 1:i + 1]
            filtered_average = sum(window) / window_size
            filtered_values.append(filtered_average)
    return filtered_values
    
def prepend_rows_with_zeros(timestamps, acc_x_values, window_size):
    # Function to prepend rows with zero values to ensure a consistent window for the moving average filter
    if timestamps:
        initial_timestamp = float(timestamps[0])
        sampling_interval = 0.1
        for i in range(1, window_size + 1):
            new_timestamp = initial_timestamp - (i * sampling_interval)
            timestamps.insert(0, f"{new_timestamp:.3f}")
            acc_x_values.insert(0, 0.0)

def process_imu_data(input_filename, output_dir, Joint1_angle, window_size):
    # Function to process IMU data, calculate inclinations, and save results to CSV files
    input_path = os.path.join(output_dir, input_filename)
    output_path_filtered = os.path.join(output_dir, 'filtered_calculated_inclinations.csv')
    output_path_unfiltered = os.path.join(output_dir, 'calculated_inclinations.csv')
    output_path_shifted = os.path.join(output_dir, 'shifted_filtered_calculated_inclinations.csv')

    with open(input_path, 'r') as infile, \
         open(output_path_filtered, 'w', newline='') as outfile_filtered, \
         open(output_path_unfiltered, 'w', newline='') as outfile_unfiltered, \
         open(output_path_shifted, 'w', newline='') as outfile_shifted:

        reader = csv.reader(infile)
        writer_filtered = csv.writer(outfile_filtered)
        writer_unfiltered = csv.writer(outfile_unfiltered)
        writer_shifted = csv.writer(outfile_shifted)

        headers = ['TimeStamp', 'Inclination TCP (rad)', 'Inclination TCP (deg)']
        writer_filtered.writerow(headers)
        writer_unfiltered.writerow(headers)
        writer_shifted.writerow(headers)

        next(reader)  # Skip the header row

        timestamps = []
        acc_x_values = []
        for row in reader:
            timestamps.append(row[0])
            acc_x_values.append(float(row[1]))

        prepend_rows_with_zeros(timestamps, acc_x_values, window_size)

        filtered_acc_x_values = moving_average_filter(acc_x_values, window_size)
        shift_amount = window_size // 2
        shifted_acc_x_values = shift_backward(filtered_acc_x_values, shift_amount)
        shifted_acc_x_values.extend([0.0] * shift_amount)

        for i in range(len(timestamps)):
            timestamp = timestamps[i]
            acc_x = acc_x_values[i]
            acc_x_rotated = calculate_rotation(acc_x, Joint1_angle)
            angle_x, degrees_x = calculate_inclination(acc_x_rotated)
            writer_unfiltered.writerow([timestamp, angle_x, degrees_x])

            if i < len(filtered_acc_x_values):
                acc_x_filtered = filtered_acc_x_values[i]
                acc_x_rotated_filtered = calculate_rotation(acc_x_filtered, Joint1_angle)
                angle_x_filtered, degrees_x_filtered = calculate_inclination(acc_x_rotated_filtered)
                writer_filtered.writerow([timestamp, angle_x_filtered, degrees_x_filtered])

            if i < len(shifted_acc_x_values):
                acc_x_shifted = shifted_acc_x_values[i]
                acc_x_rotated_shifted = calculate_rotation(acc_x_shifted, Joint1_angle)
                angle_x_shifted, degrees_x_shifted = calculate_inclination(acc_x_rotated_shifted)
                writer_shifted.writerow([timestamp, angle_x_shifted, degrees_x_shifted])

if __name__ == '__main__':
    rospy.init_node('inclination_calculator')
    
    # Get parameters from ROS
    Joint1_angle = rospy.get_param('/Joint1_orientation')
    test_file_name = rospy.get_param('/test_file_name')
    test_folder_name = rospy.get_param('/test_folder_name')
    base_dir = os.path.join(os.getenv('ROS_PACKAGE_PATH').split(':')[0], 'robot_model', 'data', test_folder_name)

    # Get window size for moving average filter
    window_size = get_valid_window_size()

    # Check if the input file exists
    if not os.path.exists(os.path.join(base_dir, test_file_name)):
        rospy.logerr(f"Input file does not exist: {trim_path_for_output(os.path.join(base_dir, test_file_name))}")
    else:
        # Process IMU data and save results
        process_imu_data(test_file_name, base_dir, Joint1_angle, window_size)
        rospy.loginfo("Inclination calculations are completed and saved to both filtered and unfiltered files.")

