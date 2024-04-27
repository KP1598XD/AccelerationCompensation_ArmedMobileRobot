#!/usr/bin/env python3
import rospy
import pandas as pd
import matplotlib.pyplot as plt
import os

def load_data(base_dir):
    # Load data from CSV files into Pandas DataFrames
    path_imu = os.path.join(base_dir, 'imu_data.csv')
    path_filtered = os.path.join(base_dir, 'filtered_calculated_inclinations.csv')
    path_shifted = os.path.join(base_dir, 'shifted_filtered_calculated_inclinations.csv')
    path_calculated = os.path.join(base_dir, 'calculated_inclinations.csv')
    path_orientations = os.path.join(base_dir, 'real_orientations.csv')

    df_imu = pd.read_csv(path_imu)
    df_filtered = pd.read_csv(path_filtered)
    df_shifted = pd.read_csv(path_shifted)
    df_calculated = pd.read_csv(path_calculated)
    df_orientations = pd.read_csv(path_orientations)

    return df_imu, df_filtered, df_shifted, df_calculated, df_orientations

def generate_graphs(base_dir, df_imu, df_filtered, df_shifted, df_calculated, df_orientations):
    # Generate and save graphs based on loaded data
    plt.figure(figsize=(12, 6))
    plt.plot(df_imu['TimeStamps'], df_imu['Linear acceleration X'], label='Acceleration (imu_data)')
    plt.plot(df_filtered['TimeStamp'], df_filtered['Inclination TCP (deg)'], label='Inclination (Filtered)')
    plt.plot(df_shifted['TimeStamp'], df_shifted['Inclination TCP (deg)'], label='Inclination (Shifted)')
    plt.plot(df_calculated['TimeStamp'], df_calculated['Inclination TCP (deg)'], label='Inclination (Calculated)')
    plt.xlabel('Timestamp')
    plt.ylabel('Values')
    plt.title('Acceleration and Inclinations')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(base_dir, 'first_graph.png'))  # Save the first graph
    plt.show()

    plt.figure(figsize=(12, 6))
    plt.plot(df_imu['TimeStamps'], df_imu['Linear acceleration X'], label='Acceleration (imu_data)')
    plt.plot(df_filtered['TimeStamp'], df_shifted['Inclination TCP (deg)'], label='Inclination (Shifted)')
    plt.plot(df_orientations['Timestamp'], df_orientations['Pitch'], label='Pitch (Orientations)')
    plt.xlabel('Timestamp')
    plt.ylabel('Values')
    plt.title('Acceleration, Inclination and Pitch')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(base_dir, 'second_graph.png'))  # Save the second graph
    plt.show()

def main():
    # Main function to load data and generate graphs
    rospy.init_node('data_plotter_node')
    test_folder_name = rospy.get_param('/test_folder_name')
    base_dir = os.path.join(os.getenv('ROS_PACKAGE_PATH').split(':')[0], 'robot_model', 'data', test_folder_name)
    df_imu, df_filtered, df_shifted, df_calculated, df_orientations = load_data(base_dir)
    generate_graphs(base_dir, df_imu, df_filtered, df_shifted, df_calculated, df_orientations)

if __name__ == '__main__':
    main()

