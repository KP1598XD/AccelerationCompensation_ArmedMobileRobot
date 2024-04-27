#!/usr/bin/env python3
import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from sensor_msgs.msg import Imu
import tf.transformations
from math import degrees
from geometry_msgs.msg import Vector3

class IMUViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        
        # ROS initialization
        rospy.init_node('imu_euler_gui', anonymous=True)
        rospy.Subscriber('/imu2/data', Imu, self.imu_callback)
        self.orientation_pub = rospy.Publisher('/orientation_ee', Vector3, queue_size=10)

    def initUI(self):
        # Initialize the user interface layout
        self.layout = QVBoxLayout()
        
        # Labels to display roll, pitch, and yaw
        self.roll_label = QLabel('Roll: Waiting for data...')
        self.pitch_label = QLabel('Pitch: Waiting for data...')
        self.yaw_label = QLabel('Yaw: Waiting for data...')
        
        # Add labels to the layout
        self.layout.addWidget(self.roll_label)
        self.layout.addWidget(self.pitch_label)
        self.layout.addWidget(self.yaw_label)
        
        # Set the layout for the widget
        self.setLayout(self.layout)
        
        # Set window properties
        self.setGeometry(300, 300, 300, 150)
        self.setWindowTitle('IMU Euler Angles Viewer')
        self.show()

    def imu_callback(self, msg):
        # Callback function to process IMU data
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # Convert quaternion to Euler angles
        euler = tf.transformations.euler_from_quaternion(quaternion)
        
        # Convert Euler angles to degrees
        roll_deg = degrees(euler[0])
        pitch_deg = degrees(euler[1])
        yaw_deg = degrees(euler[2])
        
        # Update labels with Euler angles
        self.roll_label.setText(f'Roll: {roll_deg:.2f}°')
        self.pitch_label.setText(f'Pitch: {pitch_deg:.2f}°')
        self.yaw_label.setText(f'Yaw: {yaw_deg:.2f}°')
        
        # Publish Euler angles as Vector3 message
        orientation_msg = Vector3(roll_deg, pitch_deg, yaw_deg)
        self.orientation_pub.publish(orientation_msg)

if __name__ == '__main__':
    # Main function
    app = QApplication(sys.argv)
    ex = IMUViewer()
    sys.exit(app.exec_())

