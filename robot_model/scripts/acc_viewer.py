#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import Imu
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import pyqtSlot, QTimer

class IMUDataWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        
        # Initialize ROS node and subscribe to IMU data topic
        rospy.init_node('acc_viewer', anonymous=True)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        
        self.acceleration = None
        
        # Set up timer to update the interface
        self.timer = QTimer(self)
        self.timer.setInterval(100)  # Update every 100 ms
        self.timer.timeout.connect(self.update_imu_data)
        self.timer.start()
        
    def initUI(self):
        # Set up basic UI structure
        self.layout = QVBoxLayout(self)
        self.imu_label = QLabel("IMU Acceleration: Waiting for data...")
        self.layout.addWidget(self.imu_label)
        self.setGeometry(300, 300, 350, 150)
        self.setWindowTitle('IMU Accelerations Viewer')

    @pyqtSlot()
    def update_imu_data(self):
        # Update UI with latest IMU data
        if self.acceleration:
            accel_text = f"x: {self.acceleration.x:.3f} m/s², y: {self.acceleration.y:.3f} m/s², z: {self.acceleration.z:.3f} m/s²"
            self.imu_label.setText(f"IMU Acceleration:\n{accel_text}")

    def imu_callback(self, msg):
        # Callback function for handling received IMU data
        self.acceleration = msg.linear_acceleration

def main():
    # Main function to set up and run the application
    app = QApplication(sys.argv)
    ex = IMUDataWidget()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

