#!/usr/bin/env python3

# Import necessary modules
import sys
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import pyqtSlot, QTimer
import tf.transformations as transformations

# Definition of the class for displaying end effector pose
class EndEffectorPoseWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

        # ROS node initialization and TF2 buffer creation
        rospy.init_node('pose_glass', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publisher for TCP pose
        self.pose_publisher = rospy.Publisher('/pose_tcp', PoseStamped, queue_size=10)
        
        # Configuration of timer for updating pose
        self.timer = QTimer(self)
        self.timer.setInterval(100)  # Update interval in milliseconds
        self.timer.timeout.connect(self.update_pose)  # Connection to update function
        self.timer.start()

    # Initialize the user interface
    def initUI(self):
        self.layout = QVBoxLayout(self)
        self.pose_label = QLabel("TCP Pose: Waiting for data...")
        self.layout.addWidget(self.pose_label)
        self.setGeometry(300, 300, 280, 150)
        self.setWindowTitle('Glass/TCP Pose Viewer')

    # Method to update the pose
    @pyqtSlot()
    def update_pose(self):
        end_effector_link = 'Link6'
        reference_frame = 'base_link'

        try:
            # Lookup transform from base_link to end effector
            trans = self.tf_buffer.lookup_transform(reference_frame, end_effector_link, rospy.Time(0), rospy.Duration(1.0))
            
            # Create PoseStamped message for TCP pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = reference_frame
            pose_msg.pose.position = trans.transform.translation
            pose_msg.pose.orientation = trans.transform.rotation
            self.pose_publisher.publish(pose_msg)

            # Convert quaternion to Euler angles for display
            euler = transformations.euler_from_quaternion([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w])
            euler_degrees = tuple(map(lambda x: x * 180.0 / 3.141592653589793, euler))
            pose_text = f"Position: ({trans.transform.translation.x:.3f}, {trans.transform.translation.y:.3f}, {trans.transform.translation.z:.3f}),\n" \
                        f"Orientation (Euler): Roll: {euler_degrees[0]:.2f}°, Pitch: {euler_degrees[1]:.2f}°, Yaw: {euler_degrees[2]:.2f}°"
            self.pose_label.setText(f"End Effector Pose:\n{pose_text}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.pose_label.setText("TCP Pose: Not available")

# Main function to run the application
def main():
    app = QApplication(sys.argv)
    ex = EndEffectorPoseWidget()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

