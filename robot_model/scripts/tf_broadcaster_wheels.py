#!/usr/bin/env python3

# Import ROS and necessary libraries
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Function to publish wheel transforms
def publish_wheel_transforms():
    # Initialize ROS node
    rospy.init_node('wheel_tf_broadcaster', anonymous=True)
    
    # Initialize transform broadcaster
    broadcaster = tf2_ros.TransformBroadcaster()	
    
    # Set the publishing rate to 10 Hz
    rate = rospy.Rate(10.0)

    # Main loop
    while not rospy.is_shutdown():
        # Loop through each wheel
        for wheel in ["link_leftFrontWheel", "link__rightFrontWheel", "link__rightRearWheel", "link__leftRearWheel"]:
            # Create a new transform message
            t = TransformStamped()
            
            # Set header information
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "base_link"
            t.child_frame_id = wheel
            
            # Set translation values based on the wheel type
            if wheel == "link_leftFrontWheel" or wheel == "link__rightFrontWheel":
                t.transform.translation.x = 0.315 
            if wheel == "link_leftFrontWheel" or wheel == "link__leftRearWheel":
                t.transform.translation.y = 0.3525 
            if wheel == "link__rightFrontWheel" or wheel == "link__rightRearWheel":
                t.transform.translation.y = -0.3525 
            if wheel == "link__rightRearWheel" or wheel == "link__leftRearWheel":
                t.transform.translation.x = -0.315  
            t.transform.translation.z = -0.32235
            
            # Set fixed rotation values
            t.transform.rotation.x = 0.7071081
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 0.7071081
            
            # Broadcast the transform
            broadcaster.sendTransform(t)
        
        # Sleep to maintain the publishing rate
        rate.sleep()

# Main function
if __name__ == '__main__':
    try:
        # Call the function to publish wheel transforms
        publish_wheel_transforms()
    except rospy.ROSInterruptException:
        pass

