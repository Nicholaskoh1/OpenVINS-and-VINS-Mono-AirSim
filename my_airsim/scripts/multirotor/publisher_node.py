#!/usr/bin/env python

import setup_path
import airsim
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Enable API control of the drone
client.enableApiControl(True)

# ROS node initialisation
rospy.init_node('publisher_node')

# ROS publishers
imu_pub = rospy.Publisher( 'airsim_node/drone/imu/Imu', Imu, queue_size = 10 )
image_pub = rospy.Publisher( 'airsim_node/drone/front_center_custom/0', Image, queue_size = 10 )

# CV bridge for converting AirSim images to ROS images
bridge = CvBridge()

# Main loop
def talker():
    while not rospy.is_shutdown():      
        
        # Get IMU data
        imu_data = client.getImuData(vehicle_name = 'drone' )
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'airsim'
        imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y_val
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z_val
        imu_msg.angular_velocity.x = imu_data.angular_velocity.x_val
        imu_msg.angular_velocity.y = imu_data.angular_velocity.y_val
        imu_msg.angular_velocity.z = imu_data.angular_velocity.z_val
        imu_pub.publish(imu_msg)

        # Get camera image
        responses = client.simGetImages([airsim.ImageRequest("front_center_custom", airsim.ImageType.Scene, False, False)])
        response = responses[0]
        
        # Get numpy array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)

        # Reshape array to 3 channel image array H X W X 3
        img_rgb = img1d.reshape(response.height, response.width, 3)

        # Convert to grayscale
        img_mono8 = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

        # Publish to ROS topic
        img_ros = bridge.cv2_to_imgmsg(img_mono8, encoding="mono8")
        img_ros.header.stamp = rospy.Time.now()
        img_ros.header.frame_id = 'airsim'
        image_pub.publish(img_ros)
     
        # Sleep for a short time
        rospy.sleep(0.1)



# Allow ctrl-C to stop the node
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        # Clean up
        client.reset()
        client.enableApiControl(False)
