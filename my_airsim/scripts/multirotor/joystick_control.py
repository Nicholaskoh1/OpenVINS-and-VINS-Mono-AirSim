import setup_path
import airsim
import pygame
from pygame.locals import *
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Enable API control of the drone
client.enableApiControl(True)

# ROS node initialization
rospy.init_node('airsim_joystick_control')

# ROS publishers
imu_pub = rospy.Publisher('airsim_node/drone/imu/Imu', Imu, queue_size=10)
image_pub = rospy.Publisher('airsim_node/drone/front_center_custom/0', Image, queue_size=10)

# CV bridge for converting AirSim images to ROS images
bridge = CvBridge()

# Initialize pygame
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Main loop
def joystick_control():
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                if event.axis == 0:
                    # Joystick left/right movement (axis 0)
                    roll = -event.value
                elif event.axis == 1:
                    # Joystick forward/backward movement (axis 1)
                    pitch = event.value
                elif event.axis == 2:
                    # Joystick up/down movement (axis 2)
                    throttle = -event.value
                elif event.axis == 3:
                    # Joystick yaw movement (axis 3)
                    yaw = event.value

            elif event.type == JOYBUTTONDOWN:
                if event.button == 0:
                    # Button 0 (A) for taking off
                    client.takeoffAsync().join()
                elif event.button == 1:
                    # Button 1 (B) for landing
                    client.landAsync().join()

        # Set the drone's velocity based on joystick input
        client.moveByVelocityAsync(roll, pitch, yaw, throttle, 0.1)

        # Get IMU data
        imu_data = client.getImuData(vehicle_name='drone')
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
        img_ros.header.stamp = rospy
	    img_ros.header.frame_id = 'airsim'
        image_pub.publish(img_ros)

        # Sleep for a short duration to control the loop rate
        rospy.sleep(0.01)

    # Disable API control of the drone before exiting
    client.enableApiControl(False)

# Run the joystick control loop
if __name__ == '__main__':
    try:
        joystick_control()
    except rospy.ROSInterruptException:
        pass

