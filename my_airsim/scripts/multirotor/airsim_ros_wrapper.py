#!/usr/bin/env python
import setup_path
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
import airsim

class AirSimROSWrapper:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.pub_image = rospy.Publisher('/airsim/image_raw/compressed', CompressedImage, queue_size=1)

    def capture_image(self):
        rawImage = self.client.simGetImage("0", airsim.ImageType.Scene)
        png = rawImage.to_png_string()
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "png"
        msg.data = png
        self.pub_image.publish(msg)

    def capture_imu(self):
        imu_data = self.client.getImuData()
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.linear_acceleration.x = imu_data.linear_acceleration.x_val
        imu.linear_acceleration.y = imu_data.linear_acceleration.y_val
        imu.linear_acceleration.z = imu_data.linear_acceleration.z_val
        imu.angular_velocity.x = imu_data.angular_velocity.x_val
        imu.angular_velocity.y = imu_data.angular_velocity.y_val
        imu.angular_velocity.z = imu_data.angular_velocity.z_val
        imu.orientation.x = imu_data.orientation.x_val
        imu.orientation.y = imu_data.orientation.y_val
        imu.orientation.z = imu_data.orientation.z_val
        imu.orientation.w = imu_data.orientation.w_val
        self.pub_imu.publish(imu)

    def run(self):
        while not rospy.is_shutdown():
            self.capture_image()
            self.capture_imu()

if __name__ == '__main__':
    wrapper = AirSimROSWrapper()
    wrapper.run() 


