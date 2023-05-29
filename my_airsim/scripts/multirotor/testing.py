import setup_path
import airsim
import math
import time


# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)


# variables
max_speed_x = 10
max_speed_y = 4
altitude = -40


# functions
def rotate(direction):
    client.rotateByYawRateAsync((direction * 45), 2).join()
    client.hoverAsync().join()
    time.sleep(3)


def movement(x, y, velocity, direction_of_rotation = 1):
    # initial coordinates of the drone
    initial_x = client.getMultirotorState().kinematics_estimated.position.x_val
    initial_y = client.getMultirotorState().kinematics_estimated.position.y_val

    # length of list used for 'for' loop for x and y directions
    range_x = 50 * velocity
    range_y = 50 * velocity

    # moving right
    if (client.getMultirotorState().kinematics_estimated.position.y_val - y) < -10:
        print ("moving right")

        print ("ramping up...")

        for i in range(range_y):
            ramping_speed_y = (i + 1) / 50
            client.moveByVelocityZAsync(0, ramping_speed_y, altitude, 0.01).join()

        print ("max speed...")

        for i in range(1000):
            if abs(y - client.getMultirotorState().kinematics_estimated.position.y_val) < 12:
                break
            client.moveByVelocityZAsync(0, velocity, altitude, 0.01).join()

        print ("ramping down...")

        for i in range(1000):
            if abs(y - client.getMultirotorState().kinematics_estimated.position.y_val) < 0.1:
                break
            decline_speed_y = velocity - ((i + 1) / 50)
            client.moveByVelocityZAsync(0, decline_speed_y, altitude, 0.01).join()
    
    # moving left
    elif (client.getMultirotorState().kinematics_estimated.position.y_val - y) > 10:
        print ("moving left")

        print ("ramping up...")

        for i in range(range_x):
            ramping_speed_y = -(i + 1) / 50
            client.moveByVelocityZAsync(0, ramping_speed_y, altitude, 0.01).join()

        print ("max speed...")

        for i in range(1000):
            if abs(y - client.getMultirotorState().kinematics_estimated.position.y_val) < 70:
                break
            client.moveByVelocityZAsync(0, -velocity, altitude, 0.01).join()

        print ("ramping down...")

        for i in range(1000):
            if abs(y - client.getMultirotorState().kinematics_estimated.position.y_val) < 0.1:
                break
            decline_speed_y = -velocity + ((i + 1) / 50)
            client.moveByVelocityZAsync(0, decline_speed_y, altitude, 0.01).join()

    # moving forward
    elif (x - initial_x) > 10:
        print ("moving forward")
      
        print ("ramping up...")

        for i in range(range_x):
            ramping_speed_x = (i + 1) / 50
            client.moveByVelocityZAsync(ramping_speed_x, 0, altitude, 0.01).join()

        print ("max speed...")

        for i in range(1000):
            if abs(x - client.getMultirotorState().kinematics_estimated.position.x_val) < 70:
                break
            client.moveByVelocityZAsync(velocity, 0, altitude, 0.1).join()

        print ("ramping down...")

        for i in range(1000):
            if abs(x - client.getMultirotorState().kinematics_estimated.position.x_val) < 0.1:
                break
            decline_speed_x = velocity - ((i + 1) / 50)
            client.moveByVelocityZAsync(decline_speed_x, 0, altitude, 0.01).join()
    
    # for moving backwards
    elif (x - initial_x) < 10:
        print ("moving backwards")

        print ("ramping up...")

        for i in range(range_x):
            ramping_speed_x = -(i + 1) / 50
            client.moveByVelocityZAsync(ramping_speed_x, 0, altitude, 0.01).join()

        print ("max speed...")

        for i in range(1000):
            if abs(x - client.getMultirotorState().kinematics_estimated.position.x_val) < 70:
                break
            client.moveByVelocityZAsync(-velocity, 0, altitude, 0.1).join()

        print ("ramping down...")

        for i in range(1000):
            if abs(x - client.getMultirotorState().kinematics_estimated.position.x_val) < 0.1:
                break
            decline_speed_x = -velocity + ((i + 1) / 50)
            client.moveByVelocityZAsync(decline_speed_x, 0, altitude, 0.01).join()
    
    client.hoverAsync().join()
    time.sleep(3)
    rotate(direction_of_rotation)


def teleport(x, y, z):

    pose = client.simGetVehiclePose()

    pose.position.x_val += (x - pose.position.x_val)
    pose.position.y_val += (y - pose.position.y_val)
    pose.position.z_val += (z - pose.position.z_val)

    client.simSetVehiclePose(pose, True)
    time.sleep(2)


# teleport to starting point
teleport(-125, -130, -1)
airsim.wait_key('Press any key to takeoff')


# disarm
client.armDisarm(True)
client.takeoffAsync().join()




teleport(-125, 125, -40)
client.hoverAsync().join()
rotate(-1)


# go back to starting point
movement(-125, -125, max_speed_x)

# descend
print ("descending...")
client.moveToPositionAsync(-125, 125, -1, 2).join()
client.armDisarm(False)


# reset
airsim.wait_key('Press any key to reset to original state')
client.reset()


# clean quit
client.enableApiControl(False)






