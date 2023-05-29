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
altitude = -45


# functions
def rotate(direction):
    client.rotateByYawRateAsync((direction * 9), 10).join()
    client.hoverAsync().join()
    time.sleep(3)


def movement(x, y, velocity_max = max_speed_x, direction_of_rotation = 1):
    # initial coordinates of the drone
    initial_x = client.getMultirotorState().kinematics_estimated.position.x_val
    initial_y = client.getMultirotorState().kinematics_estimated.position.y_val
    print (initial_x)
    print (initial_y)

    # length of list used for 'for' loop for x and y directions
    range_list = 50 * velocity_max
    ramp_down_distance = 0.75 * ((velocity_max ** 2) - 1)

    # moving right
    if (y - initial_y) > 20:
        print ("moving right")

        print ("ramping up...")

        for i in range(range_list):
            ramping_speed = (i + 1) / 50
            client.moveByVelocityZAsync(0, ramping_speed, altitude, 0.01).join()

        print ("max speed...")

        for i in range(1000):
            if abs(y - client.getMultirotorState().kinematics_estimated.position.y_val) < ramp_down_distance:
                break
            client.moveByVelocityZAsync(0, velocity_max, altitude, 0.02).join()

        print ("ramping down...")

        for i in range(1000):
            if abs(y - client.getMultirotorState().kinematics_estimated.position.y_val) < 0.1 or abs(client.getMultirotorState().kinematics_estimated.linear_velocity.y_val) < 0.1:
                break
            decline_speed = velocity_max - ((i + 1) / 50)
            client.moveByVelocityZAsync(0, decline_speed, altitude, 0.02).join()
    
    # moving left
    elif (y - initial_y) < -20:
        print ("moving left")

        print ("ramping up...")

        for i in range(range_list):
            ramping_speed = -(i + 1) / 50
            client.moveByVelocityZAsync(0, ramping_speed, altitude, 0.01).join()

        print ("max speed...")

        for i in range(1000):
            if abs(y - client.getMultirotorState().kinematics_estimated.position.y_val) < ramp_down_distance:
                break
            client.moveByVelocityZAsync(0, -velocity_max, altitude, 0.02).join()

        print ("ramping down...")

        for i in range(1000):
            if abs(y - client.getMultirotorState().kinematics_estimated.position.y_val) < 0.1 or abs(client.getMultirotorState().kinematics_estimated.linear_velocity.y_val) < 0.1:
                break
            decline_speed = -velocity_max + ((i + 1) / 50)
            client.moveByVelocityZAsync(0, decline_speed, altitude, 0.02).join()

    # moving forward
    elif (x - initial_x) > 20:
        print ("moving forward")
      
        print ("ramping up...")

        for i in range(range_list):
            ramping_speed = (i + 1) / 50
            client.moveByVelocityZAsync(ramping_speed, 0, altitude, 0.01).join()

        print ("max speed...")

        for i in range(1000):
            if abs(x - client.getMultirotorState().kinematics_estimated.position.x_val) < ramp_down_distance:
                break
            client.moveByVelocityZAsync(velocity_max, 0, altitude, 0.02).join()

        print ("ramping down...")

        for i in range(1000):
            if abs(x - client.getMultirotorState().kinematics_estimated.position.x_val) < 0.1 or abs(client.getMultirotorState().kinematics_estimated.linear_velocity.x_val) < 0.1:
                break
            decline_speed = velocity_max - ((i + 1) / 50)
            client.moveByVelocityZAsync(decline_speed, 0, altitude, 0.02).join()
    
    # for moving backwards
    elif (x - initial_x) < -20:
        print ("moving backwards")

        print ("ramping up...")

        for i in range(range_list):
            ramping_speed = -(i + 1) / 50
            client.moveByVelocityZAsync(ramping_speed, 0, altitude, 0.01).join()

        print ("max speed...")

        for i in range(1000):
            if abs(x - client.getMultirotorState().kinematics_estimated.position.x_val) < ramp_down_distance:
                break
            client.moveByVelocityZAsync(-velocity_max, 0, altitude, 0.02).join()

        print ("ramping down...")

        for i in range(1000):
            if abs(x - client.getMultirotorState().kinematics_estimated.position.x_val) < 0.1 or abs(client.getMultirotorState().kinematics_estimated.linear_velocity.x_val) < 0.1:
                break
            decline_speed = -velocity_max + ((i + 1) / 50)
            client.moveByVelocityZAsync(decline_speed, 0, altitude, 0.02).join()
    
    #client.hoverAsync().join()
    time.sleep(4)
    rotate(direction_of_rotation)


# disarm
client.armDisarm(True)
client.takeoffAsync().join()


# move upwards
print("ascending...")
client.moveToPositionAsync(0, 0, altitude, 2).join()
client.hoverAsync().join()
time.sleep(3)


# move from origin to top center
movement(200, 0)

# move from top center to top right
movement(200, 200)

# move from top right to bottom right
movement(-200, 200)

# move from bottom right to bottom left
movement(-200, -200)

# move from bottom left to top left
movement(200, -200)

# move from top left to top center
movement(200, 0)

# move from top center to origin
movement(0, 0, max_speed_x, 2)


# descend
print ("descending...")
client.moveToPositionAsync(0, 0, -1, 2).join()
client.armDisarm(False)


# reset
airsim.wait_key('Press any key to reset to original state')
client.reset()


# clean quit
client.enableApiControl(False)