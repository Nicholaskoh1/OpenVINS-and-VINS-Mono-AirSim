import setup_path 
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()


pose = client.simGetVehiclePose()

# functions
def teleport(x, y, z):

    pose.position.x_val += (x - pose.position.x_val)
    pose.position.y_val += (y - pose.position.y_val)
    pose.position.z_val += (z - pose.position.z_val)

    client.simSetVehiclePose(pose, True)
 

# teleport
teleport(20, 20, -5)
time.sleep(2)

teleport(0, 0, 0)
