from cmath import pi
import setup_path
import airsim
import math
import time

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# speed
zh = 2

# functions
def rotate():
    client.rotateByYawRateAsync(90, 1)
    client.hoverAsync().join()
    time.sleep(2)

def movement(x, y, z, s):
    client.moveToPositionAsync(x, y, z, s).join()
    client.hoverAsync().join()
    time.sleep(2)

# takeoff
client.armDisarm(True)
client.takeoffAsync().join()

# move upwards
movement(0, 0, -5, 1)

# move from origin to top center
movement(20, 20, -5, zh)

# descend
client.moveToPositionAsync(0, 0, -1, 1).join()
client.armDisarm(False)

# reset
airsim.wait_key('Press any key to reset to original state')
client.reset()

# clean quit
client.enableApiControl(False)