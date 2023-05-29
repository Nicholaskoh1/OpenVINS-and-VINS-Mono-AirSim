import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import time

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# Taking off
#airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

# Move until camera lines up with board
#client.moveToPositionAsync(-120, 0, -27, 10).join()
#client.hoverAsync().join()

# Move to left of board
##client.moveByVelocityBodyFrameAsync(0, 0, 0, 3, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 50)).join()
#client.hoverAsync().join()
#time.sleep(5)

# Move to right of board
#client.moveByVelocityBodyFrameAsync(0, 0, 0, 3, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
#client.moveToPositionAsync(-30, 70, -27, 10).join()
#client.moveByVelocityBodyFrameAsync(0, 0, 0, 3, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, -50)).join()
#client.hoverAsync().join()
#time.sleep(5)

# Move to top of board
#client.moveByVelocityBodyFrameAsync(0, 0, 0, 1, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
#client.moveToPositionAsync(-60, 0, -85, 30).join()
#client.hoverAsync().join()
#client.moveByRollPitchYawThrottleAsync(0, (0.3 * np.pi), 0, 1.0, 8).join()
#client.hoverAsync().join()

# Move to bottom of board
client.moveToPositionAsync(0, 0, -19, 30).join()
client.hoverAsync().join()
client.moveByRollPitchYawThrottleAsync(0, (-0.22 * np.pi), 0, 1.0, 3).join()
client.hoverAsync().join()

# Return to original state
airsim.wait_key('Press any key to reset to original state')

client.reset()
client.armDisarm(False)
client.enableApiControl(False)