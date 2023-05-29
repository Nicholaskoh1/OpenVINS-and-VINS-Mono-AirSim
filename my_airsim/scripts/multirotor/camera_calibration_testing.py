import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

# variables
yy = 75.0
pp = 70
xx = -80
vv = 10
tt = 5

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move to center of board')
client.moveToPositionAsync(-150, 0, -28, vv).join()

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move to left of board')
client.moveToPositionAsync(xx, -20, -28, vv).join()
client.moveByRollPitchYawThrottleAsync(0 ,0, yy, 1, tt)

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move to right of board')
client.moveByRollPitchYawThrottleAsync(0, 0, y, 1, t)
client.moveToPositionAsync(x, 20, -28, v).join()
client.moveByRollPitchYawThrottleAsync(0, 0, y, 1, t)

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move to top of board')
client.moveByRollPitchYawThrottleAsync(0, 0, 360 - y, 1, t)
client.moveToPositionAsync(x, 0, -40, v).join()
client.moveByRollPitchYawThrottleAsync(0, p, 0, 1, t)

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move to bottom of board')
client.moveByRollPitchYawThrottleAsync(0, 360 - p, 0, 1, t)
client.moveToPositionAsync(x, 0, -16, v).join()
client.moveByRollPitchYawThrottleAsync(0, 360 - p, 0, 1, t)

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move closer to board')
client.moveByRollPitchYawThrottleAsync(0, p, 0, 1, t)
client.moveToPositionAsync(-100, 0, -28, v).join()

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move further from board')
client.moveToPositionAsync(-180, 0, -28, v).join()

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))


airsim.wait_key('Press any key to reset to original state')

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)

