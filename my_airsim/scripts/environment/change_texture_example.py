import setup_path
import airsim

c = airsim.MultirotorClient()
c.confirmConnection()

c.simSetObjectMaterialFromTexture("OrangeBall", "Black_and_white_checkered_pattern.jpg")

