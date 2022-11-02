import time
import airsim

# Time Units -> Seconds

class FLSDrone:
    CHARGING_TIME = 10
    FLIGHT_TIME = 30
    
    def __init__(self, name, loc, col, client: airsim.MultirotorClient):
        self.name = name
        self.loc = loc
        self.col = col
        self.client = client