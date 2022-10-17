from math import cos, sin
import sys
import time
import airsim
from colour import Color
import numpy as np
import threading

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'
RESET_COMMAND = 'ke {vehicle_name} SetLightColor 0 0 0'
CUBE_LENGTH = 2 
NUMBER_DRONES = CUBE_LENGTH ** 3
SCALE = 0.5
Z_OFFSET = 3
X_OFFSET = 1
SPEED = 1
COLORS = list(Color("red").range_to(Color("blue"), 50))

class FLSDrone:
    def __init__(self, name, pose, color=(0, 0, 0)) -> None:
        self.name = name
        self.type = 'simpleflight'
        self.pose = airsim.Pose(airsim.Vector3r(pose[0], pose[1], pose[2]), airsim.Quaternionr())
        self.pawn_path = 'FLSDronePawn'
        self.color = color

client = airsim.MultirotorClient(ip=sys.argv[1])
client.confirmConnection()
client.enableApiControl(True)

for i in reversed(range(CUBE_LENGTH)):
    for j in reversed(range(CUBE_LENGTH)):
        for k in reversed(range(Z_OFFSET, CUBE_LENGTH + Z_OFFSET)):
            drone = f'fls_{((i * CUBE_LENGTH * CUBE_LENGTH) + ((j * CUBE_LENGTH) + (k - Z_OFFSET)))}'
            # print(drone)
            client.simAddVehicle(drone, 'simpleflight', airsim.Pose(airsim.Vector3r(3, 0, 0), airsim.Quaternionr()), 'FLSDronePawn')
            r, g, b = COLORS[0].get_rgb()
            client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=255 * r, g=255 * g, b=255 * b))
            client.enableApiControl(True, drone)
            client.takeoffAsync(drone).join()
            client.moveToPositionAsync(SCALE * i + X_OFFSET, SCALE * j, SCALE * -k, SPEED, vehicle_name=drone).join()
            client.hoverAsync().join()

time.sleep(1)
paths = {}

for angle in range(0, 360, 30):
    for i in reversed(range(CUBE_LENGTH)):
        for j in reversed(range(CUBE_LENGTH)):
            for k in reversed(range(Z_OFFSET, CUBE_LENGTH + Z_OFFSET)):
                drone = f'fls_{((i * CUBE_LENGTH * CUBE_LENGTH) + ((j * CUBE_LENGTH) + (k - Z_OFFSET)))}'
                # client.enableApiControl(True, drone)
                current_position = client.simGetVehiclePose(drone).position
                current_vector = np.array([[current_position.x_val], [current_position.y_val], [current_position.z_val]])
                rotation_matrix = np.array([[cos(angle), -sin(angle), 0], [sin(angle), cos(angle), 0], [0, 0, 1]])
                # print(rotation_matrix.shape, current_vector.shape)
                next_vector = rotation_matrix @ current_vector
                if drone not in paths:
                    paths[drone] = []
                paths[drone].append(next_vector.T[0].tolist())
                # client.moveToPositionAsync(next_vector[0][0], next_vector[1][0], next_vector[2][0], 0.5 * SPEED, vehicle_name=drone)
                # client.hoverAsync()
                # time.sleep(0.5)

# print(paths['fls_7'])

# threads = []
# for drone in paths:
#     threads.append(threading.Thread(target=client.moveToPositionAsync, name=drone, args=()))

for color in COLORS[1:]:
    for i in range(NUMBER_DRONES):
        r, g, b = color.get_rgb()
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=f'fls_{i}', r=255 * r, g=255 * g, b=255 * b))
    time.sleep(0.5)

time.sleep(1)