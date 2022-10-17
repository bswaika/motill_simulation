import enum
import os
import sys
from time import sleep
import numpy as np
import random
import airsim

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {color[0]} {color[1]} {color[2]}'
RESET_COMMAND = 'ke {vehicle_name} SetLightColor 0 0 0'
ORIGIN = (47.641468, -122.140165, 122)
SCALE_X = 0.8
SCALE_Y = 0.8
SCALE_Z = 0.3

DIR = './paths'
path_files = os.listdir(DIR)
path_files.sort(key=lambda x: int(x.split('.')[0]))

paths = [np.load(f'{DIR}/{path}') for path in path_files]
# print(list(map(lambda x: x.shape, paths)))

down_sampled_paths = list(map(lambda x: x.tolist(), random.sample(paths, 20)))

client = airsim.MultirotorClient(ip=sys.argv[1])
client.confirmConnection()
client.enableApiControl(True)

frame=0
# for frame in range(len(down_sampled_paths[0])):
for idx, drone in enumerate(down_sampled_paths):
    client.simAddVehicle(f'fls_{idx}', 'simpleflight', airsim.Pose(airsim.Vector3r(3, 0, 0), airsim.Quaternionr()), 'FLSDronePawn')
    client.enableApiControl(True, f'fls_{idx}')
    # print(drone[frame][0])
    client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=f'fls_{idx}', color=list(map(lambda x: x * 255, drone[frame][1]))))
    client.takeoffAsync(vehicle_name=f'fls_{idx}').join()
    # path = list(map(lambda x: airsim.Vector3r(x[:][0][0], x[0][2], -x[0][1]), drone))
    # print(path)
    # client.moveOnPathAsync(path, 2, vehicle_name=f'fls_{idx}')
    client.moveToPositionAsync(SCALE_X * drone[frame][0][2], SCALE_Y * drone[frame][0][1], SCALE_Z * -drone[frame][0][0], 5, vehicle_name=f'fls_{idx}').join()
    client.hoverAsync(f'fls_{idx}').join()
    # print(client.simGetVehiclePose(f'fls_{idx}'))
    # print('ADDED')
    # sleep(0.1)

for idx, drone in enumerate(down_sampled_paths):
    path = list(map(lambda x: airsim.Vector3r(SCALE_X * x[:][0][2], SCALE_Y * x[:][0][1], SCALE_Z * -x[:][0][0]), drone))
    client.moveOnPathAsync(path, 1, vehicle_name=f'fls_{idx}')

sleep(5)