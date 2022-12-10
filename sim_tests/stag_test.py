import math
import sys
import time
import random
import airsim
import json
import numpy as np
from copy import deepcopy

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'

STATIONS = ['GroupActor_2', 'GroupActor_3', 'GroupActor_1', 'GroupActor_0']
STATION_TOP_INTERVAL = 3
STATION_BOUND_X = 5
STATION_BOUND_Y = 10
STATION_GRID_X_INTERVAL = 0.5
STATION_GRID_Y_INTERVAL = 0.5

CHARGE_TIME = 10
FLIGHT_TIME = 60

TEST_NUM_DRONES = 200
TEST = True

GRID_FILE = './grids/grid_41_4.json'
FRAME_NUMBER = 50

paths = [np.load(f'./paths/{p}.npy') for p in range(0, 2009, 15)]
frame = [path[FRAME_NUMBER, 0, :] for path in paths]

def load_grid(grid_file=GRID_FILE):
    with open(grid_file, 'r') as infile:
        grid = json.loads('\n'.join(infile.readlines()))
    grid = [[airsim.Pose(airsim.Vector3r(*loc)) for loc in item] for k, item in grid.items()]
    return grid

def spawn_extra_drones(start_id, num_drones, stations, client:airsim.MultirotorClient):
    curr_positions = [airsim.Vector3r(station.x_val-STATION_BOUND_X, station.y_val-STATION_BOUND_Y, station.z_val) for station in stations]
    i, station = 0, 0
    added_drones = []
    grid_list = []
    while i < num_drones:
        drone = f'fls_{start_id+i+1}'
        client.simAddVehicle(drone, 'simpleflight', airsim.Pose(curr_positions[station]), 'FLSDronePawn')
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=150, g=150, b=150))
        grid_list.append(curr_positions[station].to_numpy_array().tolist())
        added_drones.append(drone)
        i += 1
        if (curr_positions[station].y_val + STATION_GRID_Y_INTERVAL) > (stations[station].y_val + STATION_BOUND_Y):
            curr_positions[station].y_val = (stations[station].y_val - STATION_BOUND_Y)
            curr_positions[station].x_val += STATION_GRID_X_INTERVAL
        else:
            curr_positions[station].y_val += STATION_GRID_Y_INTERVAL
        station += 1
        if station == len(stations):
            station = 0
    return added_drones, grid_list    

def write_apf(apf, filename):
    with open(f'./apf/{filename}.csv', 'w') as outfile:
        outfile.write('\n'.join([','.join(list(map(str, item))) for item in apf]))

def simulate_stag(frame, drones, stations, client: airsim.MultirotorClient, beta=FLIGHT_TIME, omega=CHARGE_TIME):
    num_extra_drones = math.ceil((len(drones) * omega) / beta)
    last_drone_id = int(drones[-1].split('_')[-1])
    staggering_interval = beta / len(drones)
    extra_drones, grid_list = spawn_extra_drones(last_drone_id, num_extra_drones, stations, client)
    print(len(drones), staggering_interval, len(extra_drones))
    curr_pos_dict, start_pos_dict = {}, {}
    for idx, drone in enumerate(drones):
        y, x, z = frame[idx].tolist()
        curr_pos_dict[drone] = airsim.Vector3r(x, y, -z)
        start_pos_dict[drone] = airsim.Vector3r(x, y, -z)
        # client.simSetVehiclePose(airsim.Pose(curr_pos_dict[drone]), True, drone)
    for idx, drone in enumerate(extra_drones):
        curr_pos_dict[drone] = airsim.Vector3r(*grid_list[idx])
        start_pos_dict[drone] = airsim.Vector3r(*grid_list[idx])
        # client.simSetVehiclePose(airsim.Pose(curr_pos_dict[drone]), True, drone)
    original = set(drones)
    currently_illuminating = deepcopy(drones)
    currently_charging = deepcopy(extra_drones)
    # print(currently_charging, currently_illuminating)
    # temp_pose = (airsim.Vector3r(0, 0, -2))
    idx, counter = 0, 0
    time.sleep(2)
    while True:
        # print(curr_pos_dict['fls_5'])
        to_apf = []
        for i in range(len(currently_charging)):
            start_pose, end_pose = curr_pos_dict[currently_illuminating[idx]], curr_pos_dict[currently_charging[i]]
            # print(start_pose.position.to_numpy_array(), end_pose.position.to_numpy_array())
            # print(currently_illuminating[idx], currently_charging[i])
            # print(start_pose.position.to_numpy_array(), end_pose.position.to_numpy_array(), idx, i)
            to_apf.append((currently_illuminating[idx], *(start_pose.to_numpy_array().tolist()), *(end_pose.to_numpy_array().tolist()), currently_charging[i]))
            client.simSetVehiclePose(airsim.Pose(end_pose - start_pos_dict[currently_illuminating[idx]]), True, currently_illuminating[idx])
            client.simSetVehiclePose(airsim.Pose(start_pose - start_pos_dict[currently_charging[i]]), True, currently_charging[i])
            # client.simSetVehiclePose(airsim.Pose(end_pose - temp_pose + start_pose), True, currently_illuminating[idx])
            curr_pos_dict[currently_illuminating[idx]], curr_pos_dict[currently_charging[i]] = deepcopy(end_pose), deepcopy(start_pose)
            currently_charging[i], currently_illuminating[idx] = currently_illuminating[idx], currently_charging[i]
            # print(currently_illuminating[idx], currently_charging[i])
            idx += 1
            if idx == len(drones):
                idx = 0
        WRITE_APF = False
        if WRITE_APF:
            write_apf(to_apf, counter)
            counter += 1
        # sys.exit(0)
        current = set(currently_illuminating)
        time.sleep(staggering_interval * 10)
        # print(currently_charging[:10])
        # print(curr_pos_dict['fls_5'])
        # sys.exit(0)
        if current == original:
            print(current, original)
            break

def run_sim():
    client = airsim.MultirotorClient(ip=sys.argv[1])
    client.confirmConnection()
    client.enableApiControl(True)
    

    station_locations = [client.simGetObjectPose(station).position for station in STATIONS]
    station_locations = [airsim.Vector3r(station.x_val, station.y_val, station.z_val - STATION_TOP_INTERVAL) for station in station_locations]

    drones = []
    for i, point in enumerate(frame):
        drone = f'fls_{i}'
        y, x, z = point
        r, g, b = (255, 0, 0)
        client.simAddVehicle(drone, 'simpleflight', airsim.Pose(airsim.Vector3r(x, y, -z)), 'FLSDronePawn')
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=r, g=g, b=b))
        drones.append(drone)
    
    simulate_stag(frame, drones, station_locations, client)


run_sim()