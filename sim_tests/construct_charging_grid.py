import math
import sys
import time
import random
import airsim
import json

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'

STATIONS = ['GroupActor_2', 'GroupActor_3', 'GroupActor_1', 'GroupActor_0']
STATION_TOP_INTERVAL = 2.5
STATION_BOUND_X = 5
STATION_BOUND_Y = 10
STATION_GRID_X_INTERVAL = 0.5
STATION_GRID_Y_INTERVAL = 0.5

CHARGE_TIME = 10
FLIGHT_TIME = 60

TEST_NUM_DRONES = 41 * 4 * 4
GRID_FILE = './grids/grid_41_4.json'

def spawn_extra_drones(start_id, num_drones, stations, client:airsim.MultirotorClient):
    curr_positions = [airsim.Vector3r(station.x_val-STATION_BOUND_X, station.y_val-STATION_BOUND_Y, station.z_val) for station in stations]
    i, station = 0, 0
    grid_dict = [[], [], [], []]
    added_drones = []
    while i < num_drones:
        drone = f'fls_{start_id+i+1}'
        client.simAddVehicle(drone, 'simpleflight', airsim.Pose(curr_positions[station]), 'FLSDronePawn')
        grid_dict[station].append(curr_positions[station].to_numpy_array().tolist())
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=150, g=150, b=150))
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
    return added_drones, grid_dict    

def simulate_stag_test(num_extra_drones, stations, client: airsim.MultirotorClient, beta=FLIGHT_TIME, omega=CHARGE_TIME):
    last_drone_id = -1
    drones, grid_dict = spawn_extra_drones(last_drone_id, num_extra_drones, stations, client)
    print(len(drones))
    return drones, grid_dict

def write_grid(grid, grid_file=GRID_FILE):
    grid = {key: grid[key] for key in range(len(grid))}
    with open(grid_file, 'w') as outfile:
        outfile.write(json.dumps(grid, indent=4))

def run_sim():
    client = airsim.MultirotorClient(ip=sys.argv[1])
    client.confirmConnection()
    client.enableApiControl(True)
    

    station_locations = [client.simGetObjectPose(station).position for station in STATIONS]
    station_locations = [airsim.Vector3r(station.x_val, station.y_val, station.z_val - STATION_TOP_INTERVAL) for station in station_locations]
    
    drones, grid_dict = simulate_stag_test(TEST_NUM_DRONES, station_locations, client)
    # print(grid_dict)
    write_grid(grid_dict)

run_sim()