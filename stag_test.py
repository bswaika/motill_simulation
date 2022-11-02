import math
import sys
import time
import random
import airsim

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'

STATIONS = ['GroupActor_2', 'GroupActor_3', 'GroupActor_1', 'GroupActor_0']
STATION_TOP_OFFSET = 2.5
STATION_BOUND_X = 5
STATION_BOUND_Y = 10
STATION_GRID_X_OFFSET = 0.5
STATION_GRID_Y_OFFSET = 0.5

CHARGE_TIME = 10
FLIGHT_TIME = 60

TEST_NUM_DRONES = 200

def spawn_extra_drones(start_id, num_drones, stations, client:airsim.MultirotorClient):
    curr_positions = [airsim.Vector3r(station.x_val-STATION_BOUND_X, station.y_val-STATION_BOUND_Y, station.z_val) for station in stations]
    i, station = 0, 0
    added_drones = []
    while i < num_drones:
        drone = f'fls_{start_id+i+1}'
        client.simAddVehicle(drone, 'simpleflight', airsim.Pose(curr_positions[station]), 'FLSDronePawn')
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=150, g=150, b=150))
        added_drones.append(drone)
        i += 1
        if (curr_positions[station].y_val + STATION_GRID_Y_OFFSET) > (stations[station].y_val + STATION_BOUND_Y):
            curr_positions[station].y_val = (stations[station] - STATION_BOUND_Y)
            curr_positions[station].x_val += STATION_GRID_X_OFFSET
        else:
            curr_positions[station].y_val += STATION_GRID_Y_OFFSET
        station += 1
        if station == len(stations):
            station = 0
    return added_drones    

def simulate_stag(drones, stations, client: airsim.MultirotorClient, beta=FLIGHT_TIME, omega=CHARGE_TIME):
    num_extra_drones = math.ceil((len(drones) * omega) / beta)
    last_drone_id = int(drones[-1].split('_')[-1])
    drones.extend(spawn_extra_drones(last_drone_id, num_extra_drones, stations, client))
    print(len(drones))

def run_sim():
    client = airsim.MultirotorClient(ip=sys.argv[1])
    client.confirmConnection()
    client.enableApiControl(True)
    
    station_locations = [client.simGetObjectPose(station).position for station in STATIONS]
    station_locations = [airsim.Vector3r(station.x_val, station.y_val, station.z_val - STATION_TOP_OFFSET) for station in station_locations]

    drones = []
    for i in range(TEST_NUM_DRONES):
        drone = f'fls_{i}'
        x, y = random.randint(5, 50), random.randint(5, 50)
        r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
        client.simAddVehicle(drone, 'simpleflight', airsim.Pose(airsim.Vector3r(x, y, -10)), 'FLSDronePawn')
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=r, g=g, b=b))
        drones.append(drone)
    
    simulate_stag(drones, station_locations, client)


run_sim()