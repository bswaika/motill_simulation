import os
import sys
import time
import numpy as np
import open3d as o3d
import airsim

frame_files = os.listdir('./frames')
frame_files.sort(key=lambda x: int(x.split('.')[0]))

frames = [np.load(f'./frames/{f}') for f in frame_files]
paths = [np.load(f'./paths/{p}.npy') for p in range(0, 2009, 15)]

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'
RESET_COMMAND = 'ke {vehicle_name} SetLightColor 0 0 0'

def generate_point_cloud_from_frame(frame: np.ndarray):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(frame[:, 0, :].reshape(-1, 3))
    pcd.colors = o3d.utility.Vector3dVector(frame[:, 1, :].reshape(-1, 3))
    return pcd

def get_bounding_box(pc):
    return o3d.geometry.AxisAlignedBoundingBox.create_from_points(pc.points)

pcds = list(map(generate_point_cloud_from_frame, frames))
start_pcd = pcds[0]
start_bb = get_bounding_box(start_pcd)
start_bb_min, start_bb_max = start_bb.get_min_bound(), start_bb.get_max_bound()
print(start_pcd)
start_pcd = start_pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=[start_bb_min[0], start_bb_min[1], 6 + start_bb_min[2]], max_bound=start_bb_max))
print(start_pcd)
start_dpcd = start_pcd.voxel_down_sample(1.1)
print(start_dpcd)

###########################
# Saving COMPRESSED PCD
###########################
# points = np.asarray(start_dpcd.points)
# colors = np.asarray(start_dpcd.colors)
# with open('compressed_pcds/0_points.npy', 'wb') as outfile:
#     np.save(outfile, points)
# with open('compressed_pcds/0_colors.npy', 'wb') as outfile:
#     np.save(outfile, colors)
###########################

def run_sim():
    OFFSET = (0, 0, 6)
    SCALE = 0.65
    # FRAMES = 10
    FRAMES = len(paths[0])

    client = airsim.MultirotorClient(ip=sys.argv[1])
    client.confirmConnection()
    client.enableApiControl(True)
    # client.simSetTimeOfDay(True, start_datetime = "2022-12-20 23:59:59", is_start_datetime_dst = False, celestial_clock_speed = 1, update_interval_secs = 60, move_sun = False)

    for idx, (p, c) in enumerate(zip(start_dpcd.points, start_dpcd.colors)):
        drone = f'fls_{idx}'
        client.simAddVehicle(drone, 'simpleflight', airsim.Pose(airsim.Vector3r(SCALE*p[1], SCALE*p[0], SCALE*(OFFSET[2]-p[2])), airsim.Quaternionr()), 'FLSDronePawn')
        r, g, b = c
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=255 * r, g=255 * g, b=255 * b))
    
    time.sleep(2)

    frame = 0
    for idx, path in enumerate(paths):
        drone = f'moving_fls_{idx}'
        y, x, z = path[frame][0]
        r, g, b = path[frame][1]
        client.simAddVehicle(drone, 'simpleflight', airsim.Pose(airsim.Vector3r(SCALE*x, SCALE*y, SCALE*(OFFSET[2]-z)), airsim.Quaternionr()), 'FLSDronePawn')
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=255 * r, g=255 * g, b=255 * b))
    
    frame += 1
    while frame < FRAMES:
        print(frame)
        for idx, path in enumerate(paths):
            drone = f'moving_fls_{idx}'
            y_off, x_off, z_off = path[0][0]
            z_off = OFFSET[2]-z_off
            y, x, z = path[frame][0]
            r, g, b = path[frame][1]
            client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(SCALE*(x-x_off), SCALE*(y-y_off), SCALE*(OFFSET[2]-z-z_off)), airsim.Quaternionr()), True, drone)
            client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=255 * r, g=255 * g, b=255 * b))
        frame += 1
        
run_sim()