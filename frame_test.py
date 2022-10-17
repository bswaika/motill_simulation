import os
import numpy as np
import open3d as o3d

frame_files = os.listdir('./frames')
frame_files.sort(key=lambda x: int(x.split('.')[0]))

frames = [np.load(f'./frames/{f}') for f in frame_files]
# print(list(map(lambda x: x.shape, frames)))

def generate_point_cloud_from_frame(frame: np.ndarray):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(frame[:, 0, :].reshape(-1, 3))
    pcd.colors = o3d.utility.Vector3dVector(frame[:, 1, :].reshape(-1, 3))
    return pcd

pcds = list(map(generate_point_cloud_from_frame, frames))

# dpcds = list(map(lambda x: x.voxel_down_sample(0.5), pcds[1:-1]))
# print(dpcds)

        