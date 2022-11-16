import numpy as np
import open3d as od

with open('./compressed_pcds/0_points.npy', 'rb') as infile:
    points = np.load(infile)

pcd = od.geometry.PointCloud()
pcd.points = od.utility.Vector3dVector(points)
print(pcd.get_center())
print(pcd.get_axis_aligned_bounding_box())