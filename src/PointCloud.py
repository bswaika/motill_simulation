from typing import List

import numpy as np
import open3d as o3d

class PointCloud:
    def __init__(self, points: np.ndarray, colors: np.ndarray) -> None:
        self.points = points
        self.colors = colors

    def voxel_downsample(self, factor):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.colors = o3d.utility.Vector3dVector(self.colors)
        pcd = pcd.voxel_down_sample(factor)
        return PointCloud(np.asarray(pcd.points), np.asarray(pcd.colors))

    def crop_along_z_till_end(self, offset, start=True):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.colors = o3d.utility.Vector3dVector(self.colors)
        bbox = self.get_bbox(pcd)
        bb_min, bb_max = bbox.get_min_bound(), bbox.get_max_bound()
        if not start:
            pcd = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=[bb_min[0], bb_min[1], bb_max[2]-offset], max_bound=bb_max))
        else:
            pcd = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=[bb_min[0], bb_min[1], bb_min[2]+offset], max_bound=bb_max))
        return PointCloud(np.asarray(pcd.points), np.asarray(pcd.colors))

    def translate_along_z(self, offset):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.colors = o3d.utility.Vector3dVector(self.colors)
        pcd = pcd.translate((0, 0, offset))
        return PointCloud(np.asarray(pcd.points), np.asarray(pcd.colors))

    def save_as_npy_file(self, path):
        points = np.array(self.points)
        colors = np.array(self.colors)
        with open(f'{path}_points.npy', 'wb') as outfile:
            np.save(outfile, points)
        with open(f'{path}_colors.npy', 'wb') as outfile:
            np.save(outfile, colors)

    def get_bbox(self, pc):
        return o3d.geometry.AxisAlignedBoundingBox.create_from_points(pc.points)

    
    def uniform_downsample(self, skip):
        return PointCloud(self.points[::skip, :], self.colors[::skip, :])

    def __repr__(self) -> str:
        return f'PointCloud(num_points={self.points.shape[0]})'

class PointCloudSequence:
    def __init__(self, point_clouds:List[PointCloud]) -> None:
        self._sequence = point_clouds
    
    def downsample(self, skip):
        return PointCloudSequence([pc.uniform_downsample(skip) for pc in self._sequence])

    def __len__(self) -> int:
        return len(self._sequence)

    def __getitem__(self, idx: int) -> PointCloud:
        return self._sequence[idx]
    
    def __repr__(self) -> str:
        return f'PointCloudSequence(num_point_clouds={len(self._sequence)}, avg_num_points_per_point_cloud={sum([pc.points.shape[0] for pc in self._sequence])/len(self._sequence)})'