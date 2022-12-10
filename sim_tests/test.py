import rosbag
import numpy as np
import yaml
# import open3d as o3d

class Point:
    def __init__(self, x, y, z, r, g, b, s=0, e=0) -> None:
        self.location = (x, y, z)
        self.color = (r/255, g/255, b/255)
        self.start, self.end = s, e
    
    def __str__(self) -> str:
        return f'Point(location={str(self.location)}, color={str(self.color)}, start={str(self.start)}, end={str(self.end)}'
    
    def to_list(self) -> list:
        return [[self.location[0], self.location[1], self.location[2]], [self.color[0], self.color[1], self.color[2]]]    

    def to_numpy(self) -> np.ndarray:
        return np.array([[self.location[0], self.location[1], self.location[2]], [self.color[0], self.color[1], self.color[2]]])

# def generate_point_cloud_from_frame(frame: np.ndarray):
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(frames[0][1][:, 0, :].reshape(-1, 3))
#     pcd.colors = o3d.utility.Vector3dVector(frames[0][1][:, 1, :].reshape(-1, 3))
#     return pcd

bag = rosbag.Bag('./crose115.bag')
points = []
paths = []
# i = 0
for msg in bag.read_messages():
    if str(msg).find('start: 112') + 1:
        result = yaml.load(str(msg.message), Loader=yaml.CLoader)
        path = []
        for loc, col, dur in zip(result['coordinate'], result['color'], result['duration']):
            path.append(Point(loc['x'], loc['y'], loc['z'], col['r'], col['g'], col['b'], dur['start'], dur['end']))
        path.sort(key=lambda pt: pt.start)
        paths.append(list(map(lambda pt: pt.to_list(), path)))
            # points.append(Point(loc['x'], loc['y'], loc['z'], col['r'], col['g'], col['b'], dur['start'], dur['end']))
        # print(f'Parsed {i}')
        # i += 1
    else:
        pass
        # params = dict(map(lambda x: (x[0].strip(), x[1].strip()), filter(lambda x: len(x) > 1 and x[1] != '', (map(lambda x: x.strip().split(':'), str(msg.message).split('\n'))))))
        # point = Point(float(params['x']), float(params['y']), float(params['z']), float(params['r']), float(params['g']), float(params['b']), int(params['start']), int(params['end']))
        # points.append(point)

for idx, path in enumerate(paths):
    with open(f'./paths/{idx}.npy', 'wb') as outfile:
        np.save(outfile, np.array(path))

# print(result)

# print(len(paths))
# print(list(map(lambda x: len(x), paths)))
# print(paths[-1])
# with open('test.txt', 'w') as outfile:
#     outfile.write(paths[-1])
# starts = [list(map(lambda x: x.to_list(), filter(lambda x: x.start == i, points))) for i in range(1, 116)]
# ends = [list(map(lambda x: x.to_list(), filter(lambda x: x.end == i, points))) for i in range(1, 116)]
# print(list(map(len, starts)))
# print(list(map(len, ends)))

# from_starts = list(filter(lambda x: len(x[1]) > 0, enumerate(starts)))
# from_ends = list(filter(lambda x: len(x[1]) > 0, enumerate(ends)))
# frames = list(map(lambda x: (x[0], np.array(x[1])), from_starts + from_ends))
# for frame_id, frame_info in frames:
#     with open(f'./frames/{frame_id}.npy', 'wb') as outfile:
#         np.save(outfile, frame_info)

# print(list(map(lambda x: (x[0], x[1].shape), frames)))
# pt_clds = list(map(lambda x: generate_point_cloud_from_frame(x[1]), frames))
# down_sampled_pt_clds = [cloud.voxel_down_sample(0.4) for cloud in pt_clds]
# tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pt_clds[0])
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pt_clds[0], 0.5, tetra_mesh, pt_map)
# mesh.compute_vertex_normals()
# down = o3d.geometry.PointCloud(mesh.vertices).voxel_down_sample(0.4)
# print(mesh.vertices)
# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
# o3d.visualization.draw_geometries([down])
# print(down)
# o3d.visualization.draw_geometries([down_sampled_pt_clds[0]])
# print(down_sampled_pt_clds[0])