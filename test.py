import rosbag

class Point:
    def __init__(self, x, y, z, r, g, b, a) -> None:
        self.location = (x, y, z)
        self.color = (r, g, b, a)
    
    def __str__(self) -> str:
        return f'Point(location={str(self.location)}, color={str(self.color)}'

bag = rosbag.Bag('./crose115.bag')
starts = {i:[] for i in range(1, 116)}
ends = {i:[] for i in range(1, 116)}
for msg in bag.read_messages():
    params = dict(map(lambda x: (x[0].strip(), x[1].strip()), filter(lambda x: len(x) > 1 and x[1] != '', (map(lambda x: x.strip().split(':'), str(msg.message).split('\n'))))))
    point = Point(float(params['x']), float(params['y']), float(params['z']), float(params['r']), float(params['g']), float(params['b']), float(params['a']))
    starts[int(params['start'])].append(point)
    ends[int(params['end'])].append(point)

# print(len(starts[1]))

print(bag.get_type_and_topic_info())