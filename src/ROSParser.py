import sys
from typing import List

import numpy as np
from rosbag import Bag
import yaml

from interfaces import Parser
from PointCloud import PointCloud

class ROSParser(Parser):
    def _get_rosbag_messages(self, filepath: str) -> List[str]:
        with Bag(filepath) as bag:
            result = [str(message.message) for message in bag.read_messages()]
        return result

    def parse(self, filepath: str) -> PointCloud:
        messages = self._get_rosbag_messages(filepath)
        points = []
        colors = []
        for message in messages:
            result = yaml.load(str(message), Loader=yaml.CLoader)
            if len(result['coordinate']) == 1:
                point = result['coordinate'][0]
                color = result['color'][0]
                points.append([point['x'], point['y'], point['z']])
                colors.append([color['r'], color['g'], color['b']])
        return PointCloud(np.array(points), np.array(colors))

if __name__ == '__main__':
    parser = ROSParser()
    print(parser.parse('../crose115.bag').voxel_downsample(0.8))