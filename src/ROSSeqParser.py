import sys
from typing import List

import numpy as np
from rosbag import Bag
import yaml

from interfaces import SequenceParser
from PointCloud import PointCloud, PointCloudSequence

class ROSSeqParser(SequenceParser):
    def _get_rosbag_messages(self, filepath: str) -> List[str]:
        with Bag(filepath) as bag:
            result = [str(message.message) for message in bag.read_messages()]
        return result
    
    def parse_sequence(self, filepath: str) -> PointCloudSequence:
        messages = self._get_rosbag_messages(filepath)
        for message in messages:
            result = yaml.load(str(message), Loader=yaml.CLoader)
            if len(result['coordinate']) > 1:
                num_point_clouds = len(result['coordinate'])
                break
        point_clouds = [{'points': [], 'colors': []} for _ in range(num_point_clouds)]
        for message in messages:
            result = yaml.load(str(message), Loader=yaml.CLoader)
            if len(result['coordinate']) > 1:
                for idx, (point, color) in enumerate(zip(result['coordinate'], result['color'])):
                    point_clouds[idx]['points'].append([point['x'], point['y'], point['z']])
                    point_clouds[idx]['colors'].append([color['r'], color['g'], color['b']])
        return PointCloudSequence([PointCloud(np.array(pc['points']), np.array(pc['colors'])) for pc in point_clouds])

if __name__ == '__main__':
    parser = ROSSeqParser()
    print(parser.parse_sequence('../crose115.bag').downsample(15))