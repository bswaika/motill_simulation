import os
import pickle
from typing import Dict, Union

from ROSParser import ROSParser
from ROSSeqParser import ROSSeqParser
from PointCloud import PointCloud, PointCloudSequence

if __name__ == '__main__':

    scenes: Dict[str, Union[PointCloud, PointCloudSequence]] = {
        'static': ROSParser().parse('../crose115.bag'),
        'motion': ROSSeqParser().parse_sequence('../crose115.bag')
    }

    motill = {
        'motill_static': scenes['static'].voxel_downsample(1.3),
        'motill_motion': scenes['motion'].downsample(15)
    }

    stag = {
        'stag_static': scenes['motion'].downsample(10)[50],
        'stag_static_2': scenes['motion'][50].voxel_downsample(0.8),
        'stag_static_3': scenes['motion'][50].voxel_downsample(0.6),
        'stag_rose_cropped': motill['motill_static'].crop_along_z_till_end(14, False).translate_along_z(-8)
    }

    files = set(os.listdir('../data'))

    for key in motill: 
        if f'{key}.pkl' not in files:
            with open(f'../data/{key}.pkl', 'wb') as outfile:
                pickle.dump(motill[key], outfile)
    
    for key in stag:
        if f'{key}.pkl' not in files:
            with open(f'../data/{key}.pkl', 'wb') as outfile:
                pickle.dump(stag[key], outfile)
