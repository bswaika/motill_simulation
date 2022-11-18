import pickle

from ROSParser import ROSParser
from ROSSeqParser import ROSSeqParser
from PointCloud import PointCloud, PointCloudSequence

if __name__ == '__main__':

    scenes = {
        'static': ROSParser().parse('../crose115.bag'),
        'motion': ROSSeqParser().parse_sequence('../crose115.bag')
    }

    motill = {
        'motill_static': scenes['static'].voxel_downsample(1.3),
        'motill_motion': scenes['motion'].downsample(15)
    }

    stag = {
        'stag_static': scenes['motion'].downsample(10)[50]
    }

    for key in motill:
        with open(f'../data/{key}.pkl', 'wb') as outfile:
            pickle.dump(motill[key], outfile)
    
    for key in stag:
        with open(f'../data/{key}.pkl', 'wb') as outfile:
            pickle.dump(stag[key], outfile)
