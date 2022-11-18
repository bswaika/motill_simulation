import os, pickle
from PointCloud import PointCloud, PointCloudSequence

def read_data(data_dir='../data'):
    files = os.listdir(data_dir)
    scenes = {}
    for file in files:
        with open(f'{data_dir}/{file}', 'rb') as infile:
            scenes[file.split('.')[0]] = pickle.load(infile)
    return scenes
