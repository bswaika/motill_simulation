from abc import ABC, abstractmethod
from typing import List, Dict

import numpy as np

from PointCloud import PointCloud, PointCloudSequence

class Parser(ABC):
    @abstractmethod
    def parse(self, filepath:str) -> PointCloud:
        '''Parses a file to a PointCloud'''

class SequenceParser(ABC):
    @abstractmethod
    def parse_sequence(self, filepath) -> PointCloudSequence:
        '''Parses a file to a PointCloudSequence'''

class Orchestrator(ABC):
    @abstractmethod
    def run(self):
        '''Runs the simulation for the orchestrator'''

class PathPlanner(ABC):
    @abstractmethod
    def set_bounds(self, bounds: Dict[str, List[np.ndarray]]):
        '''Sets up environment boundaries'''

    @abstractmethod
    def setup(self, sources, targets, obstacles):
        '''Sets up the Planner with the various types of locations'''

    @abstractmethod
    def run_update_loop(self, delay):
        '''Runs the update loop for path planning'''