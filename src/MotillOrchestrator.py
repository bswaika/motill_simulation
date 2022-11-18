import time
from typing import Dict, Union

import airsim

from interfaces import Orchestrator
from Client import Client
from PointCloud import PointCloud, PointCloudSequence

class MotillOrchestrator(Orchestrator):
    def __init__(self, client: Client, scene: Dict[str, Union[PointCloud, PointCloudSequence]], static_delay=0, motion_delay=0) -> None:
        super().__init__()
        self._client = client
        self._static_scene = scene['static']
        self._motion_sequence = scene['motion']
        self._delay_after_static = static_delay
        self._delay_in_motion = motion_delay

    def _get_next_id(self):
        return len(self._client.drones.keys())

    def _render_static_scene(self):
        for idx, (point, color) in enumerate(zip(self._static_scene.points, self._static_scene.colors)):
            name = f'fls_{idx}'
            y, x, z = point
            self._client.simAddVehicle(name, airsim.Vector3r(x, y, -z), tuple(color))
        time.sleep(self._delay_after_static)    
    
    def _render_motion_sequence(self):
        next_id = self._get_next_id()
        for idx, (point, color) in enumerate(zip(self._motion_sequence[0].points, self._motion_sequence[0].colors)):
            name = f'fls_{next_id + idx}'
            y, x, z = point
            self._client.simAddVehicle(name, airsim.Vector3r(x, y, -z), tuple(color))
        time.sleep(self._delay_in_motion)
        for point_cloud in self._motion_sequence[1:]:
            for idx, (point, color) in enumerate(zip(point_cloud.points, point_cloud.colors)):
                name = f'fls_{next_id + idx}'
                y, x, z = point
                self._client.drones[name].position = airsim.Vector3r(x, y, -z)
                self._client.drones[name].color = color
            time.sleep(self._delay_in_motion)

    def run(self):
        self._render_static_scene()
        self._render_motion_sequence()