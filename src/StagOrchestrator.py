import time
import math
from copy import deepcopy

import airsim

from interfaces import Orchestrator
from Client import Client
from PointCloud import PointCloud
from interfaces import PathPlanner

class StagOrchestrator(Orchestrator):
    FLIGHT_TIME = 30
    CHARGE_TIME = 10
    STATION_TOP_OFFSET = airsim.Vector3r(0, 0, -3)
    STATION_BOUND_X = 5
    STATION_BOUND_Y = 10
    STATION_GRID_X_INTERVAL = 1
    STATION_GRID_Y_INTERVAL = 1
    STATIONS = ['GroupActor_2', 'GroupActor_3', 'GroupActor_1', 'GroupActor_0']

    def __init__(self, client: Client, scene: PointCloud, planner: PathPlanner, standalone: bool=True, beta: int=FLIGHT_TIME, omega: int=CHARGE_TIME, delay: float=0) -> None:
        super().__init__()
        self._client = client
        self._scene = scene
        self._delay = delay
        self._beta = beta
        self._omega = omega
        self._stag_interval = beta / scene.points.shape[0]
        self._grid = []
        self._charging = None
        self._illuminating = None
        self._charge_color = (150, 150, 150)
        self._sort = lambda x: sorted(x, key=lambda y: int(y.split('_')[1]))
        self._planner = planner
        self._standalone = standalone

    def _get_next_id(self):
        return len(self._client.drones.keys())

    def _get_station_locations(self):
        return [self._client.simGetObjectPose(station).position + self.STATION_TOP_OFFSET for station in self.STATIONS]

    def _render_static_scene(self):
        for idx, (point, color) in enumerate(zip(self._scene.points, self._scene.colors)):
            name = f'fls_{idx}'
            y, x, z = point
            self._client.simAddVehicle(name, airsim.Vector3r(x, y, -z), tuple(color))
        time.sleep(self._delay)

    def _compute_extra_drones(self) -> int:
        return math.ceil((self._get_next_id() * self._omega) / self._beta)
    
    def _add_extra_drones(self):
        stations = self._get_station_locations()
        num_extra_drones = self._compute_extra_drones()
        next_id = self._get_next_id()
        curr_positions = [station - airsim.Vector3r(self.STATION_BOUND_X, self.STATION_BOUND_Y, 0) for station in stations]
        i, station = 0, 0
        while i < num_extra_drones:
            name = f'fls_{next_id + i}'
            self._client.simAddVehicle(name, deepcopy(curr_positions[station]), self._charge_color, True)
            self._grid.append(deepcopy(curr_positions[station]))
            i += 1
            if (curr_positions[station].y_val + self.STATION_GRID_Y_INTERVAL) > (stations[station].y_val + self.STATION_BOUND_Y):
                curr_positions[station].y_val = (stations[station].y_val - self.STATION_BOUND_Y)
                curr_positions[station].x_val += self.STATION_GRID_X_INTERVAL
            else:
                curr_positions[station].y_val += self.STATION_GRID_Y_INTERVAL
            station += 1
            if station == len(stations):
                station = 0
        time.sleep(self._delay)

    def _run_stag_exchange_loop(self, idx, debug=False):
        sources, targets = {}, {}
        obstacles = set(self._illuminating)
        for i in range(len(self._charging)):
            start_position = deepcopy(self._client.drones[self._illuminating[idx]].position)
            end_position = deepcopy(self._client.drones[self._charging[i]].position)
            sources[self._illuminating[idx]] = start_position.to_numpy_array()
            targets[self._illuminating[idx]] = end_position.to_numpy_array()
            sources[self._charging[i]] = end_position.to_numpy_array()
            targets[self._charging[i]] = start_position.to_numpy_array()
            # obstacles -= {self._illuminating[idx]}
            # self._client.drones[self._illuminating[idx]].position = end_position
            # self._client.drones[self._charging[i]].position = start_position
            if debug: print(f'Pre-Exchange: {self._illuminating[idx]} {start_position.to_numpy_array().tolist()} {end_position.to_numpy_array().tolist()} {self._charging[i]}')
            self._charging[i], self._illuminating[idx] = self._illuminating[idx], self._charging[i]
            if debug: print(f'Post-Exchange: {self._illuminating[idx]} {self._client.drones[self._illuminating[idx]].position.to_numpy_array().tolist()} {self._client.drones[self._charging[i]].position.to_numpy_array().tolist()} {self._charging[i]}')
            idx += 1
            if idx == len(self._illuminating): idx = 0
        return idx, sources, targets, obstacles

    def run(self):
        if self._standalone:
            self._render_static_scene()
        self._illuminating = set(self._client.drones.keys())
        self._add_extra_drones()
        self._charging = set(self._client.drones.keys()) - self._illuminating
        self._illuminating, self._charging = self._sort(list(self._illuminating)), self._sort(list(self._charging))
        original = set(self._illuminating)
        idx = 0
        while True:
            idx, sources, targets, obstacles = self._run_stag_exchange_loop(idx)
            self._planner.setup(sources, targets, obstacles)
            self._planner.run_update_loop()
            break
            time.sleep((self._stag_interval * 10))
            if set(self._illuminating) == original and self._standalone:
                    break
        