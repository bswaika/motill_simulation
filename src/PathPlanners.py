from copy import deepcopy

import numpy as np
import airsim

from interfaces import PathPlanner
from Client import Client

class APFPlannerParams:
    k_attraction = 500
    d_repulsion_in = 1
    d_repulsion_out = 10
    k_repulsion_in = 300
    k_repulsion_out = 50
    power_factor = 3
    step_length = 2
    d_threshold = 2

# TODO
# [] Analyze which drones are not reaching their targets
# [] Either implement 'illuminating' and 'charging' markers to not consider similarly marked items as obstacles for them
# [] Or implement 'traffic control' by stalling some percent of drones that are alternating, moving the rest, then resuming the stalled drones (BETTER IMO, BUT MAYBE HARDER TO IMPLEMENT)

class TrafficManager:
    pass

class APFPathPlanner(PathPlanner):
    def __init__(self, client: Client, similar_swaps_only: bool=True):
        self._client = client
        self._params = APFPlannerParams()
        self._no_change_threshold = 50
        self._similar_swaps = similar_swaps_only

    def _get_repulsive_force(self, source, target, obstacle):
        obstacle_dist, target_dist = np.linalg.norm(source - obstacle), np.linalg.norm(source - target)
        if obstacle_dist <= self._params.d_repulsion_in:
            coeff = self._params.k_repulsion_in
            bound = self._params.d_repulsion_in
        elif obstacle_dist <= self._params.d_repulsion_out and obstacle_dist > self._params.d_repulsion_in:
            coeff = self._params.k_repulsion_out
            bound = self._params.d_repulsion_out
        else:
            return np.zeros((3,))
        force_1 = -coeff * ((1 / obstacle_dist) - (1 / bound)) * (1 / (obstacle_dist ** 2)) * ((source - target) ** self._params.power_factor) * ((source - obstacle) / obstacle_dist)
        force_2 = (-self._params.power_factor / 2) * coeff * (((1 / obstacle_dist) - (1 / bound)) ** 2) * ((source - target) ** (self._params.power_factor - 1)) * ((source - target) / target_dist)
        return force_1 + force_2

    def _get_attractive_force(self, source, target):
        return self._params.k_attraction * (target - source)
    
    def _get_artificial_force(self, source, target, obstacles):
        attractive_force = self._get_attractive_force(source, target)
        repulsive_force = np.sum(np.array([self._get_repulsive_force(source, target, obstacle).tolist() for obstacle in obstacles]), 0)
        return attractive_force + repulsive_force

    def _step(self):
        for drone in self._sources:
            if drone not in self._reached:
                obstacles = [deepcopy(self._obstacles[obstacle]) for obstacle in self._obstacles if obstacle != drone]
                obstacles.extend([deepcopy(self._client.drones[obstacle].position).to_numpy_array() for obstacle in self._targets if obstacle != drone])
                current_position = deepcopy(self._client.drones[drone].position).to_numpy_array()
                artificial_force = self._get_artificial_force(current_position, deepcopy(self._targets[drone]), obstacles)
                next_position = airsim.Vector3r(*(current_position + ((artificial_force / np.linalg.norm(artificial_force)) * self._params.step_length)))
                self._client.drones[drone].position = next_position
                if np.linalg.norm(deepcopy(next_position).to_numpy_array() - deepcopy(self._targets[drone])) < self._params.d_threshold:
                    self._client.drones[drone].position = airsim.Vector3r(*self._targets[drone])
                    self._reached |= {drone}
        self._target_exchange()

    def _target_exchange(self, debug=True):
        if debug: all_exchanges = []
        for drone in self._sources:
            if drone not in self._reached:
                current_position = deepcopy(self._client.drones[drone].position).to_numpy_array()
                current_target = deepcopy(self._targets[drone])
                count, doi = 0, set()
                for reached_drone in self._reached:
                    repulsive_force = self._get_repulsive_force(current_position, current_target, deepcopy(self._client.drones[reached_drone].position).to_numpy_array())
                    if any(repulsive_force):
                        if self._similar_swaps:
                            if self._client.drones[reached_drone].is_charging() == self._client.drones[drone].is_charging():
                                count += 1
                                doi |= {reached_drone}
                        else:
                            count += 1
                            doi |= {reached_drone}
                if count >= 2 and np.linalg.norm(current_position - current_target) > np.linalg.norm(deepcopy(self._client.drones[drone].previous_position).to_numpy_array() - current_target):
                    to_exchange = None
                    min_dist = np.inf
                    for possible_exchange in doi:
                        if np.linalg.norm(deepcopy(self._client.drones[possible_exchange].position).to_numpy_array() - current_target) < np.linalg.norm(current_position - current_target):
                            dist = np.linalg.norm(current_position - deepcopy(self._client.drones[possible_exchange].position).to_numpy_array())
                            if dist < min_dist:
                                to_exchange = possible_exchange
                                min_dist = dist
                    if to_exchange:
                        self._targets[drone] = deepcopy(self._targets[to_exchange])
                        self._targets[to_exchange] = deepcopy(current_target)
                        self._reached -= {to_exchange}
                        if debug: all_exchanges.append((drone, to_exchange))
        if debug and all_exchanges: print(all_exchanges)

    def setup(self, sources, targets, obstacles):
        self._sources = sources
        self._targets = targets
        self._obstacles = {obstacle: deepcopy(self._client.drones[obstacle].position).to_numpy_array() for obstacle in obstacles}
        self._reached = set()

    def run_update_loop(self):
        step_counter, no_change_counter, prev_num_reached = 0, 0, 0
        while len(self._reached) != len(self._sources.keys()):
            self._step()
            if len(self._reached) == prev_num_reached:
                no_change_counter += 1
            else:
                prev_num_reached = len(self._reached)
                no_change_counter = 0
            if no_change_counter == self._no_change_threshold - 1:
                print('Invoke Traffic Manager -> Let it do its job')
                prev_num_reached = len(self._reached)
                no_change_counter = 0
            print('Step #:', step_counter, '| # Reached:', len(self._reached), '| # Drones in Client:', len(self._client.drones.keys()))
            step_counter += 1
