import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt

def get_attractive_force(drone_loc, goal_loc, attraction_bound, attraction_coeff):
    drone, goal = np.array(drone_loc), np.array(goal_loc)
    dist = np.linalg.norm(goal - drone, 2)
    force = attraction_coeff * (goal - drone)
    return force if (dist < attraction_bound) or (attraction_bound == 0) else (force * (attraction_bound / dist))

def get_repulsive_force(drone_loc, goal_loc, obstacle_loc, inner_repulsion_bound, outer_repulsion_bound, inner_repulsion_coeff, outer_repulsion_coeff, power_factor):
    drone, obstacle, goal = np.array(drone_loc), np.array(obstacle_loc), np.array(goal_loc)
    dist, goal_dist = np.linalg.norm(drone - obstacle, 2), np.linalg.norm(drone - goal, 2)
    if dist <= inner_repulsion_bound:
        coeff = inner_repulsion_coeff
        bound = inner_repulsion_bound
    elif dist <= outer_repulsion_bound and dist > inner_repulsion_bound:
        coeff = outer_repulsion_coeff
        bound = outer_repulsion_bound
    else:
        coeff = 0
        bound = inner_repulsion_bound
    force_1 = -coeff * ((1 / dist) - (1 / bound)) * (1 / (dist ** 2)) * ((drone - goal) ** power_factor) * ((drone - obstacle) / dist)
    force_2 = (-power_factor / 2) * coeff * (((1 / dist) - (1 / bound)) ** 2) * ((drone - goal) ** (power_factor - 1)) * ((drone - goal) / goal_dist)
    return force_1 + force_2

def get_artificial_force(drone_loc, goal_loc, obstacles_loc, params):
    attraction_bound, attraction_coeff = params['attraction_bound'], params['attraction_coeff']
    inner_repulsion_bound, outer_repulsion_bound, inner_repulsion_coeff, outer_repulsion_coeff, power_factor = params['inner_repulsion_bound'], params['outer_repulsion_bound'], params['inner_repulsion_coeff'], params['outer_repulsion_coeff'], params['power_factor']

    attractive_force = get_attractive_force(drone_loc, goal_loc, attraction_bound, attraction_coeff)
    repulsive_forces = np.array([get_repulsive_force(drone_loc, goal_loc, obstacle_loc, inner_repulsion_bound, outer_repulsion_bound, inner_repulsion_coeff, outer_repulsion_coeff, power_factor).tolist() for obstacle_loc in obstacles_loc])
    repulsive_force = np.sum(repulsive_forces, 0)

    return attractive_force + repulsive_force

if __name__ == '__main__':
    PARAMS = {
        'attraction_bound': 0,
        'attraction_coeff': 200,
        'inner_repulsion_bound': 5,
        'outer_repulsion_bound': 15,
        'inner_repulsion_coeff': 100,
        'outer_repulsion_coeff': 5,
        'power_factor': 3,
        'step_length': 0.001
    }

    with open('./apf_test/start.txt', 'r') as infile:
        drones = list(map(lambda x: tuple(map(float, x.split(','))), infile.readlines()))
    
    with open('./apf_test/end.txt', 'r') as infile:
        goals = list(map(lambda x: tuple(map(float, x.split(','))), infile.readlines()))

    # print(drones)
    # print(goals)

    # drones = [(40, 35, 10), (55, 30, 10), (60, 35, 10), (20, 25, 10), (50, 35, 10), (25, 25, 10), (20, 40, 10), (50, 45, 10), (25, 45, 10), (20, 30, 10)]
    drones_set = set(drones)
    # goals = [(42, 3, 10), (39, 3, 10), (48, 6, 10), (45, 6, 10), (36, 6, 10), (51, 9, 10), (33, 9, 10), (51, 12, 10), (33, 12, 10), (51, 15, 10)]
    step_length = PARAMS['step_length']

    steps = []
    locs = deepcopy(drones)
    for i in range(200):
        next_steps = []
        for drone, goal in zip(locs, goals):
            obstacles = drones_set - {drone}
            force = get_artificial_force(drone, goal, list(obstacles), PARAMS)
            next_step = tuple((np.array(drone) + np.clip(step_length * force, -2, 2)).tolist())
            # drones_set |= {next_step}
            next_steps.append(next_step)
        steps.append(next_steps)
        locs = next_steps
        drones_set = set(locs)
    
    paths = [[drone] for drone in drones]
    
    for step in steps:
        for idx, drone in enumerate(step):
            paths[idx].append(drone)

    plt.scatter(list(map(lambda x: x[0], drones)), list(map(lambda x: x[1], drones)), c='r', s=50, marker='^')
    reached = 0
    for i, path in enumerate(paths):
        if np.linalg.norm(np.array(path[-1]) - np.array(goals[i])) < 0.1:
            reached += 1
            plt.plot(list(map(lambda x: x[0], path)), list(map(lambda x: x[1], path)))
    plt.scatter(list(map(lambda x: x[0], goals)), list(map(lambda x: x[1], goals)), c='g', s=50, marker='^')

    print('# reached:', reached)
    plt.show()