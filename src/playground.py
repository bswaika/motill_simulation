import sys

from Client import Client
from MotillOrchestrator import MotillOrchestrator
from StagOrchestrator import StagOrchestrator
from FailureOrchestrator import FailureOrchestrator
from PathPlanners import APFPathPlanner
from read_data import read_data

if __name__ == '__main__':
    scenes = read_data()
    client = Client(ip=sys.argv[1])
    planner = APFPathPlanner(client)
    # orchestrator = MotillOrchestrator(client, {'static': scenes['motill_static'], 'motion': scenes['motill_motion']}, 1)
    # orchestrator.run()
    orchestrator = StagOrchestrator(client, scenes['stag_static'], planner, delay=1)
    orchestrator.run()
    # orchestrator = FailureOrchestrator(client, scenes['motill_static'].voxel_downsample(1.7), planner, delay=1)
    # orchestrator = FailureOrchestrator(client, scenes['stag_static_3'], planner, delay=1)
    # orchestrator.run()