import sys

from Client import Client
from MotillOrchestrator import MotillOrchestrator
from StagOrchestrator import StagOrchestrator
from FailureOrchestrator import FailureOrchestrator
from PathPlanners import APFPathPlanner
from read_data import read_data

def init(ip=None):
    scenes = read_data()
    client = Client(ip)
    planner = APFPathPlanner(client)
    return scenes, client, planner

def motill_scenario(scenes, client, planner):
    orchestrator = MotillOrchestrator(client, {'static': scenes['motill_static'], 'motion': scenes['motill_motion']}, 1)
    orchestrator.run()

def stag_scenario(scenes, client, planner):
    orchestrator = StagOrchestrator(client, scenes['stag_static'], planner, delay=1)
    orchestrator.run()

def failure_scenario(scenes, client, planner):
    orchestrator = FailureOrchestrator(client, scenes['stag_static_3'], planner, delay=1)
    orchestrator.run()