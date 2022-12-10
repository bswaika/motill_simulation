import argparse
from scenarios import init, motill_scenario, stag_scenario, failure_scenario

parser = argparse.ArgumentParser()

parser.add_argument('--scenario', type=str, choices=['motill', 'stag', 'failure'], default='motill')
parser.add_argument('--host', type=str, default=None)

scenarios = {
    'motill': motill_scenario,
    'stag': stag_scenario,
    'failure': failure_scenario
}

if __name__ == '__main__':
    args = parser.parse_args()
    scenes, client, planner = init(args.host)
    scenarios[args.scenario](scenes, client, planner)