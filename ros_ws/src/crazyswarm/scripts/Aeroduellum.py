import numpy as np
from blocklyTranslations import *
import matplotlib.pyplot as plt
import argparse

Hz = 20

def build_argparser(parent_parsers=[]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        parents=parent_parsers
    )
    parser.add_argument("--sim", help="Run using simulation.", action="store_true")

    return parser

# Default drone positions for user's three drones when not in use x, y, z 
default_poses = np.array([[0., -1., 1.],
                          [0.,  0., 1.],
                          [0.,  1., 1.]])

drone_available = [False, False, False]

def gotoDefault(groupState):
    crazyflies = groupState.crazyflies
    timeHelper = groupState.timeHelper
    max_duration = 0
    v = 1.5
    for i, cf in enumerate(crazyflies):
        # try:
        #     if not isinstance(cf, CrazyflieSimLogger):
        #         rclpy.spin_once(cf.node)
        # except Exception:
        #     cf.node.get_logger().error(traceback.format_exc())
        #     pass
        curr_pos = cf.position()
        dist = np.linalg.norm(np.array(curr_pos) - default_poses[i])
        duration = dist / v
        cf.goTo(default_poses[i], 0, duration=duration, relative=False)
        max_duration = max(duration, max_duration)
    timeHelper.sleep(max_duration)

def quick_attack(spell_group):
    return

def heavy_attack(spell_group):
    return

def shield(familiar):
    return

def play(groupState):
    crazyflies = groupState.crazyflies
    timeHelper = groupState.timeHelper
    all_positions = []

    while(True):
        # Get input
        input = 0
        # parse instructions
            # 0 - quick attack
        spell = input
        # check drone states
            # If drones are ready
        # send instructions to each drone async
        if spell == 0: # Quick attack
            quick_attack(spell_group_1)
        elif spell == 1: # Heavy Attack
            heavy_attack(spell_group_1)
        elif spell == 2: # Shield 
            shield(familiar_1)

        # update drone states






def main():
    parser = build_argparser()
    args, unknown = parser.parse_known_args()
    sim = args.sim

    if sim:
        # Use sim version of crazyswarm
        from pycrazyswarm import Crazyswarm
        swarm = Crazyswarm()
        crazyflies = swarm.allcfs.crazyflies
        timeHelper = swarm.timeHelper

        groupState = SimpleNamespace(crazyflies=crazyflies, timeHelper=timeHelper)
    else:
        # Use ROS and Crazyswarm2
        from crazyflie_py import Crazyswarm
        swarm = Crazyswarm()
        crazyflies = swarm.allcfs.crazyflies
        timeHelper = swarm.timeHelper

        groupState = SimpleNamespace(crazyflies=crazyflies, timeHelper=timeHelper) 
    
    takeoff(groupState, height=1, duration=3)
    gotoDefault(groupState)
    play(groupState)
    land(groupState, 0, 3)

if __name__ == '__main__':
    main()