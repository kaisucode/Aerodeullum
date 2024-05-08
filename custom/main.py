import numpy as np
from blocklyTranslations import *
from droneManagement import DroneManagement
from wand_logger_with_drone import WandFollower
import time

from action_detector import *
import queue
   
def main():
    sim = False
    multiplayer = True
    if sim:
        # Use sim version of crazyswarm
        from pycrazyswarm import Crazyswarm
        # from ros_sim import Node, TFMessage, rclpy, Joy
        rclpy = rclpy_sim
        swarm = Crazyswarm(args='--vis=null --sim')
        crazyflies = swarm.allcfs.crazyflies
        timeHelper = swarm.timeHelper

        groupState = SimpleNamespace(crazyflies=crazyflies, timeHelper=timeHelper)
    else:
        # Use real ROS
        from tf2_msgs.msg import TFMessage
        from rclpy.node import Node
        import rclpy
        from crazyflie_py import Crazyswarm
        from sensor_msgs.msg import Joy
        swarm = Crazyswarm()
        crazyflies = swarm.allcfs.crazyflies
        timeHelper = swarm.timeHelper
        
        groupState = SimpleNamespace(crazyflies=crazyflies, timeHelper=timeHelper)

    # Create groups for each player's drones
    p1_crazyflies = SimpleNamespace(crazyflies=crazyflies[0:4], timeHelper=timeHelper)
    # Start Wand Follower Nodes
    p1_wand_node = WandFollower(p1_crazyflies, timeHelper, player=1)
    # Start Drone Management Nodes 
    p1_dm = DroneManagement(p1_crazyflies, player=1)
    if multiplayer:
        p2_crazyflies = SimpleNamespace(crazyflies=crazyflies[4:8], timeHelper=timeHelper)
        p2_wand_node = WandFollower(p2_crazyflies, timeHelper, player=2)
        p2_dm = DroneManagement(p2_crazyflies, player=2)

    takeoff(groupState, 1.0, 3)
    timeHelper.sleep(3.0)

    for idx in range(4): 
        p1_dm.initialize_drone_position(p1_dm.groupState, idx, 1)
        if multiplayer:
            p2_dm.initialize_drone_position(p2_dm.groupState, idx, 2)
    
    # Game loop
    p1 = True 
    p2 = True
    max_time = time.time() + 120
    while p1 and p2 and time.time() < max_time:
      cur_time = time.time()
      # Handle Player 1
      rclpy.spin_once(p1_wand_node)
      rclpy.spin_once(p1_dm)
      p1 = p1_dm.handle_player(cur_time)
      if multiplayer:
          # Handle Player 2
          rclpy.spin_once(p2_wand_node)
          rclpy.spin_once(p2_dm)
          p2 = p2_dm.handle_player(cur_time)
      
    print("Game over: " + ("player 1 " if p1 else "player 2 ") + "wins!")
    land(groupState, 0.01, 3)
    timeHelper.sleep(3.0)

main()
