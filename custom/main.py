import numpy as np
from blocklyTranslations import *
from custom.droneManagement import DroneManagement
from wand_logger import WandFollower
import time

from action_detector import *
import queue
   
def main():
    sim = False
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
    p2_crazyflies = SimpleNamespace(crazyflies=crazyflies[4:8], timeHelper=timeHelper)
    # Start Wand Follower Nodes
    p1_wand_node = WandFollower(p1_crazyflies, timeHelper, sim=sim, player=1)
    p2_wand_node = WandFollower(p2_crazyflies, timeHelper, sim=sim, player=2)
    # Start Drone Management Nodes 
    p1_dm = DroneManagement(p1_crazyflies, timeHelper, sim=sim, player=1)
    p2_dm = DroneManagement(p2_crazyflies, timeHelper, sim=sim, player=2)

    takeoff(groupState, 1.0, 3)
    timeHelper.sleep(3.0)

    rclpy.spin(p1_wand_node)
    rclpy.spin(p2_wand_node)
    rclpy.spin(p1_dm)
    rclpy.spin(p2_dm)
    
    # Game loop
    p1 = True 
    p2 = True
    while p1 and p2:
      time = time.time()
      # Handle Player 1
      p1 = p1_dm.handle_player(time)
      # Handle Player 2
      p2 = p2_dm.handle_player(time)
      
    print("Game over: " + ("player 1 " if p1 else "player 2 ") + "wins!")
    land(groupState, 0.01, 3)
    timeHelper.sleep(3.0)

    if sim:
      fig = plt.figure()
      ax = fig.add_subplot(111, projection='3d')
      ax.set_xlim([-3, 3])
      ax.set_ylim([-3, 3])
      ax.set_zlim([0, 3])
      ax.set_xlabel("X")
      ax.set_ylabel("Y")
      ax.set_zlabel("Z")
      ax.view_init(20, 20)

      xs = [p[0] for p in wand_node.states]
      ys = [p[1] for p in wand_node.states]
      zs = [p[2] for p in wand_node.states]

      ax.plot(xs, ys, zs, label='crazyflie')

      xs = [p[0][0] for p in wand_node.goals]
      ys = [p[0][1] for p in wand_node.goals]
      zs = [p[0][2] for p in wand_node.goals]

      ax.plot(xs, ys, zs, label='wand')
      plt.legend()
      plt.show()
main()
