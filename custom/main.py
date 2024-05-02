import numpy as np
from blocklyTranslations import *
from custom.droneManagement import DroneManagement
from wand_logger import WandFollower
import time

from action_detector import *
import queue

def shield(player):
   return
def quick_attack(player):
   return
def heavy_attack(player):
   return


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
    p1_drone_management = Aeroduellum(p1_crazyflies, timeHelper, sim=sim, player=1)
    p2_drone_management = Aeroduellum(p2_crazyflies, timeHelper, sim=sim, player=2)

    takeoff(groupState, 1.0, 3)
    timeHelper.sleep(3.0)

    rclpy.spin(p1_wand_node)
    rclpy.spin(p2_wand_node)
    rclpy.spin(p1_drone_management)
    rclpy.spin(p2_drone_management)
    
    # Game loop
    shield_duration = 5 # 5 seconds, TODO change this to reflect actual length of shield spell through cooldown
    quick_attack_duration = 3
    heavy_attack_duration = 6

    p1 = 0
    p2 = 1

    shielding = [False, False]
    quick_attacking = [False, False]
    heavy_attacking = [False, False]
    quick_attack_drones = [[][]]
    heavy_attack_drones = []
    shield_end_time = [0, 0]
    quick_attack_end_time = [0, 0]
    heavy_attack_end_time = [0, 0]

    while True:
      time = time.time()
      # Handle Player 1
      if p1_drone_management.shield_flag == True: # Cast Shield
        # start shield movement and set timer for shield to sleep
        p1_drone_management.shield_flag = False
        p1_drone_management.status[0] = 0
        # shield() call command to have familiar drone enact shield behavior
        shield(p1)
        shielding[p1] = True
        shield_end_time[p1] = time + shield_duration
        
        # set timer 
      if shielding[p1] and time >= shield_end_time[p1]: # Reset shield
        shielding[p1] = False
        p1_drone_management.status[0] = 1

      # TODO maybe update this to allow for multiple quick attacks in series while the previous one is cooling down
      if p1_drone_management.quick_attack_flag == True: # Cast Quick Attack
        # start quick attack  movement and set timer for shield to sleep
        p1_drone_management.quick_attack_flag = False
        # select available drone
        quick_attack_drones[p1] = np.where(p1_drone_management.status[1:] == 1)[0]
        if len(quick_attack_drones[p1] < 1):
          # Error, not enough drones
          print("Error: not enough drones available")
        else:
          p1_drone_management.status[quick_attack_drones] = 0 # TODO make sure these indices account for the slice
        quick_attack_end_time[p1] = time + quick_attack_duration
        
      if quick_attacking[p1] and time >= quick_attack_end_time[p1]: # Reset quick attack
        quick_attacking[p1] = False
        p1_drone_management.status[quick_attack_drones] = 1
        quick_attack_drones[p1] = []
      


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
