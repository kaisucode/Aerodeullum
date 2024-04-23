import numpy as np
from blocklyTranslations import *
from aeroduellum import Aeroduellum
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

    allcfs = swarm.allcfs
    cf = allcfs.crazyflies[0]
    wand_node = WandFollower(groupState, timeHelper, sim=sim)
    drone_management = Aeroduellum(groupState, timeHelper, sim=sim)
    takeoff(groupState, 1.0, 3)
    timeHelper.sleep(3.0)

    rclpy.spin(wand_node)
    rclpy.spin(drone_management)
    
    # Game loop
    shield_duration = 5 # 5 seconds, TODO change this to reflect actual length of shield spell through cooldown
    quick_attack_duration = 3
    heavy_attack_duration = 6
    shielding = False
    quick_attacking = False
    heavy_attacking = False
    quick_attack_drones = []
    heavy_attack_drones = []

    
    prev_time = time.time()
    while True:
      time = time.time()

      if drone_management.shield_flag == True: # Cast Shield
        # start shield movement and set timer for shield to sleep
        drone_management.shield_flag = False
        drone_management.status[0] = 0
        # shield() call command to have familiar drone enact shield behavior
        shield_end_time = time + shield_duration
        
        # set timer 
      if shielding and time >= shield_end_time: # Reset shield
        shielding = False
        drone_management.status[0] = 1

      # TODO maybe update this to allow for multiple quick attacks in series while the previous one is cooling down
      if drone_management.quick_attack_flag == True: # Cast Shield
        # start shield movement and set timer for shield to sleep
        drone_management.quick_attack_flag = False
        # select available drone
        quick_attack_drones = np.where(drone_management.status[1:] == 1)[0]
        if len(quick_attack_drones < 1):
          # Error, not enough drones
          print("Error: not enough drones available")
        else:
          drone_management.status[quick_attack_drones] = 0 # TODO make sure these indices account for the slice
        quick_attack_end_time = time + quick_attack_duration
        
      if quick_attacking and time >= quick_attack_end_time: # Reset quick attack
        quick_attacking = False
        drone_management.status[quick_attack_drones] = 1
        quick_attack_drones = []
      


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
