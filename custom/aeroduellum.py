import numpy as np
from blocklyTranslations import *
import matplotlib.pyplot as plt
import argparse

class Aeroduellum(Node):
  def __init__(self, groupState, timeHelper, max_speed=0.5, update_frequency=20, sim=True):
    super().__init__('drone_management')
    self.crazyflies = groupState.crazyflies[:] # familiar crazyflie is 0, rest are spell
    self.timeHelper = groupState.timeHelper
    self.max_speed = max_speed
    self.Hz = update_frequency
    # self.crazyflie = crazyflie
    self.timeHelper = timeHelper
    self.status = np.zeros(len(self.crazyflies)) # 0 is available, 1 is busy
    

    # create subscriptions
    self.spell_subscriber = self.create_subscription(Int32, 'spell', self.spell_callback, 1)
    # self.enemy_attack_subscriber = self.create_subscription(Int32, 'attack', self.enemy_attack_callback, 1)
    self.call_timer = self.create_timer(1/self.Hz, self.timer_cb)

    # Create publisher for publishing attacks to other player
    # self.attack_publisher = self.create_publisher(Int32, 'attack', )

    # We'll use known-to-work parameters for safety, but you can alter for sim
    self.controller = PDController(1, 0.05)

    # Lists for plotting
    self.states = []
    self.goals = []

  def timer_cb(self):
    """
    Executes every time a timer is triggered (rate based on Hz)
    """
    # TODO update this
    # Get state of wand
    wand_position, wand_rotation = self.wand_pose


  def send_vel_cmd(self, vel):
    """
    send velocity-style command to crazyflie
    Args:
      vel: (array-like of float[3]): Velocity meters/second
    """
    pos = self.crazyflie.position()
    desired_position = pos + np.array(vel)*1/self.Hz
    self.crazyflie.cmdPosition(desired_position)

    # Required for sim updates, not necessary for real world
    self.timeHelper.sleepForRate(self.Hz*2)

    self.states.append(pos)
    self.goals.append(self.wand_pose)

  def spell_callback(self, msg):
    """
    Spell callback method, called everytime a message is published to the topic /spell
    triggers behavior corresponding to received spell command
    """

    if msg.data == 0: # defend
        # If familiar is available, set defense spell flag to be triggered in loop
        if self.status[0] == 0:
           self.defense_flag = True
    elif msg.data == 1: # quick attack
        # If a spell drone is available, set quick attack flag to be triggered in main loop
        if sum(self.status[1:]) >= 1:
           self.quick_attack_flag = True
    elif msg.data == 2: # heavy attack
        if sum(self.status[1:]) >= 3:
           self.heavy_attack_flag = True
    

  def enemy_attack_callback(self, msg):
    """
    Shutdown when button on game pad is pressed
    """
    if msg.buttons[5] == 1: # Button was pressed
      self.cf.notifySetpointsStop()
      self.cf.land(0., 3)

      rclpy.shutdown() # destory node