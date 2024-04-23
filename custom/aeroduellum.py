import numpy as np
from blocklyTranslations import *
import matplotlib.pyplot as plt
import argparse
from std_msgs.msg import String


class Aeroduellum(Node):
  def __init__(self, groupState):
    super().__init__('drone_management')
    self.crazyflies = groupState.crazyflies[:] # familiar crazyflie is 0, rest are spell

    self.status = np.zeros(len(self.crazyflies)) # 0 is available, 1 is busy
    self.shield_flag = False
    self.quick_attack_flag = False
    self.heavy_attack_flag = False
    

    # create subscriptions
    self.spell_subscriber = self.create_subscription(String, 'spell', self.spell_callback, 1)
    # self.enemy_attack_subscriber = self.create_subscription(Int32, 'attack', self.enemy_attack_callback, 1)
    self.call_timer = self.create_timer(1/self.Hz, self.timer_cb)

    # Create publisher for publishing attacks to other player
    # self.attack_publisher = self.create_publisher(Int32, 'attack', )

    # We'll use known-to-work parameters for safety, but you can alter for sim
    self.controller = PDController(1, 0.05)

    # Lists for plotting
    self.states = []
    self.goals = []

  def spell_callback(self, msg):
    """
    Spell callback method, called everytime a message is published to the topic /spell
    triggers behavior corresponding to received spell command
    """

    if msg.data == 'detectRotateSide': # defend
        # If familiar is available, set defense spell flag to be triggered in loop
        if self.status[0] == 0:
           self.defense_flag = True
    elif msg.data == 'detectFastAttack': # quick attack
        # If a spell drone is available, set quick attack flag to be triggered in main loop
        if sum(self.status[1:]) >= 1:
           self.quick_attack_flag = True
    elif msg.data == 'detectChargedAttack': # heavy attack
        if sum(self.status[1:]) >= 3:
           self.heavy_attack_flag = True
    

  def enemy_attack_callback(self, msg):
    """
    Shutdown when button on game pad is pressed
    """
    # TODO set flag for familiar stagger behavior 
    return