import numpy as np
from blocklyTranslations import *
import matplotlib.pyplot as plt
import argparse
from rclpy.node import Node
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String


class DroneManagement(Node):
  def __init__(self, groupState, player):
    super().__init__('drone_management')
    self.crazyflies = groupState.crazyflies[:] # familiar crazyflie is 0, rest are spell
    self.player = player
    self.status = np.zeros(len(self.crazyflies)) # 0 is available, 1 is busy
    self.shield_flag = False
    self.quick_attack_flag = False
    self.heavy_attack_flag = False
    
    self.shield_duration = 6        # TODO change this to reflect actual length of shield spell through cooldown
    self.quick_attack_duration = 5
    self.heavy_attack_duration = 10
    self.hp = 100
    self.quick_attack_damage = 10
    self.heavy_attack_damage = 50

    self.shielding = False
    self.quick_attacking = False
    self.heavy_attacking = False
    self.quick_attack_drones = []
    self.heavy_attack_drones = []
    self.shield_end_time = 0
    self.quick_attack_end_time = 0
    self.heavy_attack_end_time = 0

    # Create publishers
    self.damage_pub = rospy.Publisher("damage" + self.player, Int32, queue_size=10)

    # Create Subscribers
    self.spell_subscriber = self.create_subscription(String, 'spell'+self.player, self.spell_callback, 1)
    self.damage_subscriber = self.create_subscription(Int32, "damage" + (1 if self.player == 0 else 0), self.damage_callback, 1)

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

  def damage_callback(self, msg):
      if not self.shielding:
        self.hp -= msg.data
        # TODO change familiar HP light color
        # TODO stagger
        # TODO check if hp goes below 0, if so end game
  
  def shield(self, time):
    if self.status[0] == 0:
      return False
    self.shield_flag = False
    self.status[0] = 0
    # shield() call command to have familiar drone enact shield behavior
    self.cast_shield(self.groupState)
    self.shielding = True
    self.shield_end_time = time + self.shield_duration
    return True
  
  def quick_attack(self, time):
    self.quick_attack_flag = False
    # select available drone
    self.quick_attack_drones = np.where(self.status[1:] == 1)[0]
    if len(self.quick_attack_drones < 1):
      # Error, not enough drones
      print("Error: not enough drones available")
    else:
      self.cast_quick_attack(self.groupState)
      self.status[self.quick_attack_drones] = 0 # TODO make sure these indices account for the slice
    self.quick_attack_end_time = time + self.quick_attack_duration

  def heavy_attack(self, time):
    self.heavy_attack_flag = False
    # select available drone
    self.heavy_attack_drones = np.where(self.status[1:] == 1)[0:3]
    if len(self.heavy_attack_drones < 3):
      # Error, not enough drones
      print("Error: not enough drones available")
    else:
      self.cast_heavy_attack(self.groupState)
      self.status[self.heavy_attack_drones] = 0 # TODO make sure these indices account for the slice
    self.heavy_attack_end_time = time + self.heavy_attack_duration

  # Trigger shield movement behavior
  def cast_shield(self, groupState):
     return

  # Trigger quick_attack movement behavior
  def cast_quick_attack(self, groupState):
     return
  
  # Trigger quick_attack movement behavior
  def cast_heavy_attack(self, groupState):
     # TODO select drones from heavy_attack_drones and trigger behavior
     return
     

  
  def handle_player(self, time):
    # Handle losing
    if self.hp <= 0:
       print("player " + self.player + " loses")
       return False
    
    # Handle Shielding
    if self.shield_flag == True: # Cast Shield
      self.shield(time)
    if self.shielding and time >= self.shield_end_time: # Reset shield
      self.shielding = False
      self.status[0] = 1

    #Handle Quick Attacks
    # TODO maybe update this to allow for multiple quick attacks in series while the previous one is cooling down
    if self.quick_attack_flag == True: # Cast Quick Attack
      # start quick attack  movement and set timer for shield to sleep
      self.quick_attack(time)
    if self.quick_attacking and time >= self.quick_attack_end_time: # Reset quick attack
      self.damage_pub.publish(self.quick_attack_damage)
      self.quick_attacking = False
      self.status[self.quick_attack_drones] = 1
      self.quick_attack_drones = []

    # Handle Heavy Attacks
    if self.heavy_attack_flag == True: # Cast Quick Attack
      # start quick attack  movement and set timer for shield to sleep
      self.heavy_attack(time)
    if self.heavy_attacking and time >= self.heavy_attack_end_time: # Reset quick attack
      self.damage_pub.publish(self.heavy_attack_damage)
      self.heavy_attacking = False
      self.status[self.heavy_attack_drones] = 1
      self.heavy_attack_drones = []

    # Return true if game isn't over
    return True
  

