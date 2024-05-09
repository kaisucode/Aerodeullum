import numpy as np
from blocklyTranslations import *
import matplotlib.pyplot as plt
import argparse
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
import time

from pathlib import Path
from crazyflie_py.uav_trajectory import Trajectory



dronePositions = [
    [[-4, 2, 1], [-3, -2, 1], [-3, -1.5, 1.5], [-3, -1, 1]],
    [[4, -1.5, 1], [3, 2.5, 1], [3, 2, 1.5], [3, 1.5, 1]],
]

trajectoryNames = ["spiral", "helix1", "helix2", "helix3", "familiar", "single_shield", "straight"]

def loadTrajectories():
    trajectoryFilemapping = {}  # {"name": {"trajectory", "id"}}
    trajId = 0
    for fileprefix in trajectoryNames:
        
        for player in ["p1_", "p2_"]: 
            trajName = player + fileprefix
            trajectoryFilemapping[trajName] = {"id": trajId, "trajectory": Trajectory()}
            filename = "aero/" + trajName + ".csv"
            trajectoryFilemapping[trajName]["trajectory"].loadcsv(Path(__file__).parent / filename)
            trajId += 1
    return trajectoryFilemapping


class DroneManagement(Node):
    def __init__(self, groupState, player):
        super().__init__("drone_management" + str(player))
        self.crazyflies = groupState.crazyflies[
            :
        ]  # familiar crazyflie is 0, rest are spell
        self.player = player
        self.familiar_status = 1
        self.status = np.ones(3)  # 0 is available, 1 is busy
        self.shield_flag = False
        self.quick_attack_flag = False
        self.heavy_attack_flag = False
        self.groupState = groupState
        #  self.max_game_duration = 4 * 60 # set in main instead
        self.color = False

        # load trajectories based on csv files, and upload to the drones
        # key: numeric id, value: trajectory
        self.trajectoryFilemapping = loadTrajectories()
        for cf in self.crazyflies:
            for fileprefix in self.trajectoryFilemapping:
                trajectoryId = self.trajectoryFilemapping[fileprefix]["id"]
                cf.uploadTrajectory(
                    trajectoryId,
                    0,
                    self.trajectoryFilemapping[fileprefix]["trajectory"],
                )

        # Set default behavior durations TODO Remove if trajectory timing works
        self.shield_duration = 6
        self.protected_duration = 3
        self.stagger_duration = 3
        self.quick_attack_duration = 5
        self.heavy_attack_duration = 10
        # Durations of attack parts of spells without return time
        self.quick_damage_duration = 7#.1
        self.heavy_damage_duration = 12#.4


        # Compute actual trajectory durations
        self.shield_duration = self.trajectoryFilemapping["p1_single_shield"]["trajectory"].duration
        self.quick_attack_duration = self.trajectoryFilemapping["p1_spiral"]["trajectory"].duration
        self.heavy_attack_duration = max([self.trajectoryFilemapping["p1_helix1"]["trajectory"].duration, 
                                           self.trajectoryFilemapping["p1_helix2"]["trajectory"].duration,
                                           self.trajectoryFilemapping["p1_helix3"]["trajectory"].duration])
        self.stagger_duration = self.trajectoryFilemapping["p1_familiar"]["trajectory"].duration
        
        # Gameplay statistics 
        self.hp = 100
        self.quick_attack_damage = 10
        self.heavy_attack_damage = 50

        # Game Variables
        self.shielding = False
        self.quick_attacking = False
        self.heavy_attacking = False
        self.staggered = False
        self.quick_attack_drones = []
        self.heavy_attack_drones = []
        self.shield_end_time = 0
        self.protection_end_time = 0
        self.quick_attack_end_time = 0
        self.heavy_attack_end_time = 0
        self.stagger_end_time = 0
        self.quick_damage_inflict_time = 0
        self.heavy_damage_inflict_time = 0

        # Create publishers
        # self.damage_pub = rospy.Publisher("damage" + self.player, Int32, queue_size=10)
        self.damage_pub = self.create_publisher(Int32, "damage" + str(self.player), 10)

        # Create Subscribers
        self.spell_subscriber = self.create_subscription(
            String, "spell" + str(self.player), self.spell_callback, 1
        )
        self.damage_subscriber = self.create_subscription(
            Int32,
            "damage" + ("2" if self.player == 1 else "1"),
            self.damage_callback,
            1,
        )
        self.Hz = 5
        self.call_timer = self.create_timer(1 / self.Hz, self.timer_cb)

    def timer_cb(self):
        curr_time = time.time()
        self.handle_player(curr_time)
        if self.hp <= 0:
            print("Game over: " + ("player 1 " if self.player == 2 else "player 2 ") + "wins!")
            self.destroy_node()
        if curr_time > self.max_time:
            print("Game over: time exceeded")
            self.destroy_node()


    def getTrajectory(self, trajName):
        return (
            self.trajectoryFilemapping[trajName]["id"],
            self.trajectoryFilemapping[trajName]["trajectory"],
        )

    def spell_callback(self, msg):
        """
        Spell callback method, called everytime a message is published to the topic /spell
        triggers behavior corresponding to received spell command
        """
        if msg.data == "detectRotateSide":  # defend
            # If familiar is available, set defense spell flag to be triggered in loop
            if self.familiar_status == 1 and not self.shielding and not self.staggered:
                self.defense_flag = True

        elif msg.data == "detectFastAttack":  # quick attack
            # If a spell drone is available, set quick attack flag to be triggered in main loop
            if sum(self.status[:]) >= 1 and not self.quick_attacking and not self.heavy_attacking and not self.heavy_attack_flag:
                self.quick_attack_flag = True
        elif msg.data == "detectChargedAttack":  # heavy attack
            if sum(self.status[:]) >= 3 and not self.quick_attacking and not self.heavy_attacking and not self.quick_attack_flag:
                self.heavy_attack_flag = True

    def damage_callback(self, msg):
        # if not self.shielding:
        curr_time = time.time()
        if curr_time > self.protection_end_time:
            self.hp -= msg.data
            print(
                "Player "
                + str(self.player)
                + " was struck for "
                + str(msg.data)
                + " damage! "
                + str(self.hp)
                + " HP remaining"
            )
            self.staggered = True
            self.stagger_end_time = curr_time + self.stagger_duration
            self.cast_stagger(self.groupState)
            # TODO change familiar HP light color
            # TODO check if hp goes below 0, if so end game
        else:
            print("Attack blocked!")

    def shield(self, time):
        print("trying to cast shield")
        if self.familiar_status == 0 or self.staggered or time < self.staggered:
            print("could not cast shield, status: ", self.familiar_status, " staggered: ", self.staggered)
            return False
        self.shield_flag = False
        self.familiar_status = 0
        self.cast_shield(self.groupState)
        self.shielding = True
        self.protection_end_time = time + self.protected_duration
        self.shield_end_time = time + self.shield_duration
        return True

    def quick_attack(self, time):
        print("trying to cast quick attack")
        self.quick_attack_flag = False
        # select available drone
        self.quick_attack_drones = np.where(self.status == 1)[0]
        print("quick attack drones ", self.quick_attack_drones)
        if len(self.quick_attack_drones) < 1 or self.heavy_attacking:
            print("Error: not enough drones available", self.status, ", ", self.heavy_attacking, "len: ", len(self.quick_attack_drones))
        else:
            self.cast_quick_attack(
                self.groupState, self.quick_attack_drones[0]
            )  # TODO pass in specific quick attack drone
            self.quick_attacking = True
            self.status[self.quick_attack_drones[0]] = 0
            self.quick_attack_end_time = time + self.quick_attack_duration
            self.quick_damage_inflict_time = time + self.quick_damage_duration

    def heavy_attack(self, time):
        print("trying to cast heavy attack")
        self.heavy_attack_flag = False
        # select available drone
        self.heavy_attack_drones = np.where(self.status == 1)[0]
        print("heavy attack drones ", self.heavy_attack_drones)
        if len(self.heavy_attack_drones) < 3 or self.quick_attacking:
            # Error, not enough drones
            print("Error: not enough drones available", self.status, ", ", self.quick_attacking, "len: ", len(self.heavy_attack_drones))
        else:
            self.cast_heavy_attack(self.groupState)
            self.heavy_attacking = True
            for drone in self.heavy_attack_drones:
                self.status[drone] = 0
            self.heavy_attack_end_time = time + self.heavy_attack_duration
            self.heavy_damage_inflict_time = time + self.heavy_damage_duration

    # Trigger shield movement behavior
    def cast_shield(self, groupState):
        print("Triggering shield motion")
        player_prefix = "p" + str(self.player) + "_"
        trajId, traj = self.getTrajectory(player_prefix + "single_shield")
        if self.color:
            setLEDColorFromHex(groupState.crazyflies[0], "#00ffff") #cyan
        groupState.crazyflies[0].startTrajectory(trajId, 1.0, False)
        # executeDuration = traj.duration
        # sleep for the above duration
        return

    # Trigger quick_attack movement behavior
    def cast_quick_attack(self, groupState, quick_attack_drone):
        print("Triggering quick attack motion")
        player_prefix = "p" + str(self.player) + "_"
        trajId, traj = self.getTrajectory(player_prefix + "spiral")
        if self.color:
            setLEDColorFromHex(groupState.crazyflies[quick_attack_drone + 1], "#7f00ff") #violet
        groupState.crazyflies[quick_attack_drone + 1].startTrajectory(trajId, 1.0, False)
        return

    # Trigger quick_attack movement behavior
    def cast_heavy_attack(self, groupState):
        print("Triggering heavy attack motion")
        player_prefix = "p" + str(self.player) + "_"
        trajId1, traj = self.getTrajectory(player_prefix + "helix1")
        trajId2, traj = self.getTrajectory(player_prefix + "helix2")
        trajId3, traj = self.getTrajectory(player_prefix + "helix3")
        if self.color:
            setLEDColorFromHex(groupState.crazyflies[1], "#ed2938") #red
            setLEDColorFromHex(groupState.crazyflies[2], "#ff8c01") #orange
            setLEDColorFromHex(groupState.crazyflies[3], "#ffe733") #yellow
        groupState.crazyflies[1].startTrajectory(trajId1, 1.0, False)
        groupState.crazyflies[2].startTrajectory(trajId2, 1.0, False)
        groupState.crazyflies[3].startTrajectory(trajId3, 1.0, False)
        return

    def cast_stagger(self, groupState):
        print("Triggering stagger")
        player_prefix = "p" + str(self.player) + "_"
        trajId, traj = self.getTrajectory(player_prefix + "familiar")
        if self.color:
            setLEDColorFromHex(groupState.crazyflies[0], "#ed2938") #red
        groupState.crazyflies[0].startTrajectory(trajId, 1.0, False)
        return

    def initialize_drone_position(self, groupState, droneIndex, player, max_time):
        side = player - 1
        groupState.crazyflies[droneIndex].goTo(
            np.asarray(dronePositions[side][droneIndex]), 0, 1.0
        )
        groupState.timeHelper.sleep(3)
        self.max_time = max_time 

    def handle_player(self, time):
        # Handle losing
        if self.hp <= 0:
            print("player " + str(self.player) + " loses")
            print("Game over: " + ("player 1 " if self.player == 2 else "player 2 ") + "wins!")
            self.destroy_node()
            return False
        if time >= self.max_time:
            print("Game over: time exceeded")
            self.destroy_node()
            return False


        # Handle Shielding
        if self.shield_flag == True:  # Cast Shield
            print("handle player - shield")
            self.shield(time)
        if self.shielding and time >= self.shield_end_time:  # Reset shield
            print("Shield Finished")
            self.shielding = False
            self.familiar_status = 1
            if self.color:
                setLEDColor(self.groupState.crazyflies[0], 0, 0, 0)
        if self.staggered and time >= self.stagger_end_time:  # Reset stagger
            print("Stagger Finished")
            self.stagger_end_time = 0
            self.staggered = False

        # Handle Quick Attacks
        # TODO maybe update this to allow for multiple quick attacks in series while the previous one is cooling down
        if self.quick_attack_flag == True:  # Cast Quick Attack
            # start quick attack  movement and set timer for shield to sleep
            self.quick_attack(time)
        if (self.quick_attacking and time >= self.quick_damage_inflict_time):
            print("Quick attacked!")
            damage_message = Int32()
            damage_message.data = self.quick_attack_damage
            self.damage_pub.publish(damage_message)
            self.quick_damage_inflict_time = self.max_time + 10000
            if self.color:
                setLEDColor(self.groupState.crazyflies[self.quick_attack_drones[0] + 1], 0, 0, 0)
        if (self.quick_attacking and time >= self.quick_attack_end_time):  
            print("Quick attack drones returned!")
            # Reset quick attack 
            self.quick_attacking = False
            self.status[self.quick_attack_drones[0]] = 1
            self.quick_attack_drones = []

        # Handle Heavy Attacks
        if self.heavy_attack_flag == True:  # Cast Quick Attack
            # start quick attack  movement and set timer for shield to sleep
            self.heavy_attack(time)
        if (self.heavy_attacking and time >= self.heavy_damage_inflict_time):
            print("Heavy attack finished")
            damage_message = Int32()
            damage_message.data = self.heavy_attack_damage
            self.damage_pub.publish(damage_message)
            self.heavy_damage_inflict_time = self.max_time + 100000
            if self.color:
                setLEDColor(self.groupState.crazyflies[1], 0, 0, 0)
                setLEDColor(self.groupState.crazyflies[2], 0, 0, 0)
                setLEDColor(self.groupState.crazyflies[3], 0, 0, 0)
        if (self.heavy_attacking and time >= self.heavy_attack_end_time):  # Reset heavy attack
            print("Heavy attack drones returned!")
            self.heavy_attacking = False
            for drone in self.heavy_attack_drones:
                self.status[drone] = 1
            self.heavy_attack_drones = []

        # Return true if game isn't over
        return True
