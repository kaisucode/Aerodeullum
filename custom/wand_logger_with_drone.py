import numpy as np
import rclpy.callback_groups
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
import rclpy
from crazyflie_py import Crazyswarm
from std_msgs.msg import String
import time
from crazyflie_py.uav_trajectory import Trajectory


import numpy as np
from pathlib import Path

from crazyflie_py import *

# from sensor_msgs.msg import Joy
# from pathlib import Path
# from crazyflie_py.uav_trajectory import Trajectory
# from rclpy.executors import MultiThreadedExecutor


# import rospy
from std_msgs.msg import String

from action_detector import *
import queue
import math

haveDrones = True


dronePositions = [
    [[-4, 2, 1], [-3, -2, 1], [-3, -1.5, 1.5], [-3, -1, 1]],
    [[4, -1.5, 1], [3, 2.5, 1], [3, 2, 1.5], [3, 1.5, 1]],
]

#  trajectoryNames = ["spiral", "helix1", "helix2", "helix3", "familiar", "single_shield", "simple"]
#  trajectoryNames = ["spiral", "helix4", "familiar", "single_shield", "simple"]
trajectoryNames = ["spiral", "helix4", "familiar", "single_shield"]


def loadTrajectories():
    trajectoryFilemapping = {}  # {"name": {"trajectory", "id"}}
    trajId = 0
    for fileprefix in trajectoryNames:

        #  for player in ["p1_", "p2_"]:
        for player in ["p1_"]:
            trajName = player + fileprefix
            trajectoryFilemapping[trajName] = {"id": trajId, "trajectory": Trajectory()}
            filename = "aero/" + trajName + ".csv"
            trajectoryFilemapping[trajName]["trajectory"].loadcsv(
                Path(__file__).parent / filename
            )
            trajId += 1
            time.sleep(0.5)
    return trajectoryFilemapping


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    # return roll_x, pitch_y, yaw_z # in radians
    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z)


class WandFollower(Node):

    def __init__(
        self,
        allcfs,
        timeHelper,
        curSide="sideA",
        max_speed=0.5,
        update_frequency=10,
        player=1,
        oneDrone=False,
    ):
        super().__init__("wand_follower_node" + str(player))
        self.player = player
        self.max_speed = max_speed
        self.Hz = update_frequency
        self.wand_pose = ([0, 0, 0.25], [0, 0, 0, 1])
        # self.controller = PDController(10, 0)
        self.allcfs = allcfs
        self.timeHelper = timeHelper

        self.curSide = curSide
        self.position_subscriber = self.create_subscription(
            TFMessage, "tf", self.pose_callback, 1
        )
        self.call_timer = self.create_timer(
            1 / self.Hz,
            self.timer_cb,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
        )
        self.call_timer2 = self.create_timer(1 / self.Hz, self.timer_cb2)

        # self.call_timer2 = self.create_timer(1 / self.Hz, self.timer_cb)

        self.maxQueueSize = 50
        self.positionQueue = []
        self.rotationQueue = []
        self.actionDetector = ActionDetector(shouldFlip=(self.player != 1))
        self.curAttack = 1

        self.oneDrone = oneDrone
        self.queuedAction = "single_shield"

        #  self.pub = self.create_publisher(String, 'spell' + str(self.player), 10)

        self.gestureLock = False
        self.lastValidTimestamp = time.time()
        self.trajectoryFilemapping = loadTrajectories()
        #  for cf in self.allcfs.crazyflies:
        #      for fileprefix in self.trajectoryFilemapping:
        #          trajectoryId = self.trajectoryFilemapping[fileprefix]["id"]

        #          cf.uploadTrajectory(
        #              trajectoryId,
        #              0,
        #              self.trajectoryFilemapping[fileprefix]["trajectory"],
        #          )
        #          timeHelper.sleep(1)

    def getTrajectory(self, trajName):
        return (
            self.trajectoryFilemapping[trajName]["id"],
            self.trajectoryFilemapping[trajName]["trajectory"],
        )

    def initialize_drone_position(self, droneIndex, player=1):
        side = player - 1
        self.allcfs.crazyflies[droneIndex].goTo(
            np.asarray(dronePositions[side][droneIndex]), 0, 3.0
        )
        self.timeHelper.sleep(3)

    def executeTraj(self, droneId, trajName):
        trajId, traj = self.getTrajectory("p1_" + trajName)
        self.allcfs.crazyflies[droneId].uploadTrajectory(trajId, 0, traj)
        timeHelper.sleep(2)
        self.allcfs.crazyflies[droneId].startTrajectory(trajId, 1.0, False)
        self.timeHelper.sleep(traj.duration + 0.5)
        self.initialize_drone_position(droneId, 1)
        print("done executing trajectory ", trajName)

    def timer_cb2(self):
        action = self.queuedAction
        print("============================ called but no action maybe ", action)
        if action != None:

            # if self.gestureLock == False:
            #     self.gestureLock = True
            # else:
            #     return

            print("using action: ", action)

            if action == "detectRotateSide" or action == "detectChargedAttack":
                #  trajId, traj = self.getTrajectory("p1" + "single_shield")
                self.executeTraj(0, "single_shield")
            elif action == "detectFastAttack":
                attackDrone = self.curAttack % 3 + 1
                self.curAttack += 1
                if self.oneDrone:
                    attackDrone = 0
                self.executeTraj(attackDrone, "spiral")

            #  elif action == "detectChargedAttack":
            #      trajId, traj = self.getTrajectory("p1" + "helix4")
            #      if self.oneDrone:
            #          attackDrone = 0
            #          self.allcfs.crazyflies[attackDrone].startTrajectory(trajId, 1.0, False)
            #      else:
            #          for i in range(1, 4):
            #              self.allcfs.crazyflies[i].startTrajectory(trajId, 1.0, False)
            action = None
            # timeHelper.sleep(2)
            # self.gestureLock = False

    def timer_cb(self):
        #  if time.time() > self.max_time:
        #      self.destroy_node()

        # Get state of wand
        if self.gestureLock == True:
            print("gesture locked")
            return
        wand_position, wand_rotation = self.wand_pose

        # here detect whether the logs are good
        self.positionQueue.append(wand_position)
        self.rotationQueue.append(euler_from_quaternion(*wand_rotation))
        # print(wand_position)

        firstNPositions = np.asarray(self.positionQueue[-self.maxQueueSize :])
        firstNRotations = np.asarray(self.rotationQueue[-self.maxQueueSize :])

        action = self.actionDetector.getAction(firstNPositions, firstNRotations)
        self.queuedAction = action

        # get action
        #  msg = String()
        #  msg.data = action
        #  self.pub.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

        return
        print(action)
        # rospy.loginfo(action)
        # self.pub.publish(action)
        # self.rate.sleep()
        return

    def pose_callback(self, msg):
        """
        Pose callback method, called everytime a message is published to the topic /tf
        updates self.wand_pose to the latest pose of the object named "wand"
        """

        # Loop through all transforms (for all objects/crazyflies)
        for transform in msg.transforms:
            # Find the transform named "wand"
            childFrame = "tfaeropurple" if self.player == 1 else "tfaerobrown"
            childFrame = "tf1aero"
            childFrame = "wand"

            if transform.child_frame_id == childFrame:
                # if transform.child_frame_id == "wand":
                # position (x, y, z)
                position = np.array(
                    [
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ]
                )
                # Rotation (roll, pitch, yaw, 1)
                rotation = np.array(
                    [
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w,
                    ]
                )

                self.wand_pose = (position, rotation)
                # job done, end method early by calling return
                return


if __name__ == "__main__":

    #  allTrajectories = loadTrajectories()
    #  time.sleep(1)
    timeHelper = None
    allcfs = None
    haveDrones = True
    if haveDrones:
        swarm = Crazyswarm()
        timeHelper = swarm.timeHelper
        allcfs = swarm.allcfs

        wand_node = WandFollower(allcfs, timeHelper, 1, oneDrone=True)

        timeHelper.sleep(3)
        # takeoff all drones
        print("taking off")
        allcfs.takeoff(targetHeight=1.0, duration=3.0)
        timeHelper.sleep(2)

        if haveDrones:
            print("Going to start positions")
            wand_node.initialize_drone_position(0, 1)

            # for idx in range(4):
            #     wand_node.initialize_drone_position(idx, 1)

        rclpy.spin(wand_node)

    else:
        try:
            rclpy.init()

        except:
            print("Error in rclpy init")

    # wand_node = WandFollower(allcfs, timeHelper, "sideA")
    wand_node = WandFollower(allcfs, timeHelper, 1)

    # executor = rclpy.Executor()
    # executor = MultiThreadedExecutor(num_threads=2)
    # executor.add_node(wand_node)
    # executor.add_node(listener) # add second wand node

    # executor.spin()

    rclpy.spin(wand_node)
