import numpy as np
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
import rclpy
from crazyflie_py import Crazyswarm
#from sensor_msgs.msg import Joy
from pathlib import Path
from rclpy.executors import MultiThreadedExecutor

import rospy
from std_msgs.msg import String

from action_detector import *
import queue

haveDrones = True

dronePositions = {
        "sideA": {
            "id_1": [-3, -2, 1],
            "id_2": [-3, -1.5, 1.5],
            "id_3": [-3, -1, 1],
            "id_4": [-4, 2, 1]
            },
        "sideB": {
            "id_5": [3, 2.5, 1],
            "id_6": [3, 2, 1.5],
            "id_7": [3, 1.5, 1],
            "id_8": [4, -1.5, 1]
            }
        }

trajectoryFilemapping = {
        "triple_shield_center": 0,
        "triple_shield_left": 1,
        "triple_shield_right": 2,
        "spiral": 3,
        }



class WandFollower(Node):

    def __init__(self, allcfs, timeHelper, curSide="side_a", max_speed=0.5, update_frequency=20):
        super().__init__("wand_follower_node")
        self.max_speed = max_speed
        self.Hz = update_frequency
        self.wand_pose = ([0, 0, 0.25], [0, 0, 0, 1])
        #self.controller = PDController(10, 0)
        self.allcfs = allcfs
        self.timeHelper = timeHelper

        self.curSide = curSide
        self.position_subscriber = self.create_subscription(
            TFMessage, "tf", self.pose_callback, 1
        )
        self.call_timer = self.create_timer(1 / self.Hz, self.timer_cb)
        self.maxQueueSize = 50
        self.positionQueue = []
        self.rotationQueue = []
        self.actionDetector = ActionDetector()

        self.pub = rospy.Publisher("wandMovement", String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

    def timer_cb(self):
        # Get state of wand
        wand_position, wand_rotation = self.wand_pose

        # here detect whether the logs are good
        self.positionQueue.append(wand_position)
        self.rotationQueue.append(wand_rotation)
        # print(wand_position)

        firstNPositions = np.asarray(self.positionQueue[-self.maxQueueSize :])
        firstNRotations = np.asarray(self.rotationQueue[-self.maxQueueSize :])

        action = actionDetector.getAction(firstNPositions, firstNRotations)
        if action != None:

            # get action

            if action == "detectRotateSide": 
                # shield
                resetDrones(self.curSide, self.allcfs)

                if self.curSide == "side_a": 
                    self.allcfs.crazyfliesById["id_1"].startTrajectory(trajectoryFilemapping["triple_shield_left"])
                    self.allcfs.crazyfliesById["id_2"].startTrajectory(trajectoryFilemapping["triple_shield_center"])
                    self.allcfs.crazyfliesById["id_3"].startTrajectory(trajectoryFilemapping["triple_shield_right"])
                self.timeHelper.sleep(3) # TODO change duration
                resetDrones(self.curSide, self.allcfs)

            elif action == "detectFastAttack": 
                resetDrones(self.curSide, self.allcfs)
                if self.curSide == "side_a": 
                    self.allcfs.crazyfliesById["id_1"].startTrajectory(trajectoryFilemapping["spiral"])
                    #self.allcfs.crazyfliesById["id_2"].startTrajectory(trajectoryFilemapping["spiral"])
                    #self.allcfs.crazyfliesById["id_3"].startTrajectory(trajectoryFilemapping["spiral"])
                self.timeHelper.sleep(3) # TODO change duration
                resetDrones(self.curSide, self.allcfs)

            print(action)
            rospy.loginfo(action)
            self.pub.publish(action)
            rate.sleep()
        return

    def pose_callback(self, msg):
        """
        Pose callback method, called everytime a message is published to the topic /tf
        updates self.wand_pose to the latest pose of the object named "wand"
        """

        # Loop through all transforms (for all objects/crazyflies)
        for transform in msg.transforms:
            # Find the transform named "wand"
            if transform.child_frame_id == "wand":
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


def resetDrones(side, allcfs): 
    for droneId in dronePositions[side]: 
        allcfs.crazyfliesById[droneId].goTo(np.asarray(dronePositions[side][droneId]), 0, 5.0)
        timeHelper.sleep(3)
def loadTrajectories(): 
    allTrajectories = {} # key: numeric id, value: trajectory 

    trajId = 0
    for fileprefix in trajectoryFilemapping: 
        filename = "aero/" + fileprefix + ".csv"
        allTrajectories[trajId] = Trajectory()
        allTrajectories[trajId].loadcsv(Path(__file__).parent / filename)
        trajId += 1

    return allTrajectories


if __name__ == "__main__":

    timeHelper = None
    allcfs = None
    if haveDrones:
        swarm = Crazyswarm()
        timeHelper = swarm.timeHelper
        allcfs = swarm.allcfs

        allTrajectories = loadTrajectories()

        for cf in allcfs.crazyflies: 
            for trajectoryId in allTrajectories: 
                cf.uploadTrajectory(trajectoryId, 0, allTrajectories[trajectoryId])

        # takeoff all drones
        allcfs.takeoff(targetHeight=1.0, duration=3.0)
        timeHelper.sleep(3)

        # move all drones to where they should be
        for side in dronePositions: 
            resetDrones(side, allcfs)

    rclpy.init()

    wand_node = WandFollower(allcfs, timeHelper, "side_a")

    executor = rclpy.Executor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(wand_node)
    #executor.add_node(listener) # add second wand node

    executor.spin()

    #rclpy.spin(wand_node)
