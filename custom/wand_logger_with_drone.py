import numpy as np
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
import rclpy
from crazyflie_py import Crazyswarm
from std_msgs.msg import String

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


def resetDrones(side, allcfs):
    for droneId in dronePositions[side]:
        allcfs.crazyfliesById[droneId].goTo(
            np.asarray(dronePositions[side][droneId]), 0, 5.0
        )
        timeHelper.sleep(3)


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
        self.call_timer = self.create_timer(1 / self.Hz, self.timer_cb)
        self.maxQueueSize = 50
        self.positionQueue = []
        self.rotationQueue = []
        self.actionDetector = ActionDetector(shouldFlip=(self.player != 1))

        self.pub = self.create_publisher(String, "spell" + str(self.player), 10)
        print("spell" + str(self.player))

    def timer_cb(self):
        # Get state of wand
        wand_position, wand_rotation = self.wand_pose

        # here detect whether the logs are good
        self.positionQueue.append(wand_position)
        self.rotationQueue.append(euler_from_quaternion(*wand_rotation))
        # print(wand_position)

        firstNPositions = np.asarray(self.positionQueue[-self.maxQueueSize :])
        firstNRotations = np.asarray(self.rotationQueue[-self.maxQueueSize :])

        action = self.actionDetector.getAction(firstNPositions, firstNRotations)
        if action != None:

            # get action
            print("pretend dooing action ", action)

            msg = String()
            msg.data = action
            self.pub.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

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

            childFrame = "wand"
            childFrame = "tf1aero" if self.player == 1 else "tf2aero"

            if transform.child_frame_id == childFrame:
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

    timeHelper = None
    allcfs = None
    haveDrones = False
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
        # allcfs.crazyfliesById[18].startTrajectory(3, 5.0, False)
        # timeHelper.sleep(5)

        # print(allcfs.crazyfliesById.keys())

        # move all drones to where they should be
        for side in dronePositions:
            resetDrones(side, allcfs)
    else:
        try:
            rclpy.init()

        except:
            print("nooooool")

    wand_node = WandFollower(allcfs, timeHelper, 1)

    # executor = rclpy.Executor()
    # executor = MultiThreadedExecutor(num_threads=2)
    # executor.add_node(wand_node)
    # executor.add_node(listener) # add second wand node

    # executor.spin()

    rclpy.spin(wand_node)
