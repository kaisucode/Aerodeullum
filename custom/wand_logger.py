import numpy as np
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
import rclpy
#from crazyflie_py import Crazyswarm
#from sensor_msgs.msg import Joy

import rospy
from std_msgs.msg import String

from action_detector import *
import queue

class WandFollower(Node):

    def __init__(self, crazyflie, timeHelper, max_speed=0.5, update_frequency=20):
        super().__init__("wand_follower_node")
        self.max_speed = max_speed
        self.Hz = update_frequency
        <self class="crazyfli"></self>e = crazyflie
        # self.timeHelper = timeHelper
        self.wand_pose = ([0, 0, 0.25], [0, 0, 0, 1])
        self.controller = PDController(10, 0)

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

if __name__ == "__main__":

    timeHelper = None
    cf = None
    rclpy.init()

    wand_node = WandFollower(cf, timeHelper)

    rclpy.spin(wand_node)
