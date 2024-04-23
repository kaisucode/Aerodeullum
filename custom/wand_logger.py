import numpy as np
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from crazyflie_py import Crazyswarm
import rclpy
from sensor_msgs.msg import Joy

import rospy
from std_msgs.msg import String

from action_detector import *
import queue

class PDController:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd

        self.previous_error = 0

    def pd_controller(self, error):
        # TODO: compute control output given error

        derivative = error - self.previous_error

        u = self.Kp * error + self.Kd * derivative

        self.previous_error = error

        return u


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
        # self.joy_subscriber = self.create_subscription(Joy, "joy", self.joy_callback, 1)
        self.maxQueueSize = 50
        self.positionQueue = []
        self.rotationQueue = []

        self.pub = rospy.Publisher("wandMovement", String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz


        # self.positionQueue = queue.Queue(maxsize=self.maxQueueSize)
        # self.rotationQueue = queue.Queue(maxsize=self.maxQueueSize)
        # self.index = 0

    def timer_cb(self):
        # Get state of wand
        wand_position, wand_rotation = self.wand_pose
        # print(repr(self.wand_pose[0]))
        # np.set_printoptions(precision=3, suppress=True)
        # print(np.array2string(self.wand_pose[0], separator=","))
        # print(np.array2string(self.wand_pose[1], separator=","))

        # here detect whether the logs are good
        # self.positionQueue.put(wand_position)
        # self.rotationQueue.put(wand_rotation)
        self.positionQueue.append(wand_position)
        self.rotationQueue.append(wand_rotation)
        # print(wand_position)

        # firstNPositions = [
        #     self.positionQueue.get() for _ in range(self.positionQueue.qsize())
        # ]
        # firstNRotations = [
        #     self.rotationQueue.get() for _ in range(self.rotationQueue.qsize())
        # ]
        firstNPositions = np.asarray(self.positionQueue[-self.maxQueueSize :])
        firstNRotations = np.asarray(self.rotationQueue[-self.maxQueueSize :])

        # self.index += 1

        # print("attempt")
        action = getAction(firstNPositions, firstNRotations)
        if action != None:
            print(action)
            rospy.loginfo(action)
            self.pub.publish(action)
            rate.sleep()
        return

        # TODO: calculate error
        # position of crazyflie can be accessed with self.crazyflie.position()
        error = wand_position - self.crazyflie.position()

        # Get Velocity (using PDController)
        desired_velocity = self.controller.pd_controller(error)

        # TODO: use send_vel_cmd to send velocity to crazyflie
        desired_velocity[0] = 0
        self.send_vel_cmd(desired_velocity)

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

    def send_vel_cmd(self, vel):
        """
        send velocity-style command to crazyflie
        Args:
          vel: (array-like of float[3]): Velocity meters/second
        """
        pos = self.crazyflie.position()
        self.crazyflie.cmdPosition(pos + vel * 1 / self.Hz)

    def joy_callback(self, msg):
        """
        Shutdown when button on game pad is pressed
        """
        if msg.buttons[5] == 1:  # Button was pressed
            self.cf.notifySetpointsStop()
            self.cf.land(0.0, 3)

            rclpy.shutdown()  # destory node


if __name__ == "__main__":
    # swarm = Crazyswarm()

    timeHelper = None
    # timeHelper = swarm.timeHelper

    #    allcfs = swarm.allcfs

    #    cf = allcfs.crazyflies[0]
    cf = None
    # cf.takeoff(0.25, 3.0)
    # cf.setLEDColor(0, 255, 255)
    # timeHelper.sleep(3.0)
    rclpy.init()

    wand_node = WandFollower(cf, timeHelper)  #

    rclpy.spin(wand_node)
