import numpy as np
from blocklyTranslations import *

class WandFollower(Node):
  def __init__(self, groupState, timeHelper, max_speed=0.5, update_frequency=20, sim=True):
    super().__init__('wand_follower_node')
    crazyflie = groupState.crazyflies[0]
    self.timeHelper = groupState.timeHelper
    self.max_speed = max_speed
    self.Hz = update_frequency
    self.crazyflie = crazyflie
    self.timeHelper = timeHelper
    self.wand_pose = ([0, 0, 1], [0, 0, 0, 1])

    # create subscriptions
    self.position_subscriber = self.create_subscription(TFMessage, 'tf', self.pose_callback, 1)
    self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 1)
    self.call_timer = self.create_timer(1/self.Hz, self.timer_cb)

    # We'll use known-to-work parameters for safety, but you can alter for sim
    self.controller = PDController(1, 0.05)

    # Lists for plotting
    self.states = []
    self.goals = []

  def timer_cb(self):
    """
    Executes every time a timer is triggered (rate based on Hz)
    """
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

  def pose_callback(self, msg):
    """
    Pose callback method, called everytime a message is published to the topic /tf
    updates self.wand_pose to the latest pose of the object named "wand"
    """

    # Loop through all transforms (for all objects/crazyflies)
    for transform in msg.transforms:
      # Find the transform named "wand"
      if transform.child_frame_id == 'wand':
        # position (x, y, z)
        position = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    ])
        # Rotation (roll, pitch, yaw, 1)
        rotation = np.array([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                    ])

        self.wand_pose = (position, rotation)
        # job done, end method early by calling return
        return

  def joy_callback(self, msg):
    """
    Shutdown when button on game pad is pressed
    """
    if msg.buttons[5] == 1: # Button was pressed
      self.cf.notifySetpointsStop()
      self.cf.land(0., 3)

      rclpy.shutdown() # destory node




def main():
    sim = True
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
    takeoff(groupState, 1.0, 3)
    timeHelper.sleep(3.0)

    rclpy.spin(wand_node)
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
