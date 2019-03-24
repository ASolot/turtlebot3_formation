#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rospy
import sys

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
# Position.
from tf import TransformListener
# Goal.
from geometry_msgs.msg import PoseStamped
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf.transformations import euler_from_quaternion
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan

# Import the detectors.py code rather than copy-pasting.
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
  import detectors
except ImportError:
  raise ImportError('Unable to import detectors.py. Make sure this file is in "{}"'.format(directory))

# Import the config.py variables rather than copy-pasting 
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.insert(0, directory)
try:
  import config
except ImportError:
  raise ImportError('Unable to import config.py. Make sure this file is in "{}"'.format(directory))

# Import the controllers.py variables rather than copy-pasting 
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
  import controllers
except ImportError:
  raise ImportError('Unable to import controlers.py. Make sure this file is in "{}"'.format(directory))

# class GoalPose(object):
#   def __init__(self):
#     rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)
#     self._position = np.array([np.nan, np.nan], dtype=np.float32)

#   def callback(self, msg):
#     # The pose from RViz is with respect to the "map".
#     self._position[X] = msg.pose.position.x
#     self._position[Y] = msg.pose.position.y
#     print('Received new goal position:', self._position)

#   @property
#   def ready(self):
#     return not np.isnan(self._position[0])

#   @property
#   def position(self):
#     return self._position

class SimpleLaser(object):
  def __init__(self, robot_namespace):
    self._name_space = robot_namespace
    rospy.Subscriber(robot_namespace + '/scan', LaserScan, self.callback)

    # 10 degree res
    first = np.linspace(0, np.pi/3, 13)
    second = -first
    second = second[1:]

    self._angles = np.concatenate((first,second), axis=0)
    self._width = np.pi / 180. * 2.  # 2 degrees cone of view.
    self._measurements = [float('inf')] * len(self._angles)
    self._indices = None

  def callback(self, msg):
    # Helper for angles.
    def _within(x, a, b):
      pi2 = np.pi * 2.
      x %= pi2
      a %= pi2
      b %= pi2
      if a < b:
        return a <= x and x <= b
      return a <= x or x <= b;

    # Compute indices the first time.
    if self._indices is None:
      self._indices = [[] for _ in range(len(self._angles))]
      for i, d in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        for j, center_angle in enumerate(self._angles):
          if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
            self._indices[j].append(i)

    ranges = np.array(msg.ranges)
    for i, idx in enumerate(self._indices):
      # We do not take the minimum range of the cone but the 10-th percentile for robustness.
      # self._measurements[i] = np.percentile(ranges[idx], 10)
      self._measurements[i] = ranges[idx]

  @property
  def ready(self):
    return not np.isnan(self._measurements[0])

  @property
  def measurements(self):
    return self._measurements

# controller-based navigation, computed in local coordinate system
def controller_based(laser_measurements, goal_position, controller):
  u = 0
  w = 0
  
  


  # if the distance to the goal is low, we stop
  if np.sqrt(goal[X]**2 + goal[Y]**2) <= MIN_DISTANCE_TO_TARGET: 
    return u, w, None

  # if not, we use one of the controllers to compute the commands
  

  # TODO: implement state-space machine to switch between 
  # goal reach and obstacle avoidance 


  return u, w, None


# path-based navigation
def path_based(laser_measurements, goal_position, controller):

    # # Update plan every 0.5s.
    # time_since = current_time - previous_time
    # if current_path and time_since < 2.:
    #   rate_limiter.sleep()
    #   continue

    # Run RRT.
    # start_node, final_node = rrt.rrt(slam.pose, goal.position, slam.occupancy_grid)
    # current_path = get_path(final_node)
    # if not current_path:
    #   print('Unable to reach goal position:', goal.position)
  
  return 0, 0, None

current_time = 0
previous_detection_time = 0


def run(args):

  robot_namespace = args.robot
  robot_role      = args.mode

  rospy.init_node(robot_namespace + '_navigation')

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = rospy.Publisher(robot_namespace + '/cmd_vel', Twist, queue_size=5)
  path_publisher = rospy.Publisher(robot_namespace + '/path', Path, queue_size=1)

  laser = SimpleLaser(robot_namespace)

  # Get the type of detector
  if args.detector == "average":
    detector = AverageDetector(robot_namespace, robot_role)
  else if args.detector == "smart": 
    detector = SmartDetector(robot_namespace, robot_role)
  else: # by default we use the dumb one
    detector = DumbDetector(robot_namespace, robot_role)

  # Get the navigation method
  navigation_method = globals()[args.navigation]
  
  # Get to choose which controller to use
  if args.controller == "lqr":
    controller = lqr(0,0,0,0)

  # this might get out 
  else if args.controller == "feedbackLinearized":
    controller = feedbackLinearized()

  else if args.controller == "pid":
    controller = pid(P,I,D)
  else: 
    controller = pid(P,I,0)
  

  # slam = SLAM()
  # goal = GoalPose()
  frame_id = 0
  current_path = []
  previous_time = rospy.Time.now().to_sec()

  # Stop moving message.
  stop_msg = Twist()
  stop_msg.linear.x = 0.
  stop_msg.angular.z = 0.

  # Make sure the robot is stopped.
  i = 0
  while i < 10 and not rospy.is_shutdown():
    publisher.publish(stop_msg)
    rate_limiter.sleep()
    i += 1

  while not rospy.is_shutdown():
    
    # slam.update()
    current_time = rospy.Time.now().to_sec()

    # Make sure all measurements are ready.
     # Get map and current position through SLAM:
    # > roslaunch exercises slam.launch
    # if not goal.ready or not slam.ready:
    #   rate_limiter.sleep()
    #   continue

    if not laser.ready: 
      rate_limiter.sleep()
      continue

    # Update goal every 0.5s.
    time_since = current_time - previous_detection_time
    if time_since > 0.5:
      detector.find_goal(laser.measurements)
      previous_detection_time = current_time
      
    # Nothing from the detector yet? sleep!
    if not detector.ready:
      rate_limiter.sleep()
      continue

    # returns the coordinate and the type of object for dumb detector
    goal_position = detector.goal_pose()
    
    # goal_reached = np.linalg.norm(slam.pose[:2] - goal.position) < .2
    # if goal_reached:
    #   publisher.publish(stop_msg)
    #   rate_limiter.sleep()
    #   continue

    u, w, current_path = navigation_method(laser.measurements, goal_position, controller)
    vel_msg = Twist()
    vel_msg.linear.x = u
    vel_msg.angular.z = w
    publisher.publish(vel_msg)

    # check if we have some path to publish
    if current_path != None:

      # Publish path to RViz.
      path_msg = Path()
      path_msg.header.seq = frame_id
      path_msg.header.stamp = rospy.Time.now()
      path_msg.header.frame_id = 'map'
      for u in current_path:
        pose_msg = PoseStamped()
        pose_msg.header.seq = frame_id
        pose_msg.header.stamp = path_msg.header.stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = u[X]
        pose_msg.pose.position.y = u[Y]
        path_msg.poses.append(pose_msg)
      path_publisher.publish(path_msg)

    rate_limiter.sleep()
    frame_id += 1


if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Runs navigation for one robot')
  parser.add_argument('--mode', action='store', default='HUMAN_FOLLOWER', help='Which velocity field to plot.', choices=['TURTLE_FOLLOWER', 'HUMAN_FOLLOWER'])
  parser.add_argument('--robot', action='store', default='tb3_0', help='Whose robot navigation system to start')
  parser.add_argument('--detector', action='store', default='dumb', help='Choose the type of detector', choices=['dumb', 'average', 'smart'])
  parser.add_argument('--navigation', action='store', default='controller_based', help='Choose the navigation type', choices=['controller_based','path_based'])
  parser.add_argument('--controller', action='store', default='pid', help='Choose the robot controller', choices=['pi','pid','lqr', 'feedbackLinearized'])
  args, unknown = parser.parse_known_args()
  try:
    run(args)
  except rospy.ROSInterruptException:
    pass