#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from sklearn.cluster import DBSCAN

import argparse
import matplotlib.pylab as plt
import matplotlib.patches as patches
import numpy as np
import os
import sys
import re
import scipy.signal
import yaml
import rospy

from std_msgs.msg import String

from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import CircleObstacle

X = 0
Y = 1

# Import the config.py variables rather than copy-pasting 
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.insert(0, directory)
try:
  import config
except ImportError:
  raise ImportError('Unable to import config.py. Make sure this file is in "{}"'.format(directory))

def euclidian_norm(first, last=[0,0]):
  return np.sqrt((first[X] - last[X])**2 + (first[Y] - last[Y])**2)

# The DUMB_DETECTOR looks for the closest points that could be clustered 
# then looks at the clusters, and how they have moved, and  

# for human detection, we look for two such semi-circles
class DumbDetector(object):
  def __init__(self, robot_namespace, robot_role) : 
    self._namespace = robot_namespace
    self._role = robot_role
    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._db = DBSCAN(eps=0.075, min_samples=3)
    np.seterr(all='warn')

  def find_goal(self, coordinates):
    
    self._prev_pose = self._pose

    # sanity check -> if nan we cut to 0
    coordinates = np.nan_to_num(coordinates)
    # coordinates = [x for x in coordinates if x[0]!=0 or x[1]!=0]
    coordinates = coordinates[~np.all(coordinates==0, axis=1)]
    # print (coordinates)

    # cluster the points
    self._db.fit(coordinates)

    potential_followable = [] 

    unique_labels = set(self._db.labels_)
    for i in unique_labels:
      if i == -1:
        continue

      labels_mask = (self._db.labels_ == i)

      class_members = coordinates[labels_mask]
      
      first = class_members[0]
      last= class_members[-1]

      # Robot:  3 random points on a wall? 
      # Me:     No, thanks! 
      if len(class_members) < 3: 
        continue 
      
      # we discard all clusters with the max distance between points greater than
      # the diameter of a potential leg / turtlebot
      if euclidian_norm(first, last) > 2*config.LEG_RADIUS_MAX:
        continue

      centre = np.average(class_members, axis=0)
      distance = euclidian_norm(centre)

      # probably error or wall
      if distance < 0.01 or distance > config.MAX_DISTANCE_TO_TARGET:
        continue

      potential_followable.append([centre, distance])

    # we assume we won't find anything 
    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    if potential_followable == None: 
      return

    first_time = np.isnan(self._prev_pose[0])
    minimum = config.MAX_DISTANCE_TO_TARGET

    # return the closest point to the last, which is within displacement 
    for item in potential_followable:
      if minimum > item[1]:
        if euclidian_norm(item[0], self._prev_pose) < config.MAX_TARGET_DISPLACEMENT or first_time: 
          self._pose = item[0]
          minimum = item[1]

    # TODO: Sanity checks, differentiate between human and turtle 
  
  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def goal_pose(self):
    return self._pose

  @property
  def obstacles(self): 
    return None


# We used the Obstacle Detector package 
# https://github.com/tysik/obstacle_detector/ 
# depending on the robot's role, we either track humans or turtlebots

# TODO: Use it in conjunction with 

class AverageDetector(object):

  def __init__(self, robot_namespace, robot_role):
    self._namespace = robot_namespace
    self._role = robot_role
    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._speed = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_speed = np.array([np.nan, np.nan], dtype=np.float32)
    self._obstacles     = None
    self._target_coords = None
    self._target_speeds = None
    self._detectFollowable = True
    rospy.Subscriber(robot_namespace + '/tracked_obstacles', Obstacles, self.callback)

  # filtering the objects
  def callback(self, msg):
    obstacles = []
    target_coords = []
    target_speeds = []

    for idx, circle in enumerate(msg.circles):

      obstacles.append(np.array([msg.circles[idx].center.x, 
                                msg.circles[idx].center.y, 
                                msg.circles[idx].velocity.x, 
                                msg.circles[idx].velocity.y, 
                                msg.circles[idx].radius, 
                                msg.circles[idx].true_radius
                                ], dtype=np.float64))

      # filter out obstacles which are too big
      if msg.circles[idx].true_radius < 4*config.LEG_RADIUS_MAX:
        target_coords.append(np.array([msg.circles[idx].center.x, 
                                      msg.circles[idx].center.y
                                      ], dtype=np.float64))

        target_speeds.append(np.array([msg.circles[idx].velocity.x, 
                                      msg.circles[idx].velocity.y
                                      ], dtype=np.float64))
    
    self._obstacles = obstacles
    self._target_coords = target_coords
    self._target_speeds = target_speeds


  # based on the filtered obstacles, last tracked obstacle position, etc, find the goal 
  def find_goal(self, coordinates):
    print ("Finding goal")

    pose = []
    speed = []

    if self._target_coords == None:
      print ("NO TARGETS")
      return

    # first time we are trying to detect smth
    if self._detectFollowable:
      tolerance = config.MAX_INITIAL_VIEW_WIDTH

      min_distance = config.MAX_DISTANCE_TO_TARGET
      for idx, target in enumerate(self._target_coords): 
        
        # check if in our field of view -> forward and between some [-x, x] coords
        # the coords of the object finder are reversed 
        if target[Y] < tolerance and target[Y] > -1.0*tolerance and target[X] > 0:
          # compute distance to target: 
          # print (target)
          distance = euclidian_norm(target)

          # get the closest initial obstacle and consider it as 
          # the object that should be followed 
          if distance < min_distance:
            min_distance = distance
            pose = target
            speed = self._target_speeds[idx]
            self._detectFollowable = False

    else:
      tolerance = config.MAX_DISTANCE_TO_TARGET*2

      potential_positions = []
      indexes             = []


      for idx, target in enumerate(self._target_coords): 
        # check if in our field of view -> forward and between some [-x, x] coords
        if target[Y] < tolerance and target[Y] > -1.0*tolerance and target[X] > 0:
          # compute distance to target: 

          distance = np.nan_to_num(euclidian_norm(target, self._prev_pose))
          # if np.isnan(self._prev_pose[0])
          # print (target)
          # print (distance)

          # look at the objects within a certain radius of the previous pose
          # -> the objects won't teleport :)
          if distance < config.MAX_TARGET_DISPLACEMENT:
            potential_positions.append(target)
            indexes.append(idx)
      
      # now that we've narrowed down things we could search for 
      # our target 
      # print ("Potential Positions")
      # print (potential_positions)

      # simple
      if len(potential_positions) > 0:

        # we take the closest yet again 
        min_distance = config.MAX_DISTANCE_TO_TARGET
        for idx, target in enumerate(potential_positions):

          distance = euclidian_norm(target)
          if distance < min_distance:
            pose = target
            speed = self._target_speeds[indexes[idx]]
            min_distance = distance
          
        #TODO: Improve it by taking into consideration speeds, headings, etc
      
      else: 
        # we have lost the target
        print ("Target Lost")
        self._detectFollowable = True
        self._pose = np.array([0,0])

    if len(pose) > 0: 
      self._prev_pose = self._pose
      self._prev_speed = self._speed
      self._pose = pose
      self._speed = speed
      print ("Target found")
      print (pose)

  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def goal_pose(self):
    return self._pose

  @property
  def obstacles(self): 
    return self._obstacles


# The SMART_DETECTOR makes use of the Kinect Point-cloud to detect humans
# TODO: implement
# using angusleigh/leg_tracker https://github.com/angusleigh/leg_tracker/blob/melodic/scripts/joint_leg_tracker.py
# https://github.com/wg-perception/people 
class SmartDetector(object): 
  def __init__(self, robot_namespace): 
    raise Exception("The SmartDetector based on Kinect Point-cloud was not implemented")
    self.namespace = robot_namespace
    self._pose = []
  
  def find_object(self):
    return None


  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def pose(self):
    return None

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Detects humans, turtlebots, and obstacles')
  parser.add_argument('--visible', action='store', default='no', options=['human', 'turtlebots', 'all', 'all_plus_moving_obstacles'], help='Which detections to show.')
  args, unknown = parser.parse_known_args()
  