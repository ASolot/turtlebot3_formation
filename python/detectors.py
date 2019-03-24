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

# Import the config.py variables rather than copy-pasting 
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.insert(0, directory)
try:
  import config
except ImportError:
  raise ImportError('Unable to import config.py. Make sure this file is in "{}"'.format(directory))

# Each moving object gets to be shared between robots :) 
# TODO: implement this awesome functionality
class MovingObject: 
  def __init__(self): 
    pass

def euclidian_norm(first, last=[(0,0)]):
  return np.sqrt((first[X] - last[X])**2 + (first[Y] - last[Y])**2)

# The DUMB_DETECTOR looks for the closest points that could be clustered 
# then looks at the clusters, and how they have moved, and  

# for human detection, we look for two such semi-circles
class DumbDetector(object):
  def __init__(self, robot_namespace, robot_role) : 
    self._namespace = robot_namespace
    self._role = robot_role
    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_pose = [(float('inf'), float('inf'))]
    self._pose = [(float('inf'), float('inf'))]
    self._db = DBSCAN(eps=0.075, min_samples=3)

  def find_goal(self, coordinates):
    self._prev_pose = self._pose

    # cluster the points
    self._db.fit(coordinates)

    potential_followable = [] 

    unique_labels = set(self._db.labels_)
    for i in unique_labels:
      if i == -1:
        continue

      labels_mask = (labels == i)

      class_members = coordinates[labels_mask]
      
      first = class_members[0]
      last= class_members[-1]

      # Robot:  3 random points on a wall? 
      # Me:     No, thanks! 
      if len(class_members) < 4: 
        continue 
      
      # we discard all clusters with the max distance between points greater than
      # the diameter of a potential leg / turtlebot
      if euclidian_norm(first, last) > 2*LEG_RADIUS_MAX:
        continue

      centre = np.average(class_members, axis=0)
      distance = euclidian_norm(centre)

      # probably error or wall
      if distance < 0.01 or distance > MAX_DISTANCE_TO_TARGET:
        continue

      potential_followable.append([centre, distance])

    # we assume we won't find anything 
    self._pose = [(float('inf'), float('inf'))]
    if potential_followable == None: 
      return

    first_time = np.isnan(self._prev_pose[0])

    for item in potential_followable: 
      if euclidian_norm(item[0], self._prev_pose) < MAX_TARGET_DISPLACEMENT or first_time: 
        self._pose = item[0]

    # TODO: Sanity checks, differentiate between human and turtle 
  
  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def goal_pose(self):
    return self._pose


# TODO: implement -> uses inference and some smart-ish stuff, looks all around 
# and shares obstacles with others

# using angusleigh/leg_tracker https://github.com/angusleigh/leg_tracker/blob/melodic/scripts/joint_leg_tracker.py
# using https://github.com/wg-perception/people

class AverageDetector(object):

  def __init__(self, robot_namespace):
    self.namespace = robot_namespace
    self._pose = []

  def find_object(self):
    # Get the new position of the object  
  
    # If there is no position available 
    return None

  @property
  def ready(self):
    return not np.isnan(self._pose[0])


# The SMART_DETECTOR makes use of the Kinect Point-cloud to detect humans
# TODO: implement
class SmartDetector(object): 
  def __init__(self, robot_namespace): 
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
  

  #TODO: implement others 