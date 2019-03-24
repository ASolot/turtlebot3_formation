#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

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
# TODO: implement
class MovingObject: 
  def __init__(self): 
    pass


# The DUMB_DETECTOR looks for the closest points that could be clustered into 
# a semi-circle with a radius similar with the robot's radius 

# for human detection, we look for two such semi-circles
class DumbDetector(object):
  def __init__(self, robot_namespace, robot_role) : 
    self._namespace = robot_namespace
    self._role = robot_role
    self._pose = np.array([np.nan, np.nan], dtype=np.float32)

  def find_goal(self, laser_measurements): 

    # find all circles

    # look for those who suit the description

    if self._role == HUMAN_FOLLOWER:

    else: 
  
  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def goal_pose(self):
    return self._pose


# TODO: implement -> uses inference and some smart-ish stuff, looks all around 
# and shares obstacles with others
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