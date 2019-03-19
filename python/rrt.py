from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import matplotlib.pylab as plt
import matplotlib.patches as patches
import numpy as np
import os
import re
import scipy.signal
import yaml


# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

ROBOT_RADIUS = 0.105 / 2.
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)  # Any orientation is good.
START_POSE = np.array([-1.5, -1.5, 0.], dtype=np.float32)
MAX_ITERATIONS = 2000



def sample_random_position(occupancy_grid):
  position = np.zeros(2, dtype=np.float32)

  # MISSING: Sample a valid random position (do not sample the yaw).
  # The corresponding cell must be free in the occupancy grid.

  while True:

    # a position is a pixel in the occupancy_grid -> [0, shape[X] or shape[Y]]
    # based on get_index we get "position" for the "idx" 0 and shape[X] or shape[Y]

    # we assume that image is not always square :) 
    position[X] = np.random.uniform(occupancy_grid.origin[X], occupancy_grid._values.shape[X] * occupancy_grid.resolution + occupancy_grid.origin[X])
    position[Y] = np.random.uniform(occupancy_grid.origin[Y], occupancy_grid._values.shape[Y] * occupancy_grid.resolution + occupancy_grid.origin[Y])

    if occupancy_grid.is_free(position):
      break

  return position


def adjust_pose(node, final_position, occupancy_grid):
  final_pose = node.pose.copy()
  final_pose[:2] = final_position
  

  # # # MISSING: Check whether there exists a simple path that links node.pose
  # # # to final_position. This function needs to return a new node that has
  # # # the same position as final_position and a valid yaw. The yaw is such that
  # # # there exists an arc of a circle that passes through node.pose and the
  # # # adjusted final pose. If no such arc exists (e.g., collision) return None.
  # # # Assume that the robot always goes forward.
  # # # Feel free to use the find_circle() function below.

  # the starting point P1
  x1 = node.pose[X]
  y1 = node.pose[Y]

  # the destination point P2
  x2 = final_position[X]
  y2 = final_position[Y]

  # C - circle center of coordinates xc and yc
  # we init the coordinates with 0 
  xc = 0
  yc = 0 

  # m1      - slope of the tangent in point P1 
  # m2      - slope of the tangent in point P2 
  # m       - slope of the line drawn through P1 and C
  # m_prime - slope of the line drawn through P2 and C
  # theta1  - angle between the horizontal through C and P1C
  # theta2  - angle between the horizontal through C and P2C 

  # the equation of the tangent through P1 is y = m1 x + n1 
  # the equation of P1C is y = m x + n  => y - mx = y1 - mx1 => y - y1 = m (x - x1)

  m1 = np.tan(node.pose[YAW])

  # account for null 
  if m1 == 0: 
    xc = x1 
    yc = (1.0/2.0 * (x1**2 + y1**2 - x2**2 - y2**2) - (x1 - x2) * xc ) / (y1 - y2)

  else: 
    # the two lines are perpendicular => m * m1 = -1
    m = -1.0 / m1

    xc = ((y2-y1)*(y1 - m*x1) + 1.0/2.0 * (x1**2 + y1**2 - x2**2 - y2**2)) / (x1 - x2 + m*(y1 - y2))
    yc = m * (xc - x1) + y1

  center = np.array([xc, yc])

  # radius is distance between P1 and C
  radius = np.sqrt((yc - x1)**2 + (xc - x1)**2 )

  # m_prime = (yc - y2) / (xc - x2)

  # m2 = -1.0 / m_prime

  # we use the center of the circle as new origin, and we translate coordinates 
  du = node.position - center
  dv = final_position - center

  # we compute the angles between
  # P1C and horizontal
  theta1 = np.arctan2(du[1], du[0])
  # P2C and horizontal
  theta2 = np.arctan2(dv[1], dv[0])

  # we get to know if we are moving clockwise or anti-clockwise
  clockwise = np.cross(node.direction, du).item() > 0.

  # we compute angular sweeping resolution, starting from linear resolution
  # using lenght_arc = angle * radius 
  # for small numbers we can aproximate the arc as a line
  sweep_angle = occupancy_grid.resolution / 6.0 / radius

  if clockwise:
    # see report for figure 
    yaw2 = theta2 - np.pi/2.0
    #clockwise we decrement
    sweep_angle = -sweep_angle

    # we assure we stay in [-pi, pi]
    if yaw2 < -np.pi: 
      yaw2 += 2*np.pi
  else:
    # see report for figure
    yaw2 = theta2 + np.pi/2.0

    # we assure we stay in [-pi, pi]
    if yaw2 > np.pi:
      yaw2 -= 2*np.pi

  final_pose[YAW] = yaw2

  # generate the angles corresponding to the points on the arc we are evaluating
  if clockwise and theta2 > theta1: 
    # this means that either theta2 is positive or negative, and we had to build an arc with a span > 180* 
    angles = np.concatenate((np.arange(theta1, -np.pi, sweep_angle), np.arange(np.pi, theta2, sweep_angle)))
  
  else:
    if (not clockwise) and theta1 > theta2: 
      angles = np.concatenate((np.arange(theta1, np.pi, sweep_angle), np.arange(-np.pi, theta2, sweep_angle)))

    else: 
      angles = np.arange(theta1, theta2, sweep_angle)

  # using the angles, generate the arc, and check if any point on it is on a free position in the grid
  for angle in angles:
    point = np.array([center[X] + np.cos(angle) * radius, center[Y] + np.sin(angle) * radius])
    
    # tough luck, getting out of the free zone 
    # we do not accept unknowns
    if not occupancy_grid.is_free(point):
      return None

    # works ok when building a map
    # if not occupancy_grid.is_occupied(point):
    #   return None

  # A BETTER APPROACH IS TO RASTERIZE THE ARCS'S LINE AND COMPUTE THE INTERSECTION BETWEEN THE RASTER IMAGE 
  # AND THE OCCUPANCY GRID. HOWEVER, THIS IS COMPUTATIONALLY INTENSIVE

  final_node = Node(final_pose)
  return final_node


# Defines an occupancy grid.
class OccupancyGrid(object):
  def __init__(self, values, origin, resolution):
    self._original_values = values.copy()
    self._values = values.copy()
    # Inflate obstacles (using a convolution).
    inflated_grid = np.zeros_like(values)
    inflated_grid[values == OCCUPIED] = 1.
    w = 2 * int(ROBOT_RADIUS / resolution) + 1
    inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
    self._values[inflated_grid > 0.] = OCCUPIED
    self._origin = np.array(origin[:2], dtype=np.float32)
    self._origin -= resolution / 2.
    assert origin[YAW] == 0.
    self._resolution = resolution

  @property
  def values(self):
    return self._values

  @property
  def resolution(self):
    return self._resolution

  @property
  def origin(self):
    return self._origin

  def draw(self):
    plt.imshow(self._original_values.T, interpolation='none', origin='lower',
               extent=[self._origin[X],
                       self._origin[X] + self._values.shape[0] * self._resolution,
                       self._origin[Y],
                       self._origin[Y] + self._values.shape[1] * self._resolution])
    plt.set_cmap('gray_r')

  def get_index(self, position):
    idx = ((position - self._origin) / self._resolution).astype(np.int32)
    if len(idx.shape) == 2:
      idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
      idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
      return (idx[:, 0], idx[:, 1])
    idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
    idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
    return tuple(idx)

  def get_position(self, i, j):
    return np.array([i, j], dtype=np.float32) * self._resolution + self._origin

  def is_occupied(self, position):
    return self._values[self.get_index(position)] == OCCUPIED

  def is_free(self, position):
    return self._values[self.get_index(position)] == FREE


# Defines a node of the graph.
class Node(object):
  def __init__(self, pose):
    self._pose = pose.copy()
    self._neighbors = []
    self._parent = None
    self._cost = 0.

  @property
  def pose(self):
    return self._pose

  def add_neighbor(self, node):
    self._neighbors.append(node)

  @property
  def parent(self):
    return self._parent

  @parent.setter
  def parent(self, node):
    self._parent = node

  @property
  def neighbors(self):
    return self._neighbors

  @property
  def position(self):
    return self._pose[:2]

  @property
  def yaw(self):
    return self._pose[YAW]
  
  @property
  def direction(self):
    return np.array([np.cos(self._pose[YAW]), np.sin(self._pose[YAW])], dtype=np.float32)

  @property
  def cost(self):
      return self._cost

  @cost.setter
  def cost(self, c):
    self._cost = c


def rrt(start_pose, goal_position, occupancy_grid):
  # RRT builds a graph one node at a time.
  graph = []
  start_node = Node(start_pose)
  final_node = None
  if not occupancy_grid.is_free(goal_position):
    print('Goal position is not in the free space.')
    return start_node, final_node
  graph.append(start_node)
  for _ in range(MAX_ITERATIONS): 
    position = sample_random_position(occupancy_grid)
    # With a random chance, draw the goal position.
    if np.random.rand() < .05:
      position = goal_position
    # Find closest node in graph.
    # In practice, one uses an efficient spatial structure (e.g., quadtree).
    potential_parent = sorted(((n, np.linalg.norm(position - n.position)) for n in graph), key=lambda x: x[1])
    # Pick a node at least some distance away but not too far.
    # We also verify that the angles are aligned (within pi / 4).
    u = None
    for n, d in potential_parent:
      if d > .2 and d < 1.5 and n.direction.dot(position - n.position) / d > 0.70710678118:
        u = n
        break
    else:
      continue
    v = adjust_pose(u, position, occupancy_grid)
    if v is None:
      continue
    u.add_neighbor(v)
    v.parent = u
    graph.append(v)
    if np.linalg.norm(v.position - goal_position) < .2:
      final_node = v
      break
  return start_node, final_node


def find_circle(node_a, node_b):
  def perpendicular(v):
    w = np.empty_like(v)
    w[X] = -v[Y]
    w[Y] = v[X]
    return w
  db = perpendicular(node_b.direction)
  dp = node_a.position - node_b.position
  t = np.dot(node_a.direction, db)
  if np.abs(t) < 1e-3:
    # By construction node_a and node_b should be far enough apart,
    # so they must be on opposite end of the circle.
    center = (node_b.position + node_a.position) / 2.
    radius = np.linalg.norm(center - node_b.position)
  else:
    radius = np.dot(node_a.direction, dp) / t
    center = radius * db + node_b.position
  return center, np.abs(radius)


def read_pgm(filename, byteorder='>'):
  """Read PGM file."""
  with open(filename, 'rb') as fp:
    buf = fp.read()
  try:
    header, width, height, maxval = re.search(
        b'(^P5\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n]\s)*)', buf).groups()
  except AttributeError:
    raise ValueError('Invalid PGM file: "{}"'.format(filename))
  maxval = int(maxval)
  height = int(height)
  width = int(width)
  img = np.frombuffer(buf,
                      dtype='u1' if maxval < 256 else byteorder + 'u2',
                      count=width * height,
                      offset=len(header)).reshape((height, width))
  return img.astype(np.float32) / 255.


def draw_solution(start_node, final_node=None):
  ax = plt.gca()

  def draw_path(u, v, arrow_length=.1, color=(.8, .8, .8), lw=1):
    du = u.direction
    plt.arrow(u.pose[X], u.pose[Y], du[0] * arrow_length, du[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    dv = v.direction
    plt.arrow(v.pose[X], v.pose[Y], dv[0] * arrow_length, dv[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    center, radius = find_circle(u, v)
    du = u.position - center
    theta1 = np.arctan2(du[1], du[0])
    dv = v.position - center
    theta2 = np.arctan2(dv[1], dv[0])
    # Check if the arc goes clockwise.
    if np.cross(u.direction, du).item() > 0.:
      theta1, theta2 = theta2, theta1
    ax.add_patch(patches.Arc(center, radius * 2., radius * 2.,
                             theta1=theta1 / np.pi * 180., theta2=theta2 / np.pi * 180.,
                             color=color, lw=lw))

  points = []
  s = [(start_node, None)]  # (node, parent).
  while s:
    v, u = s.pop()
    if hasattr(v, 'visited'):
      continue
    v.visited = True
    # Draw path from u to v.
    if u is not None:
      draw_path(u, v)
    points.append(v.pose[:2])
    for w in v.neighbors:
      s.append((w, v))

  points = np.array(points)
  plt.scatter(points[:, 0], points[:, 1], s=10, marker='o', color=(.8, .8, .8))
  if final_node is not None:
    plt.scatter(final_node.position[0], final_node.position[1], s=10, marker='o', color='k')
    # Draw final path.
    v = final_node
    while v.parent is not None:
      draw_path(v.parent, v, color='k', lw=2)
      v = v.parent


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Uses RRT to reach the goal.')
  parser.add_argument('--map', action='store', default='map', help='Which map to use.')
  args, unknown = parser.parse_known_args()

  # Load map.
  with open(args.map + '.yaml') as fp:
    data = yaml.load(fp)
  img = read_pgm(os.path.join(os.path.dirname(args.map), data['image']))
  occupancy_grid = np.empty_like(img, dtype=np.int8)
  occupancy_grid[:] = UNKNOWN
  occupancy_grid[img < .1] = OCCUPIED
  occupancy_grid[img > .9] = FREE
  # Transpose (undo ROS processing).
  occupancy_grid = occupancy_grid.T
  # Invert Y-axis.
  occupancy_grid = occupancy_grid[:, ::-1]
  occupancy_grid = OccupancyGrid(occupancy_grid, data['origin'], data['resolution'])

  # Run RRT.
  start_node, final_node = rrt(START_POSE, GOAL_POSITION, occupancy_grid)

  # Plot environment.
  fig, ax = plt.subplots()
  occupancy_grid.draw()
  plt.scatter(.3, .2, s=10, marker='o', color='green', zorder=1000)
  draw_solution(start_node, final_node)
  plt.scatter(START_POSE[0], START_POSE[1], s=10, marker='o', color='green', zorder=1000)
  plt.scatter(GOAL_POSITION[0], GOAL_POSITION[1], s=10, marker='o', color='red', zorder=1000)
  
  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-.5 - 2., 2. + .5])
  plt.ylim([-.5 - 2., 2. + .5])
  plt.show()
  