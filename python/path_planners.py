#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

# Import the config.py variables rather than copy-pasting 
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.insert(0, directory)
try:
  import config
except ImportError:
  raise ImportError('Unable to import config.py. Make sure this file is in "{}"'.format(directory))

# class SLAM(object):
#   def __init__(self):
#     rospy.Subscriber('/map', OccupancyGrid, self.callback)
#     self._tf = TransformListener()
#     self._occupancy_grid = None
#     self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    
#   def callback(self, msg):
#     values = np.array(msg.data, dtype=np.int8).reshape((msg.info.width, msg.info.height))
#     processed = np.empty_like(values)
#     processed[:] = rrt.FREE
#     processed[values < 0] = rrt.UNKNOWN
#     processed[values > 50] = rrt.OCCUPIED
#     processed = processed.T
#     origin = [msg.info.origin.position.x, msg.info.origin.position.y, 0.]
#     resolution = msg.info.resolution
#     self._occupancy_grid = rrt.OccupancyGrid(processed, origin, resolution)

#   def update(self):
#     # Get pose w.r.t. map.
#     a = 'occupancy_grid'
#     b = 'base_link'
#     if self._tf.frameExists(a) and self._tf.frameExists(b):
#       try:
#         t = rospy.Time(0)
#         position, orientation = self._tf.lookupTransform('/' + a, '/' + b, t)
#         self._pose[X] = position[X]
#         self._pose[Y] = position[Y]
#         _, _, self._pose[YAW] = euler_from_quaternion(orientation)
#       except Exception as e:
#         print(e)
#     else:
#       print('Unable to find:', self._tf.frameExists(a), self._tf.frameExists(b))
#     pass

#   @property
#   def ready(self):
#     return self._occupancy_grid is not None and not np.isnan(self._pose[0])

#   @property
#   def pose(self):
#     return self._pose

#   @property
#   def occupancy_grid(self):
#     return self._occupancy_grid

# we compute the shortest holonomic path that links us to the goal position, 
# regardless the obstacles


# TODO: insert the description of the code 

# Code adapted from Atsushi Sakai
# https://atsushisakai.github.io/PythonRobotics/ 

# The planner receives updated target coordinates
# The planner receives updated obstacle coordinates 


class DynamicWindowVO(object):

    def __init__(self):
        # robot parameter
        self.max_speed = config.SPEED
        self.min_speed = config.REVERSE_SPEED  # [m/s]
        self.max_yawrate = config.MAX_W  # [rad/s]
        self.max_accel = config.MAX_ACCEL
        self.max_dyawrate = config.MAX_W_ACCEL
        self.v_reso = 0.01  # [m/s]
        self.yawrate_reso = 0.1 * np.pi / 180.0  # [rad/s]
        self.dt = config.SIM_LOOP_TIME  # [s]
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.robot_radius = config.ROBOT_RADIUS
        self.show_animation = True

    def motion(x, u, dt):
        # motion model

        x[2] += u[1] * dt
        x[0] += u[0] * np.cos(x[2]) * dt
        x[1] += u[0] * np.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x


    def calc_dynamic_window(self, x):

        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
            -self.max_yawrate, self.max_yawrate]

        # Dynamic window from motion model
        Vd = [x[3] - self.max_accel * self.dt,
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_dyawrate * self.dt,
            x[4] + self.max_dyawrate * self.dt]

        #  [vmin,vmax, yawrate min, yawrate max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw


    def calc_trajectory(self, xinit, v, y):

        x = np.array(xinit)
        traj = np.array(x)
        time = 0
        while time <= self.predict_time:
            x = motion(x, [v, y], self.dt)
            traj = np.vstack((traj, x))
            time += self.dt

        return traj


    def calc_final_input(self, x, u, dw, goal, ob):

        xinit = x[:]
        min_cost = 10000.0
        min_u = u
        min_u[0] = 0.0
        best_traj = np.array([x])

        # evalucate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_reso):
            for y in np.arange(dw[2], dw[3], self.yawrate_reso):
                traj = calc_trajectory(xinit, v, y)

                # calc cost
                to_goal_cost = calc_to_goal_cost(traj, goal)
                speed_cost = self.speed_cost_gain * \
                    (self.max_speed - traj[-1, 3])
                ob_cost = calc_obstacle_cost(traj, ob)
                # print(ob_cost)

                final_cost = to_goal_cost + speed_cost + ob_cost

                #print (final_cost)

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    min_u = [v, y]
                    best_traj = traj

        return min_u, best_traj


    def calc_obstacle_cost(self, traj, ob):
        # calc obstacle cost inf: collistion, 0:free

        skip_n = 2
        minr = float("inf")

        for ii in range(0, len(traj[:, 1]), skip_n):
            for i in range(len(ob[:, 0])):
                ox = ob[i, 0]
                oy = ob[i, 1]
                dx = traj[ii, 0] - ox
                dy = traj[ii, 1] - oy

                r = np.sqrt(dx**2 + dy**2)
                if r <= self.robot_radius:
                    return float("Inf")  # collision

                if minr >= r:
                    minr = r

        return 1.0 / minr  # OK


    def calc_to_goal_cost(self, traj, goal):
        # calc to goal cost. It is 2D norm.

        goal_magnitude = np.sqrt(goal[0]**2 + goal[1]**2)
        traj_magnitude = np.sqrt(traj[-1, 0]**2 + traj[-1, 1]**2)
        dot_product = (goal[0] * traj[-1, 0]) + (goal[1] * traj[-1, 1])
        error = dot_product / (goal_magnitude * traj_magnitude)
        error_angle = np.acos(error)
        cost = self.to_goal_cost_gain * error_angle

        return cost


    def dwa_control(self, x, u, goal, ob):
        # Dynamic Window control

        dw = calc_dynamic_window(x)

        u, traj = calc_final_input(x, u, dw, goal, ob)

        return u, traj


    def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
                head_length=width, head_width=width)
        plt.plot(x, y)


def main(gx=10, gy=10):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, np.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])
    # obstacles [x(m) y(m), ....]
    ob = np.array([[-1, -1],
                   [0, 2],
                   [4.0, 2.0],
                   [5.0, 4.0],
                   [5.0, 5.0],
                   [5.0, 6.0],
                   [5.0, 9.0],
                   [8.0, 9.0],
                   [7.0, 9.0],
                   [12.0, 12.0]
                   ])

    u = np.array([0.0, 0.0])

    traj = np.array(x)

    for i in range(1000):
        u, ltraj = dwa_control(x, u, goal, ob)

        x = motion(x, u, dt)
        traj = np.vstack((traj, x))  # store state history

        # print(traj)

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if np.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= self.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()


if __name__ == '__main__':
    main()