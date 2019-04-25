import numpy as np
import scipy
import config

from rvo import RVO_update, reach, compute_V_des, reach

X = 0
Y = 1
YAW = 2

def euclidian_norm(first, last=[0,0]):
  return np.sqrt((first[X] - last[X])**2 + (first[Y] - last[Y])**2)


# two separate PIDs, for u and w
class pid(object): 
    def __init__(self, Pu=0.25, Iu=0, Du=0, Pw=1, Iw=0.0001, Dw=0.): 
    # def __init__(self, Pu=0.0003, Iu=0.01, Du=0.0, Pw=0.0001, Iw=0.001, Dw=0.0): 
        self._Pu = Pu
        self._Iu = Iu
        self._Du = Du
        self._Pw = Pw
        self._Iw = Iw
        self._Dw = Dw
        self._last_u_error = 0.
        self._last_w_error = 0.
        self._u_integral  = 0.
        self._w_integral  = 0.
        self._prev_position = [(0,0)]
        self._distance_error = 0.
        self._orientation_error = 0.

    def reset(self):
        self._last_u_error = 0.
        self._last_w_error = 0. 
        self._u_integral  = 0. 
        self._w_integral  = 0.
        self._prev_position = [(0,0)]
    
    # scale it to step response
    def compute_commands(self, current_pose, goal_position, obstacles, dt):
        
        self._distance_error    = euclidian_norm(current_pose[0:2], goal_position) - config.MIN_DISTANCE_TO_TARGET - 3*config.ROBOT_RADIUS
        self._orientation_error = np.arctan2(goal_position[Y], goal_position[X]) - current_pose[YAW]

        # compute errors for u and w
        u_error = self._distance_error * np.cos(self._orientation_error)
        w_error = self._orientation_error 

        # print (self._distance_error)
        
        print (dt)

        # compute the integrals
        self._u_integral += u_error * dt
        self._w_integral += w_error * dt

        # compute the derivatives
        u_Derivative = 0.0
        w_Derivative = 0.0

        delta_u_error = u_error - self._last_u_error
        delta_w_error = w_error - self._last_w_error

        if dt > 0: 
            u_Derivative = self._Du * delta_u_error/dt
            w_Derivative = self._Dw * delta_w_error/dt

        u = u_error * self._Pu + self._Iu * self._u_integral + self._Du * u_Derivative
        w = w_error * self._Pw + self._Iw * self._w_integral + self._Dw * w_Derivative
        # w = -w
        # remember the errors 
        self._last_u_error = u_error
        self._last_w_error = w_error

        if u > config.SPEED:
            u = config.SPEED

        if u < 0: 
            u = 0
        
        if np.abs(w) > config.SPEED:
            w = w/w*config.SPEED

        if  np.abs(self._orientation_error) < 0.1: 
            w = 0

        print(u, w, u_error, w_error )

        return u, w 

class dynamic(object):

    def __init__(self):
        #define workspace model
        self.ws_model = dict()
        #robot radius
        self.ws_model['robot_radius'] = 2*config.ROBOT_RADIUS
        self.ws_model['circular_obstacles'] = []
        self.ws_model['boundary'] = [] 
        self.prev_speed = [0,0]


    
    # for an obstacle we get x, y, speed_x, speed_y, radius, real_radius
    def compute_commands(self, current_pose, goal_position, obstacles, dt):

        if obstacles == None:
            return 0, 0

        if euclidian_norm(goal_position) < config.MIN_DISTANCE_TO_TARGET + 2*config.ROBOT_RADIUS:
            return 0, 0

        X_c     = []
        V_c     = []
        goal    = []
        V_max = [config.SPEED/2 for i in range(len(obstacles)+1)]

        # data for the first robot, which is our robot
        X_c.append([0,0])
        V_c.append(self.prev_speed)
        goal.append(goal_position)
        
        # print (V)
        for obstacle in obstacles:
            X_c.append([obstacle[0], obstacle[1]])
            V_c.append([obstacle[2], obstacle[3]])
            goal.append([0,0]) # we simulate that each obstacle wants to go to the goal

        V_des = compute_V_des(X_c, goal, V_max)
        # print (V)
        # compute the optimal vel to avoid collision
        V_c = RVO_update(X_c, V_des, V_c, self.ws_model)
        
        velocity = V_c[0]

        # compute the yaw from the actual speed
        yaw = np.arctan2(velocity[Y], velocity[X])

        # feedback linearization
        u = velocity[X] * np.cos(yaw) + velocity[Y] * np.sin(yaw)
        w = 1.0 / 0.001 * ( -1.0 * velocity[X] * np.sin(yaw) + velocity[Y] * np.cos(yaw))

        print (u, w, yaw, velocity[0], velocity[1])

        if u > config.SPEED:
            u = config.SPEED
        
        if np.abs(w) > config.SPEED:
            w = w/w*config.SPEED

        return u, w


# class feedbackLinearized(object): 
#     # Follow path using feedback linearization.

#     def __init__(self):
#         pass

#     def feedback_linearized(self, pose, velocity, epsilon):
#         u = 0.  # [m/s]
#         w = 0.  # [rad/s] going counter-clockwise.

#         # MISSING: Implement feedback-linearization to follow the velocity
#         # vector given as argument. Epsilon corresponds to the distance of
#         # linearized point in front of the robot.

#         u = velocity[X] * np.cos(pose[YAW]) + velocity[Y] * np.sin(pose[YAW])
#         w = 1.0 / epsilon * ( -1.0 * velocity[X] * np.sin(pose[YAW]) + velocity[Y] * np.cos(pose[YAW]))

#         return u, w

#     def get_velocity(self, position, path_points):
#         v = np.zeros_like(position)
#         if len(path_points) == 0:
#             return v
#         # Stop moving if the goal is reached.
#         if np.linalg.norm(position - path_points[-1]) < .2:
#             return v

#         # MISSING: Return the velocity needed to follow the
#         # path defined by path_points. Assume holonomicity of the
#         # point located at position.

#         # first point in the path_points is the point we need to head to
#         # then we compute YAW angle to head that closest point on the path 
#         for point in reversed(path_points):
            
#             # we should get the speed for the point that corresponds to the next position, 
#             # which is the first satisfying the condition, given reverse traversal of the list
#             if np.linalg.norm(position - point) < 0.1: # and idx+1 < len(path_points): 
            
#             # compute the heading
#             d = point - position
#             theta = np.arctan2(d[Y], d[X])

#             # compute the speed and return it
#             v = np.array((SPEED*np.cos(theta), SPEED*np.sin(theta)))
#             return v

#         return v


#     def compute(self, x):
#         position = np.array([
#             slam.pose[X] + EPSILON * np.cos(slam.pose[YAW]),
#             slam.pose[Y] + EPSILON * np.sin(slam.pose[YAW])], dtype=np.float32)
#         v = get_velocity(position, np.array(current_path, dtype=np.float32))
#         u, w = feedback_linearized(slam.pose, v, epsilon=EPSILON)
#         vel_msg = Twist()
#         vel_msg.linear.x = u
#         vel_msg.angular.z = w
#         return vel_msg
