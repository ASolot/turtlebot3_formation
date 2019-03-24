import numpy as np
import scipy


class pid(object): 

    def __init__(self, P, I, D): 
        self.P = P
        self.I = I
        self.D = D
        self._u_error = 0
        self._w_error = 0

        

    def reset(self):
        pass
    
    def compute(self, position, goal):
        pass



# TODO: implement the controller
class lqr(object): 

    def __init__(self, A, B, Q, R): 
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

    def compute(self, x): 

        X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
     
        #compute the LQR gain
        K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))
     
        eigVals, eigVecs = scipy.linalg.eig(A-B*K)
     
        return K, X, eigVals


class feedbackLinearized(object): 
    # Follow path using feedback linearization.

    def __init__(self):
        pass

    def feedback_linearized(self, pose, velocity, epsilon):
        u = 0.  # [m/s]
        w = 0.  # [rad/s] going counter-clockwise.

        # MISSING: Implement feedback-linearization to follow the velocity
        # vector given as argument. Epsilon corresponds to the distance of
        # linearized point in front of the robot.

        u = velocity[X] * np.cos(pose[YAW]) + velocity[Y] * np.sin(pose[YAW])
        w = 1.0 / epsilon * ( -1.0 * velocity[X] * np.sin(pose[YAW]) + velocity[Y] * np.cos(pose[YAW]))

        return u, w

    def get_velocity(self, position, path_points):
        v = np.zeros_like(position)
        if len(path_points) == 0:
            return v
        # Stop moving if the goal is reached.
        if np.linalg.norm(position - path_points[-1]) < .2:
            return v

        # MISSING: Return the velocity needed to follow the
        # path defined by path_points. Assume holonomicity of the
        # point located at position.

        # first point in the path_points is the point we need to head to
        # then we compute YAW angle to head that closest point on the path 
        for point in reversed(path_points):
            
            # we should get the speed for the point that corresponds to the next position, 
            # which is the first satisfying the condition, given reverse traversal of the list
            if np.linalg.norm(position - point) < 0.1: # and idx+1 < len(path_points): 
            
            # compute the heading
            d = point - position
            theta = np.arctan2(d[Y], d[X])

            # compute the speed and return it
            v = np.array((SPEED*np.cos(theta), SPEED*np.sin(theta)))
            return v

        return v


    def compute(self, x):
        position = np.array([
            slam.pose[X] + EPSILON * np.cos(slam.pose[YAW]),
            slam.pose[Y] + EPSILON * np.sin(slam.pose[YAW])], dtype=np.float32)
        v = get_velocity(position, np.array(current_path, dtype=np.float32))
        u, w = feedback_linearized(slam.pose, v, epsilon=EPSILON)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        return vel_msg
