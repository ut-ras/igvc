import rospy, pylab, math, random, numpy, time

class ExtendedKalmanFilter:
    def __init__(self, _G_funct, _G_jacobian_funct, _H_functs, _H_jacobian_functs, _state, _P, _Q, _R_arr):
        self.G_funct = _G_funct                     # Transition function
        self.G_jacobian_funct = _G_jacobian_funct   # Transition jacobian
        self.H_functs = _H_functs                   # Observation functions
        self.H_jacobian_functs = _H_jacobian_functs # Observation matrices
        self.state = _state                         # Initial state estimate
        self.P = _P                                 # Initial covariance estimate
        self.Q = _Q                                 # Estimated variance in process
        self.R_arr = _R_arr                         # Estimated variances in measurements
        self.I = numpy.eye(_P.shape[0])             # Identity matrix
        self.prev_time = None

    def GetCurrentCovMatrix(self):
        return self.P

    def GetCurrentState(self):
        return self.state

    def CalcDT(self, cur_time):
        dt = 0
        if self.prev_time != None:
            dt = cur_time - self.prev_time
        self.prev_time = cur_time
        return dt

    def Predict(self, dt=None):
        if dt == None:
            dt = self.CalcDT(rospy.get_time())

        G_jacobian = self.G_jacobian_funct(self.state, dt)

        #---------------------------Prediction step-----------------------------
        self.state = self.G_funct(self.state, dt)
        self.P = (G_jacobian * self.P) * numpy.transpose(G_jacobian) + self.Q

    def Step(self, sensor_index, measurement_vector, dt=None):
        if dt == None:
            dt = self.CalcDT(rospy.get_time())

        G_jacobian = self.G_jacobian_funct(self.state, dt)

        #---------------------------Prediction step-----------------------------
        predicted_state = self.G_funct(self.state, dt)
        predicted_P = (G_jacobian * self.P) * numpy.transpose(G_jacobian) + self.Q

        #--------------------------Observation step-----------------------------
        H_funct = self.H_functs[sensor_index]
        H_jacobian = self.H_jacobian_functs[sensor_index](self.state, dt)
        R = self.R_arr[sensor_index]

        innovation = measurement_vector - H_funct(predicted_state, dt)
        innovation_covariance = H_jacobian*predicted_P*numpy.transpose(H_jacobian) + R

        #-----------------------------Update step-------------------------------
        kalman_gain = predicted_P * numpy.transpose(H_jacobian) \
                * numpy.linalg.inv(innovation_covariance)
        self.state = predicted_state + kalman_gain * innovation
        self.P = (self.I - kalman_gain*H_jacobian)*predicted_P


def orientation_observation_funct(state, dt):
    z = state[2, 0]
    w = state[4, 0]
    r = state[6, 0]
    p = state[7, 0]

    return numpy.matrix(
        [ [r],\
          [p],\
          [z + dt*w] ])

def orientation_jacobian_funct(state, dt):
    return numpy.matrix(
        [ [0, 0, 0, 0,  0, 0, 1, 0],\
          [0, 0, 0, 0,  0, 0, 0, 1],\
          [0, 0, 1, 0, dt, 0, 0, 0] ])

def velocity_observation_funct(state, dt):
    v = state[3, 0]
    w = state[4, 0]
    a = state[5, 0]

    return numpy.matrix(
        [ [v + dt*a],\
          [w] ])

def velocity_jacobian_funct(state, dt):
    return numpy.matrix(\
        [ [0, 0, 0, 1, 0, dt, 0, 0],\
          [0, 0, 0, 0, 1,  0, 0, 0] ])

def position_observation_funct(state, dt):
    x = state[0, 0]
    y = state[1, 0]
    z = state[2, 0]
    v = state[3, 0]
    w = state[4, 0]
    a = state[5, 0]

    return numpy.matrix(
          [ [x + (dt*v + 0.5*dt*dt*a)*math.cos(z + dt*w)],\
            [y + (dt*v + 0.5*dt*dt*a)*math.sin(z + dt*w)]])

def position_jacobian_funct(state, dt):
    x = state[0, 0]
    y = state[1, 0]
    z = state[2, 0]
    v = state[3, 0]
    w = state[4, 0]
    a = state[5, 0]
    r = state[6, 0]
    p = state[7, 0]

    return numpy.matrix(
        [ [1, 0, -dt*(v + .5*a*dt)*math.sin(z + dt*w), dt*math.cos(z + dt*w),\
                 -dt*dt*(v + .5*a*dt)*math.sin(z + dt*w), .5*dt*dt*math.cos(z + dt*w), 0, 0],\
          [0, 1,  dt*(v + .5*a*dt)*math.cos(z + dt*w), dt*math.sin(z + dt*w),\
                  dt*dt*(v + .5*a*dt)*math.cos(z + dt*w), .5*dt*dt*math.sin(z + dt*w), 0, 0] ])

def transition_funct(state, dt):
    x = state[0, 0] # x pos
    y = state[1, 0] # y pos
    z = state[2, 0] # yaw
    v = state[3, 0] # linear velocity
    w = state[4, 0] # angular velocity (about yaw)
    a = state[5, 0] # linear acceleration
    r = state[6, 0] # roll
    p = state[7, 0] # pitch

    return numpy.matrix(
        [ [x + (dt*v + 0.5*dt*dt*a)*math.cos(z + dt*w)],\
          [y + (dt*v + 0.5*dt*dt*a)*math.sin(z + dt*w)],\
          [z + dt*w],\
          [v + dt*a],\
          [w],\
          [a],\
          [r],\
          [p] ])

def transition_jacobian_funct(state, dt):
    z = state[2, 0]
    v = state[3, 0]
    w = state[4, 0]
    a = state[5, 0]

    return numpy.matrix(
        [ [1, 0, -dt*(v + .5*a*dt)*math.sin(z + dt*w), dt*math.cos(z + dt*w),\
                 -dt*dt*(v + .5*a*dt)*math.sin(z + dt*w), .5*dt*dt*math.cos(z + dt*w), 0, 0],\
          [0, 1,  dt*(v + .5*a*dt)*math.cos(z + dt*w), dt*math.sin(z + dt*w),\
                  dt*dt*(v + .5*a*dt)*math.cos(z + dt*w), .5*dt*dt*math.sin(z + dt*w), 0, 0],\
          [0, 0, 1, 0, dt,  0, 0, 0],\
          [0, 0, 0, 1,  0, dt, 0, 0],\
          [0, 0, 0, 0,  1,  0, 0, 0],\
          [0, 0, 0, 0,  0,  1, 0, 0],\
          [0, 0, 0, 0,  0,  0, 1, 0],\
          [0, 0, 0, 0,  0,  0, 0, 1] ])

