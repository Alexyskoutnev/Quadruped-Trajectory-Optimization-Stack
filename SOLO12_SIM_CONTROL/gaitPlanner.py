import time
import numpy as np
import matplotlib.pyplot as plt 

def binomial_factor(n, k):
    return np.math.factorial(n) / (np.math.factorial(k)*np.math.factorial(n - k))

def bezier(t, k, p):
    n = 9
    return p*binomial_factor(n, k)*np.power(t, k)*np.power(1 - t, n - k)

def plot(t, val):
    plt.plot(t, val)



class Gait(object):

    def __init__(self):
        self.feetPose = {"FL": np.zeros(3), "FR": np.zeros(3), "BL": np.zeros(3), "BR": np.zeros(3)}
        self.t = 0
        self.phiStance = 0
        self.lastTime = 0
        self.alpha = 0
        self.s = False
        self.offset = np.array([0, 0.5, 0.5, 0]) #(FR, FL, BR, BL)
        self.bodytofeet_cmd = np.zeros((4, 3))
    

    def calculateStance(self, t, V, angle):
        c = np.cos(angle)
        s = np.sin(angle)
        A = 0.0005
        halfStance =0.05
        p_stance = halfStance*(1 - 2*t)
        stanceX = c * halfStance*(1 - 2*t) * np.abs(V)
        stanceY = -s * halfStance*(1 - 2*t) * np.abs(V)
        stanceZ = -A * np.cos(np.pi/(2 * halfStance)*p_stance)
        return stanceX, stanceY, stanceZ

    def calculateSwing(self, t, velocity, angle):
        


        X_pts = np.abs(velocity) * np.cos(angle) * np.array([-0.05 ,
                                  -0.06 ,
                                  -0.07 , 
                                  -0.07 ,
                                  0. ,
                                  0. , 
                                  0.07 ,
                                  0.07 ,
                                  0.06 ,
                                  0.05 ])
        Y_pts = np.abs(velocity) * np.sin(angle) * np.array([ 0.05 ,
                                   0.06 ,
                                   0.07 , 
                                   0.07 ,
                                   0. ,
                                   -0. , 
                                   -0.07 ,
                                   -0.07 ,
                                   -0.06 ,
                                   -0.05 ])
        Z_pts = np.abs(velocity) * np.array([0. ,
                                0. ,
                                0.05 , 
                                0.05 ,
                                0.05 ,
                                0.06 , 
                                0.06 ,
                                0.06 ,
                                0. ,
                                0. ])

        swingX = 0.0
        swingY = 0.0
        swingZ = 0.0

        for i in range(10):
            swingX += bezier(t, i, X_pts[i])
            swingY += bezier(t, i, Y_pts[i])
            swingZ += bezier(t, i, Z_pts[i])
        
        return swingX, swingY, swingZ




    def stepTrajectory(self, t, velocity, angle, angle_vel, centerTofoot):
        
        stepOffset = 0.75
        if t <= stepOffset:
            T_stance = t/stepOffset
            stepX_pos, stepY_pos, stepZ_pos = self.calculateStance(T_stance, velocity, angle)
            # stepX_rot, stepY_rot, stepZ_rot = self.calculateStance(phiStance, angle_vel, )
        else:
            T_swing = (t - stepOffset)/(1 - stepOffset)
            stepX_pos, stepY_pos, stepZ_pos = self.calculateSwing(T_swing, velocity, angle)

        coord = np.empty(3)
        coord[0] = stepX_pos
        coord[1] = stepY_pos
        coord[2] = stepZ_pos
        return coord

    def runTrajectory(velocity, angle, offset, T, cmd):
        if T <= 0.01:
            T = 0.01
        if (self.t >= 0.99):
            self.lastTime = time.time()   
        self.t = (time.time() - self.lastTime)/T


        #Front-left
        step_coord = self.stepTrajectory(self.t + offset[0], velocity, angle, cmd[0, :])
        self.bodytofeet_cmd[0][0] += step_coord[0] 
        self.bodytofeet_cmd[0][1] += step_coord[1]
        self.bodytofeet_cmd[0][2] += step_coord[2]

        #Front-right
        step_coord = self.stepTrajectory(self.t + offset[1], velocity, angle, cmd[1, :])
        self.bodytofeet_cmd[1][0] += step_coord[0] 
        self.bodytofeet_cmd[1][1] += step_coord[1]
        self.bodytofeet_cmd[1][2] += step_coord[2]

        #Back-left
        step_coord = self.stepTrajectory(self.t + offset[2], velocity, angle, cmd[2, :])
        self.bodytofeet_cmd[2][0] += step_coord[0] 
        self.bodytofeet_cmd[2][1] += step_coord[1]
        self.bodytofeet_cmd[2][2] += step_coord[2]

        #Back-right
        step_coord = self.stepTrajectory(self.t + offset[3], velocity, angle, cmd[3, :])
        self.bodytofeet_cmd[3][0] += step_coord[0] 
        self.bodytofeet_cmd[3][1] += step_coord[1]
        self.bodytofeet_cmd[3][2] += step_coord[2]



        
if __name__ == "__main__":

    t = np.linspace(0, 1, 1000)
    velocity = 1
    X = list()
    Y = list()
    Z = list()
    gait = Gait()
    angle = np.pi/4
    offsets = np.array([0.5, 0.0, 0.0, 0.5])
    for _ in t:
        _X, _Y, _Z = gait.calculateSwing(_, 1, np.pi/3)
        X.append(_X)
        Y.append(_Y)
        Z.append(_Z)
    

    breakpoint()