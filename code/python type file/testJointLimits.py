import numpy as np
import modern_robotics as mr

def testJointLimits(config,J_arm):
    if config[0,5] > -0.1:
        J_arm[:,2] = np.array([0,0,0,0,0,0])
    if config[0,6] > -0.1:
        J_arm[:,3] = np.array([0,0,0,0,0,0])