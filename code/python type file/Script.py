import numpy as np
import modern_robotics as mr
from TrajectoryGenerator import TrajectoryGenerator
from NextState import NextState
from FeedbackControl import FeedbackControl
from YouBot import YouBot

# here we generate a reference trajectory
Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
Tsc_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1, 0.025],[0,0,0,1]])
Tce_grasp = np.array([[-np.sqrt(2)/2,0,np.sqrt(2)/2,0],[0,1,0,0],[-np.sqrt(2)/2,0,-np.sqrt(2)/2,0],[0,0,0,1]])
Tce_standoff = np.array([[-np.sqrt(2)/2,0,np.sqrt(2)/2,0],[0,1,0,0],[-np.sqrt(2)/2,0,-np.sqrt(2)/2,0.1],[0,0,0,1]])
k = 1
Tse_post = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k)

# set up initial robot configuration
config_initial = np.array([np.pi/10,-0.5,0.5,np.pi/10,-np.pi/10,-np.pi/10,-np.pi/10,0,0,0,0,0,0]).reshape(1,13)
new_config = config_initial

# determine wehther using joint limits
jointlimit = 0

# here we set ups Kp and Ki
Kp = 2.2
Ki = 0

# here we call YouBot function
YouBot(Tse_post,new_config,Kp,Ki,jointlimit)

