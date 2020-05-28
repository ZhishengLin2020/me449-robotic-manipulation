import numpy as np
import modern_robotics as mr
from TrajectoryGenerator import TrajectoryGenerator
from NextState import NextState
from FeedbackControl import FeedbackControl
from YouBot import YouBot
import sys

######################################
# generate log
# make a copy of original stdout route
stdout_backup = sys.stdout
# define the log file that receives your log info
log_file = open("C:/Users/10448/Desktop/Lin_Zhisheng/results/newTask/newTaskScript_log.log", "w")
# redirect print output to log file
sys.stdout = log_file

# feedback in log file
print(">> runscript")

# here we generate a reference trajectory
Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
Tsc_initial = np.array([[1,0,0,0.7071],[0,1,0,0.7071],[0,0,1,0.025],[0,0,0,1]])
Tsc_final = np.array([[0,1,0,0],[-1,0,0,-0.5],[0,0,1, 0.025],[0,0,0,1]])
Tce_grasp = np.array([[-np.sqrt(2)/2,0,np.sqrt(2)/2,0],[0,1,0,0],[-np.sqrt(2)/2,0,-np.sqrt(2)/2,0],[0,0,0,1]])
Tce_standoff = np.array([[-np.sqrt(2)/2,0,np.sqrt(2)/2,0],[0,1,0,0],[-np.sqrt(2)/2,0,-np.sqrt(2)/2,0.1],[0,0,0,1]])
k = 1
Tse_post = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k)
print("Generating input trajectory:")
print(Tse_post)

# set up initial robot configuration
config_initial = np.array([np.pi/10,-0.5,0.5,np.pi/10,-np.pi/10,-np.pi/10,-np.pi/10,0,0,0,0,0,0]).reshape(1,13)
new_config = config_initial
print("Generating initial input configuration:")
print(new_config)

# determine wehther using joint limits
jointlimit = 0
print("Generating input joint limits:")
print(jointlimit)

# here we set ups Kp and Ki
Kp = 2.2
Ki = 0
print("Generating input Kp and Ki:")
print(Kp,end=' ')
print(Ki)

# here we call YouBot function
print("Calling fucntion YouBot.")
YouBot(Tse_post,new_config,Kp,Ki,jointlimit)

# feedback in log file
print("Generating animation csv file.")
print("Writing error plot data.")
print("Done.")
print(">>")

# log file close
log_file.close()
# restore the output to initial pattern
sys.stdout = stdout_backup