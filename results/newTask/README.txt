Controller: feedforward-plus-P controller
Feedback Gains: Kp=2.2, Ki=0
config_initial = np.array([np.pi/10,-0.5,0.5,np.pi/10,-np.pi/10,-np.pi/10,-np.pi/10,0,0,0,0,0,0]).reshape(1,13)
Initial Cube Configuration: Tsc_initial = np.array([[1,0,0,0.7071],[0,1,0,0.7071],[0,0,1,0.025],[0,0,0,1]])
Final Cube Configuration: Tsc_final = np.array([[0,1,0,0],[-1,0,0,-0.5],[0,0,1, 0.025],[0,0,0,1]])

In this newTask condition, I use the same configuration as the best condition but I use the different cube initial and final condition.
