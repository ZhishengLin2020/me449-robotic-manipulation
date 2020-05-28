Controller: feedforward-plus-PI controller
Feedback Gains: Kp=40, Ki=30
Initial Configuration: config_initial = np.array([np.pi/10,-1,1.5,0,-np.pi/10,-np.pi/6,-np.pi/10,0,0,0,0,0,0]).reshape(1,13)

In this overshoot condition, we can clearly see that there is a little shaking in the animation and some wave at first in the error figure,
but finally, the error is eliminated.