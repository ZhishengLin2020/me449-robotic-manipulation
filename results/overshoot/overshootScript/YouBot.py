import numpy as np
import modern_robotics as mr
from TrajectoryGenerator import TrajectoryGenerator
from NextState import NextState
from FeedbackControl import FeedbackControl
import matplotlib.pyplot as plt

def YouBot(Tse_post,new_config,Kp,Ki,jointlimit):
    # here we loop
    dt = 0.01
    new_config_arr = []
    X_err_arr = []
    new_config_arr.append(new_config)
    for i in np.arange(np.shape(Tse_post)[0]-1):
        # here we write down Xd
        Xd = np.array([[Tse_post[i,0],Tse_post[i,1],Tse_post[i,2],Tse_post[i,9]],[Tse_post[i,3],Tse_post[i,4],Tse_post[i,5],Tse_post[i,10]],[Tse_post[i,6],Tse_post[i,7],Tse_post[i,8],Tse_post[i,11]],[0,0,0,1]])
        print("Computing Xd:")
        print(Xd)
    
        # here we write down Xd_next
        Xd_next = np.array([[Tse_post[i+1,0],Tse_post[i+1,1],Tse_post[i+1,2],Tse_post[i+1,9]],[Tse_post[i+1,3],Tse_post[i+1,4],Tse_post[i+1,5],Tse_post[i+1,10]],[Tse_post[i+1,6],Tse_post[i+1,7],Tse_post[i+1,8],Tse_post[i+1,11]],[0,0,0,1]])
        print("Computing Xd_next:")
        print(Xd_next)

        # here we compute speeds and X_err
        V,speeds,X_err = FeedbackControl(new_config,Xd,Xd_next,Kp,Ki,dt,jointlimit)
        print("Calling function FeedbackControl and return speeds and X_err:")
        print("speeds:")
        print(speeds)
        print("X_err:")
        print(X_err)
        print("Appending this X_err.")

        # here we adjust the order of wheels speeds and joints speeds
        speeds = np.array([speeds[4],speeds[5],speeds[6],speeds[7],speeds[8],speeds[0],speeds[1],speeds[2],speeds[3]]).reshape(1,9)
        speeds_max = 1000
        print("Adjusting the order of wheels and joints speeds:")
        print(speeds)
        print("Setting up speeds_max:")
        print(speeds_max)

        # here we compute new configuration
        new_config = NextState(new_config,speeds,dt,speeds_max)
        new_config = np.append(new_config,[[Tse_post[i,12]]],axis=1)
        new_config_arr.append(new_config)
        X_err_arr.append(X_err)
        print("Calling function NextState and return new configuration:")
        print(new_config)
        print("Appending this new configuration.")

    new_config_arr = np.squeeze(new_config_arr)
    X_err_arr = np.squeeze(X_err_arr)
    print("Generating the final new configuration array:")
    print(new_config_arr)
    print("Generating the final X_err array:")
    print(X_err_arr)

    ##################################
    # generate YouBot.csv file
    # Open a file for output
    # Overwrite
    f = open("C:/Users/10448/Desktop/Lin_Zhisheng/results/overshoot/overshootScript/YouBot.csv", "w") 
    # give y Tse_post
    y = new_config_arr
    # For loop running to print each csv row
    for i in range(np.shape(y)[0]):
        output = "%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f\n" % (y[i,0],y[i,1],y[i,2],y[i,3],y[i,4],y[i,5],y[i,6],y[i,7],y[i,8],y[i,9],y[i,10],y[i,11],y[i,12])
        f.write(output)
    # close file
    f.close()

    ##################################
    # generate X_err.csv file
    # Open a file for output
    # Overwrite
    f = open("C:/Users/10448/Desktop/Lin_Zhisheng/results/overshoot/overshootScript/X_err.csv", "w") 
    # give y Tse_post
    z = X_err_arr
    # For loop running to print each csv row
    for i in range(np.shape(z)[0]):
        output = "%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f\n" % (z[i,0],z[i,1],z[i,2],z[i,3],z[i,4],z[i,5])
        f.write(output) 
    # close file
    f.close()

    ##################################
    # here we plot
    # load X_err.csv
    X = np.array(np.genfromtxt('C:/Users/10448/Desktop/Lin_Zhisheng/results/overshoot/overshootScript/X_err.csv', delimiter=','))

    # set up steps list
    steps = np.arange(2227)

    # plot
    plt.figure(dpi=110,facecolor='w')
    plt.plot(steps,X[:,0])
    plt.plot(steps,X[:,1])
    plt.plot(steps,X[:,2])
    plt.plot(steps,X[:,3])
    plt.plot(steps,X[:,4])
    plt.plot(steps,X[:,5])
    plt.title("Figure of X_err")
    plt.xlabel('Stpes')
    plt.ylabel('Error')
    plt.legend([r'$w_{e1}$',r'$w_{e2}$',r'$w_{e3}$',r'$\dot{p}_{e1}$',r'$\dot{p}_{e2}$',r'$\dot{p}_{e3}$'])
    plt.grid(True)
    plt.show()