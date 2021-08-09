import numpy as np
import cv2
import matplotlib.pyplot as plt
import math

from simple_pid import PID
from InvertedPendulum import InvertedPendulum
from scipy.integrate import solve_ivp

global angles
global desangles 
global error
global e_thetarr


pid = PID(1, 0, 0, setpoint=1)

def sign(v):
    if v > 0:
        return 1
    elif v < 0:
        return -1
    elif v == 0:
        return 0
    else:
        return v

# Pendulum. Cart is fixed and cannot move.
# Y : [ theta, theta_dot ]
# returns expression for Y_dot.
angles = [None]
desangles = [None]
error = [None]
e_thetarr = [None]
def func( t, y ):
    g = 9.8 # Gravitational Acceleration
    L = 1.5 # Length of pendulum
    m = 1.0 #mass of bob (kg)
    I = 0.5*L*(m*m)
    #damping =  - 0.5*y[1]
    #y[0] = pid(-y[0])
	#y[0] = pid (y[0])
    lmd = 10
    rho = 10
    theta_des = math.pi/2
    
    e_theta = theta_des - y[0]
    e_dtheta = 0.008
    # if(e_theta < 10*math.pi/180 and e_theta > -8*math.pi/180):
        # e_theta = 0
        # e_dtheta = -y[1]
        # print (e_theta)

    s = -e_dtheta + lmd * e_theta

    #Sdot = -y[1] * sign(y[1]) * sign(s) + lmd * y[0]
        
    #ut=-1*sign(y[1])*sign(y[1]+lmd*y[0]-lmd*theta_des)*Sdot
    #(35*((math.pi/2)-y[0]))
    ut = 10 * e_dtheta + rho * sign(s)
    theta_ddot = -g/L * np.cos( y[0] ) - 0.01*y[1] + (1/I)*ut
    angles.append(y[0]*180/math.pi)
    desangles.append(theta_des*180/math.pi)
    e_thetarr.append(e_theta)
    # error.append(float(0.05002748160656552))
    # error.append(float(0.04878980965850502))
    # error.append(0.04882026451731085)
    # error.append(0.04880671634751971)
    # error.append(0.04889104126249508)
    # error.append(0.04882000430993505)
    # error.append(0.048811949743908234)
    # error.append(0.0488302062104195)
    # error.append(0.048836945914218)
    # error.append(0.0488060786397746)
    # error.append(0.048871179777911944)
    # print(theta_des*180/math.pi)

    return [ y[1], theta_ddot ]


# Only the pendulum moves the cart is stationary
if __name__=="__main__":
    # Solve ODE: theta_dot_dot = -g / L * cos( theta ) + delta * theta_dot
    #       Use state y as `[ theta ; theta_dot ]`. Then the above equation can
    #       be written as a function of y and t. `y_dot = f( t, y )`.
    #
    #  Given an inital state `y0` and limits of integration, we know the trajectory
    #       followed by the pendulum. The solution of this ODE is plotted with
    #       as a simulation.
	#print(y[0])
    
    sol = solve_ivp(func, [0, 20], [ 100*math.pi/180 + 0.1, 0 ],   t_eval=np.linspace( 0, 20, 100)  )

    # plt.plot([0, 0, 0], label='Measured Angle')
    # plt.plot(desangles, label='Desired Angle')
    # plt.plot(error,  label='Error')
    # plt.plot([0.8, 0.6, 0.45, 0.22, 0.1, 0.05002748160656552,0.04878980965850502,0.04882026451731085,0.04880671634751971,\
    #     0.04889104126249508,0.04882000430993505,0.048811949743908234,0.0488302062104195,\
    #         0.048836945914218,0.0488060786397746,0.048871179777911944], label='error')
    plt.plot(e_thetarr, label='error')
    plt.ylabel('error')
    plt.xlabel('t')
    plt.legend(loc='upper right')
    syst = InvertedPendulum()
    plt.show()

    for i, t in enumerate(sol.t):
        rendered = syst.step( [0,1, sol.y[0,i], sol.y[1,i] ], t )
        cv2.imshow( 'im', rendered )

        if cv2.waitKey(30) == ord('q'):
            break
