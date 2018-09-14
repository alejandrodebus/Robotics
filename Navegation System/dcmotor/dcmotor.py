# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

import rk4
import pid

plt_titles = ("$\\theta$","$\omega$","$i$","$u_k$")

Nx = 3
Ny = 1
Nu = 1

# Lego parameters
# http://www.mathworks.com/matlabcentral/fileexchange/35206-simulink-support-package-for-lego-mindstorms-nxt-hardware--r2012a-/content/lego/legodemos/lego_selfbalance_plant.m
# http://www.nt.ntnu.no/users/skoge/prost/proceedings/ecc-2013/data/papers/0959.pdf
# DC motor state space system model
# http://ctms.engin.umich.edu/CTMS/index.php?example=MotorPosition&section=SystemModeling

fm = 0.0022     # DC motor friction coefficient [Nm rad s^−1]
Jm = 1e-5       # DC motor inertia moment [kg m^2]
Rm = 6.69       # DC motor resistance [Ω]
Kb = 0.468      # DC motor back EMF constant [V sec rad^-1]
Kt = 0.317      # DC motor torque constant [Nm A^-1]
Gu = 1E-2       # PWM gain factor
# Vb = 8.00       # V Power Supply voltage
Vo = 0.625      # V Power Supply offset
mu = 1.089      # Power Supply gain factor
L = 1.0         # DC motor electric inductance [H]
ta = 4e-3       # DC motor actuator delay [s]


def dcmotor_model(x, u=0.0, Vb=8.00):
    """
    Model for a Lego NXT Mindstorms DC motor.
    x[0] -> DC motor position θ
    x[1] -> DC motor speed ω
    x[2] -> DC motor armature current i
    """
    v = Gu*(mu*Vb-Vo)*u
    
    return np.array([         x[1],
                     -(fm/Jm)*x[1] + (Kt/Jm)*x[2],
                     -(Kb/L)* x[1] - (Rm/L)* x[2] + v/L
                     ])

def dcmotor_measurement(x, nxt_sim=False):
    if nxt_sim:
        x_deg = np.rad2deg(x[0])
        x_deg = np.around(x_deg, decimals=0)
        return np.deg2rad(x_deg)
    else:
        return np.array([x[0]]).flatten()
    
dt = 0.15
x0 = np.zeros((Nx,))
Vb0 = 8.0
Nsim = 50
tsim = np.arange(0,Nsim)*dt

u_max =  95.0
u_min = -95.0

xk = np.zeros((Nx,Nsim+1))
xk[:,0] = x0
uk = np.zeros((Nu,Nsim))

# Velocity control
Kp =  0.5
Ki = 20.0
Kd =  0.0
pid_vel = pid.PID(Kp, Ki, Kd, dt, u_max, u_min)

reference = 5.0 # rad/seg
prev_pos = 0.0

for k in xrange(Nsim):
    measurement = dcmotor_measurement(xk[:,k])
    omega = (measurement-prev_pos)/dt
    prev_pos = measurement
    uk[:,k] = pid_vel.get_output(reference, omega)
    xk[:, k+1] = rk4.rk4(dcmotor_model, xk[:,k], [uk[:,k], Vb0], dt, 50)

f, axarr = plt.subplots(4,1)
# First, we plot the reference.
# Then the state evolution.
# And at last, the controls.
axarr[1].plot(tsim,reference*np.ones((Nsim,)),'r', linewidth=4)
for i in xrange(3):
    axarr[i].plot(tsim,xk[i,:-1],".-", color='k')
    axarr[i].set_title(plt_titles[i])
    axarr[i].grid()

axarr[3].step(tsim,uk.T,"k.-", where='post')
axarr[3].set_title('$u_k$')
axarr[3].grid()
plt.tight_layout()



xk = np.zeros((Nx,Nsim+1))
xk[:,0] = x0
uk = np.zeros((Nu,Nsim))

# Position control
xk = np.zeros((Nx,Nsim+1))
xk[:,0] = x0
uk = np.zeros((Nu,Nsim))

Kp = 40.0
Ki =  0.0
Kd =  1.0
pid_pos = pid.PID(Kp, Ki, Kd, dt, u_max, u_min)

reference = 12.0

for k in xrange(Nsim):
    theta = dcmotor_measurement(xk[:,k])
    uk[:,k] = pid_pos.get_output(reference, theta)
    xk[:, k+1] = rk4.rk4(dcmotor_model, xk[:,k], [uk[:,k], Vb0], dt, 50)

f, axarr = plt.subplots(4,1)
# First, we plot the reference.
# Then the state evolution.
# And at last, the controls.
axarr[0].plot(tsim,reference*np.ones((Nsim,)),'r', linewidth=4)
for i in xrange(3):
    axarr[i].plot(tsim,xk[i,:-1],".-", color='k')
    axarr[i].set_title(plt_titles[i])
    axarr[i].grid()

axarr[3].step(tsim,uk.T,"k.-", where='post')
axarr[3].set_title('$u_k$')
axarr[3].grid()
plt.tight_layout()