import numpy as np
import matplotlib.pyplot as plt

from nxt_lib.nxt_list import nxt_list, data_idx
import nxt_lib.brick as brick

import model_lego as model_lego
import dcmotor.pid as pid
import dcmotor.rk4 as rk4
import angle_choosing as angles
import dcmotor.rk4 as rk4

plt_titles = ("$\\theta$","$\omega Estimado$","$u_k$","$u_k$")

Nx = 3
Ny = 2
Nu = 2



dt = 0.15
x0 = np.zeros((Nx,))
Vb0 = 8.0
Nsim = 500
tsim = np.arange(0,Nsim)*dt

u_max =  95.0
u_min = -95.0

# Connect to Lego NXT
nxt_conf = nxt_list["03"]
nxt = brick.Brick(nxt_conf["name"], nxt_conf["mac"], nxt_conf["port"])
nxt.connect()

do_loop = True
first_run = True
t = 0


if nxt.sock.connected:
    raw_input("Press enter to continue...")
    
    # Position control
    """
    Model for a differential drive mobile robot.
    x[0] -> x
    x[1] -> y
    x[2] -> \psi
    x[3] -> \theta_l
    x[4] -> \omega_l
    x[5] -> i_l
    x[6] -> \theta_r
    x[7] -> \omega_r
    x[8] -> i_r
    """
 
    uk = np.zeros((Nu,Nsim+1))
    
    omega = np.zeros((Nx,Nsim+1))
       
    v_max =  0.6
    v_min = -0.6
    w_max =  np.deg2rad(100)
    w_min = -np.deg2rad(100)
   

    Kp = 2.7
    Ki =  0.01
    Kd =  0.0

    Kp_v =  2.5#2.5
    Ki_v =  0#0.01
    Kd_v =  0#0.0
    
    Kp_w = 1.86 #1.75 - 1.78
    
    Ki_w =  0.00
    Kd_w =  0.0


    pid_vel_l = pid.PID(Kp, Ki, Kd, dt, u_max, u_min)
    pid_vel_r = pid.PID(Kp, Ki, Kd, dt, u_max, u_min)
    
    pid_v = pid.PID(Kp_v, Ki_v, Kd_v, dt, v_max, v_min)
    pid_w = pid.PID(Kp_w, Ki_w, Kd_w, dt, w_max, w_min)    
    

    
    prev_pos = np.zeros((Ny,))
    n=0
    
    # **** LISTA DE PUNTOS ****
    x_target = np.array([[0.90, -0.90], [0.90 , 0.00],[0.00, -0.9],[0.0, 0.0]])

    w_target_prev = 0.0
    
    
    measurement_prev_2 = np.zeros((Ny,))
    measurement_prev_1 = np.zeros((Ny,))
    measurement = np.zeros((Ny,))
    
    xhat = np.zeros((Nx, Nsim+1))
    
    lin_ang_vel = np.zeros((2,Nsim))
    w_ang=np.zeros((Nsim))
    w_ang_nuevo=np.zeros((Nsim))
    
    
    for k in xrange(Nsim):
        _data = nxt.recv_data()
        if _data:
            
            print k
            print _data
            measurement[0]= np.deg2rad(_data[4])
            measurement[1]= np.deg2rad(_data[5])
            
            
            travelled_distance = model_lego.wheel_radius * (3.0 * measurement - 4.0 * measurement_prev_1 + measurement_prev_2) / 2.0 
        
#            travelled_distance = model_lego.wheel_radius * (measurement - measurement_prev_1)
            xhat[0, k + 1] = xhat[0, k] + 0.5 * (travelled_distance[0] + travelled_distance[1]) * np.cos(xhat[2, k])#distancia recorrida en x
            xhat[1, k + 1] = xhat[1, k] + 0.5 * (travelled_distance[0] + travelled_distance[1]) * np.sin(xhat[2, k])#distancia recorrida en y
            xhat[2, k + 1] = xhat[2, k] + 1.0*(travelled_distance[1] - travelled_distance[0]) / model_lego.wheel_distance#cuanto giro todo sumado a los vcalores anteriores para seguir el recorrido          
            
            #x_target[0] = point_list_x[n] 
            #x_target[1] = point_list_y[n]
            w_target = np.arctan2((x_target[n,1] - xhat[1, k]), (x_target[n,0] - xhat[0, k]))
            
            w_target = np.unwrap([w_target_prev, w_target])[-1]
            
            w_ang_nuevo[k]=angles.choose_heading(angles.get_theta([xhat[0,k],xhat[1,k]], [x_target[n,0],x_target[n,1]], xhat[2,k]))
            w_ang[k] = w_target 
            d_target = np.linalg.norm(x_target[n]- xhat[:2, k])
            print 'w_target: ', np.rad2deg(w_target)
            print 'XHAT_angle: ', np.rad2deg(xhat[2,k])
            print 'd_target:', d_target

            _vk = pid_v.get_output(d_target, 0.0)
            _wk = pid_w.get_output(w_ang_nuevo[k], 0.0)
            
            lin_ang_vel[0,k] = _vk
            lin_ang_vel[1,k] = _wk
            
            _vlvr = model_lego.vw2vlvr(_vk, _wk)
         
            print '_vlvr: ',_vlvr
          
            omega = 1.0*(3.0*measurement - 4.0*measurement_prev_1 + measurement_prev_2) / (2.0*dt)
            print 'measurement: ',measurement
            print 'measurement_prev_1: ',measurement_prev_1
            print 'measurement_prev_2: ',measurement_prev_2
            print 'omega: ',omega

            uk[0,k] = pid_vel_l.get_output(_vlvr[0], omega[0])
            uk[1,k] = pid_vel_r.get_output(_vlvr[1], omega[1])
            
            
            w_target_prev = w_target

            measurement_prev_2[0] = measurement_prev_1[0]  
            measurement_prev_2[1] = measurement_prev_1[1]
            
            measurement_prev_1[0] = measurement[0]  
            measurement_prev_1[1] = measurement[1]
            print 'xhat: ',xhat[:,k]
            
            if d_target <= 0.05:
                
                omega=np.zeros((Ny,))
              
                print 'XHAT: ',xhat[:2,k]
                print 'X_TARGET: ',x_target[n]
                print 'OBJETIVO ' + str(n) + ' ALCANZADO.'
                
                n=n+1

                c_puntos,_ = x_target.shape
                if n == c_puntos:
                    nxt.send_motor_power(0,0)
                    break
                
            else:
                nxt.send_motor_power(uk[0,k],uk[1,k])

            print '-----------------------------------------------'
            
            
            
            
            
    nxt.send_motor_power(0,0)
    
    
#
plt_titles = ("$x [m]$","$y [m]$","$\psi [rad]$","v","w","$u_l$", "$u_r$")
Nx = 3
f, axarr = plt.subplots(Nx+Nu+Nu,1)
for i in xrange(Nx):
    axarr[i].plot(tsim[0:k], xhat[i, 0:k], "o-", color='r')
    axarr[i].set_title(plt_titles[i])
    axarr[i].grid()

for i in xrange(Nu):
    axarr[Nx+i].step(tsim[0:k],lin_ang_vel[i,0:k],"k.-", where='post')
    axarr[Nx+i].set_title(plt_titles[Nx+i])
    axarr[Nx+i].grid()
    
for i in xrange(Nu):
    axarr[Nx+Nu+i].step(tsim[0:k],uk[i,0:k],"k.-", where='post')
    axarr[Nx+Nu+i].set_title(plt_titles[Nx+2+i])
    axarr[Nx+Nu+i].grid()


plt.tight_layout()

plt.figure()
plt.plot(xhat[0,:],xhat[1,:])
plt.plot(point_list_x, point_list_y, 'ro')
plt.plot(xhat[0,0],xhat[1,0], 'ro')
plt.grid()
plt.show()
