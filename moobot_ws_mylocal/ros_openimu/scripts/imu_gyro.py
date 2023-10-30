#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Twist
from scipy.integrate import cumtrapz
import time
import numpy as np
from scipy import signal
from scipy.optimize import curve_fit

start_bool = True 


def get_accel(data):
    global array_data,ax,ay,az
    ax=data.linear_acceleration.x
    ay=data.linear_acceleration.y
    az=data.linear_acceleration.z
    array_data = np.array([ax,ay,az])

    

def imu_integrator():
    global array_data,ax,ay,az
    data_indx = 1 # index of variable to integrate
    dt_stop = 0.01 # seconds to record and integrate

    break_bool = False

    accel_array,t_array = [],[]
    print("Starting Data Acquisition")
    t0 = time.time()
    loop_bool = False
    while True:
        try:
            t_array.append(time.time()-t0)
            data_array = [ax,ay,az]
            accel_array.append(accel_fit(data_array[data_indx],
                                            *accel_coeffs[data_indx]))
            if not loop_bool:
                loop_bool = True
                print("Start Moving IMU...")
        except:
            continue
        if time.time()-t0>dt_stop:
            print("Data Acquisition Stopped")
            break
            
        if break_bool:
            break

        # Fs_approx = len(accel_array)/dt_stop
        # b_filt,a_filt = signal.butter(4,5,'low',fs=Fs_approx)
        # accel_array = signal.filtfilt(b_filt,a_filt,accel_array)
        # accel_array = np.multiply(accel_array,9.80665)
        #
        ##################################
        # Print Sample Rate and Accel
        # Integration Value
        ##################################
        #
        # print("Sample Rate: {0:2.0f}Hz".format(len(accel_array)/dt_stop))
        veloc_array = np.append(0.0,cumtrapz(accel_array,x=t_array))
        dist_approx = np.trapz(veloc_array,x=t_array)
        dist_array = np.append(0.0,cumtrapz(veloc_array,x=t_array))
        # print(dist_array)
#
def accel_fit(x_input,m_x,b):
    return (m_x*x_input)+b # fit equation for accel calibration
#

def accel_cal():
    global ax,ay,az
    print("-"*50)
    print("Accelerometer Calibration")
    mpu_offsets = [[],[],[]] # offset array to be printed
    axis_vec = ['z','y','x'] # axis labels
    cal_directions = ["upward","downward","perpendicular to gravity"] # direction for IMU cal
    cal_indices = [2,1,0] # axis indices
    for qq,ax_qq in enumerate(axis_vec):
        ax_offsets = [[],[],[]]
        print("-"*50)
        for direc_ii,direc in enumerate(cal_directions):
            input("-"*8+" Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -"+\
              ax_qq+"-axis pointed "+direc)
            [array_data for ii in range(0,cal_size)] # clear buffer between readings
            mpu_array = []
            while len(mpu_array)<cal_size:
                try:
                    mpu_array.append([ax,ay,az]) # append to array
                except:
                    continue
            ax_offsets[direc_ii] = np.array(mpu_array)[:,cal_indices[qq]] # offsets for direction

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        popts,_ = curve_fit(accel_fit,np.append(np.append(ax_offsets[0],
                                 ax_offsets[1]),ax_offsets[2]),
                   np.append(np.append(1.0*np.ones(np.shape(ax_offsets[0])),
                    -1.0*np.ones(np.shape(ax_offsets[1]))),
                        0.0*np.ones(np.shape(ax_offsets[2]))),
                            maxfev=10000)
        mpu_offsets[cal_indices[qq]] = popts # place slope and intercept in offset array
    print('Accelerometer Calibrations Complete')
    time.sleep(60)
    return mpu_offsets

if __name__ == '__main__':
    rospy.init_node('imu_accuracy_tester')
    linear_acceleration = Vector3()
    angular_velocity = Vector3()
    position = Vector3()
    orientation = Vector3()
    previous_time = rospy.get_time()

    integ1_array_x=0
    integ1_array_y=0
    integ1_array_z=0 

    integ1_array =[]

    array_data=[]
    ax = 0
    ay = 0
    az = 0
    dif=0
    cal_size = 500  # points to use for calibration
    accel_coeffs = [np.array([ 0.9978107 , -0.19471673]),
                            np.array([ 0.99740193, -0.56129248]),
                            np.array([0.9893495 , 0.20589886])]
    
    rospy.Subscriber('/imu_acc_ar', Imu,  get_accel)
    orientation_pub = rospy.Publisher('/imu/position', Vector3, queue_size=1)
    
    imu_integrator()
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()
