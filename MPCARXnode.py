#!/usr/bin/env python

from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt
import pickle
import rospy
from std_msgs.msg import Int64, Float32, Float64, Int32
from gb_msgs.msg import Gui, MPCsim
import math
# import time as t

with open("p_br_pitch.pickle", "rb") as file:
    p_pitch = pickle.load(file)

with open("p_br_dr.pickle", "rb") as file:
    p_dr = pickle.load(file)
    
class VeloGenerator():
    def __init__(self, maxVeloDes=0.1, arctan=20):
        # in meters
        self.maxVeloDes = maxVeloDes
        self.minVeloDes = 0
        # error 0.2 meters
        self.minError = 0.2
        self.maxError = (arctan*maxVeloDes)-self.minVeloDes
        self.minmaxErrorDiff = self.maxError-self.minError
        self.errorDepth = 0
    def calculate_desVelo(self, errorDepth):
        if (abs(errorDepth) > self.maxError):
            veloDes = self.maxVeloDes * (errorDepth)/abs(errorDepth)
            # print("Velo menuju {}".format(veloDes))
        elif (abs(errorDepth) <= self.minError):
            veloDes = 0
            # print("Menuju nol velonya")
        else:
            veloDes = self.maxVeloDes/self.minmaxErrorDiff * \
                (errorDepth)-((self.minError*self.maxVeloDes/self.minmaxErrorDiff)
                              * (errorDepth)/abs(errorDepth))
            # print("Transient velo")
        # print(errorDepth, veloDes)
        # veloDes in metres
        return veloDes
    
class MPC:
    def __init__(self):
        # current states
        self.publisher_be = rospy.Publisher("/current_internal_volume", Float32, queue_size=10)
        self.publisher_mm = rospy.Publisher("/mm_position", Float64, queue_size=10)
        # self.publisher_be = rospy.Publisher("/desired_internal_volume", Float32, queue_size=10)
        # self.publisher_mm = rospy.Publisher("/pitch_actuator_setpoint", Float64, queue_size=10)

        # publisher for model error
        self.publisher_MPC = rospy.Publisher("/MPCSim", MPCsim, queue_size=10)
        self.status_pitch=0
        self.status_depth=0
        self.current_depth_rate = 0
        self.current_depth = 0
        self.current_pitch = 0
        self.current_be = 350
        self.current_mm = 250
        # maximum depth rate on m/s
        self.max_depth_rate = 0.1
        self.veloGenerator = VeloGenerator(self.max_depth_rate,10)
        self.depth_sp = 0
        self.pitch_sp = 0
        self.depth_rate_sp = 0
        # frequency
        self.frequency = 1 
        rospy.Subscriber("/gather_gui", Gui, self.update_current_state)
        rospy.Subscriber("/pitch_sp", Float64, self.update_pitch_sp)
        rospy.Subscriber("/depth_sp", Float64, self.update_depth_sp)
        rospy.Subscriber('/pitch_control_status', Int32, self.update_status_pitch)
        rospy.Subscriber('/depth_control_status', Int32, self.update_status_depth)
        rospy.Subscriber("/smooth_depth_rate", Float32, self.update_depth_rate)
        self.initialization()
        with open('output_data.txt', 'w') as output:
            output.write('solving_time\n')   
    
    # mpc initialization
    def initialization(self):
        # mpc parameters
        self.control_horizon = 2
        self.prediction_horizon = 15
        # building mpc
        self.m = GEKKO(remote=False)
        self.p = {
            "a": np.array([[p_dr["a"][0][0], p_pitch["a"][0][0]]]),
            "b": np.array(
                [
                    [[p_dr["b"][0][0][0], 0], [p_dr["b"][0][1][0], 0]],
                    [[p_pitch["b"][0][0][0], p_pitch["b"][0][0][1]], [0, 0]],
                ]
            ),
            "c": np.array([0, 0]),
        }
        # initialize variables and manipulated variables
        self.MM = self.m.Var(value=0)
        self.mr = self.m.MV(value=0, lb=-10, ub=10)
        self.mr.STATUS = 1

        self.BE = self.m.Var(value=0)
        self.br = self.m.MV(value=0, lb=-15, ub=15)
        self.br.STATUS = 1

        # initialize the controlled variables
        self.pitch_angle = self.m.CV(value=0)
        self.pitch_angle.FSTATUS = 1
        self.depth = self.m.Var(value=0)
        self.depth_rate = self.m.CV(value=0)
        self.depth_rate.FSTATUS = 1

        self.m.Equation(self.depth == self.m.integral(self.depth_rate/1000))
        self.m.Equation(self.BE == self.m.integral(self.br))
        self.m.Equation(self.MM == self.m.integral(self.mr))
        
        # objective functions for tracking error
        self.m.Minimize(1 * ((self.depth_rate - self.depth_rate_sp)) ** 2)
        # self.m.Minimize(1 * ((self.depth - self.depth_sp)) ** 2)
        self.m.Minimize(1 * ((self.pitch_angle - self.pitch_sp)) ** 2)
        # objective functions for input changes
        self.m.Minimize(0.01*((self.br)**2)+ 0.01*((self.mr)**2))
        self.m.arx(self.p, y=[self.depth_rate, self.pitch_angle], u=[self.br, self.mr])
        self.m.time = np.arange(0, self.prediction_horizon, 1)
        self.m.options.IMODE = 6  # MPC mode control
        self.m.options.MAX_ITER = 100  # Maximum number of iterations
        self.m.options.SOLVER = 3  # IPOPT
        # self.m.options.CTRL_HOR = self.control_horizon
        # self.m.options.PRED_HOR = self.prediction_horizon

    # update state on subscriber callback
    def update_current_state(self, data):
        self.current_be=data.internal_volume
        self.current_mm=data.cur_mm
        self.current_depth_rate = data.depth_rate
        self.current_pitch = data.pitch_data
        self.current_depth = data.depth_data
        
    # update setpoint from subscriber
    def update_pitch_sp(self, data):
        self.pitch_sp = data.data
        
    def update_depth_sp(self, data):
        self.depth_sp = data.data
    
    # update control status
    def update_status_pitch(self, data):
        self.status_pitch = data.data
    def update_status_depth(self, data):
        self.status_depth =data.data
    def update_depth_rate(self,data):
        self.current_depth_rate = data.data

    def publish_my_data(self):
        index = 5
        my_data = MPCsim()
        my_data.depth_sp=self.depth_sp
        my_data.pitch_sp=self.pitch_sp
        my_data.depth_rate_sp=self.depth_rate_sp
        my_data.estimated_depth=0
        my_data.estimated_depth_rate = self.depth_rate.VALUE[index]
        my_data.estimated_pitch = self.pitch_angle.VALUE[index]
        my_data.solvingtime = self.m.options.SOLVETIME
        my_data.br = self.br.NEWVAL
        my_data.mr = self.mr.NEWVAL
        self.publisher_MPC.publish(my_data)
    def initSimProperty(self):
        self.time=1
        self.opt_moving_mass = np.array([])
        self.opt_buoyancy_engine_rate = np.array([])
        self.depth_sim = np.array([])
        self.depth_rate_sim = np.array([])
        self.setpointdepth_rate_sim = np.array([])
        self.setpointdepth_sim = np.array([])
        self.setpointpitch_angle_sim = np.array([])
        self.pitch_angle_sim = np.array([])
        self.BE_input = np.array([])
        self.MM_input = np.array([])
        # print(BE_input.size)
        self.totalInputChangeListBE = np.array([])
        self.totalInputChangeListMM = np.array([])
        plt.figure(figsize=(6, 8))
    def plot(self):
        time = np.arange(0, self.time, 1)
        self.opt_moving_mass=np.append(self.opt_moving_mass, self.mr.NEWVAL)
        self.opt_buoyancy_engine_rate=np.append(self.opt_buoyancy_engine_rate,self.br.NEWVAL)
        self.depth_sim=np.append(self.depth_sim,self.current_depth)
        self.pitch_angle_sim=np.append(self.pitch_angle_sim, self.current_pitch)
        self.depth_rate_sim=np.append(self.depth_rate_sim, self.current_depth_rate)
        self.setpointdepth_rate_sim=np.append(self.setpointdepth_rate_sim, self.depth_rate_sp)
        self.setpointdepth_sim=np.append(self.setpointdepth_sim, self.depth_sp)
        self.setpointpitch_angle_sim=np.append(self.setpointpitch_angle_sim,self.pitch_sp)
        plt.clf()
        # plt.figsize()
        plt.subplot(4, 1, 1)
        plt.title("MPC Control Gliding Simulation {}m/s Depth Rate".format(self.max_depth_rate))
        print(self.opt_moving_mass)
        plt.plot(time[0:self.time], self.opt_moving_mass[0:self.time],
                label='Moving Mass (Scaled by 20)')
        plt.plot(time[0:self.time], self.opt_buoyancy_engine_rate[0:self.time], label='Buoyancy Engine')
        plt.ylabel('Input')
        plt.legend()
        print("ini depth spdepth bos", self.setpointdepth_sim[0:self.time], time[0:self.time], self.setpointdepth_rate_sim[0:self.time])

        plt.subplot(4, 1, 2)
        plt.plot(time[0:self.time], self.depth_sim[0:self.time], label='Depth')
        plt.plot(time[0:self.time],self.setpointdepth_sim[0:self.time], color='r', linestyle='--', label='Setpoint Depth')
        plt.ylabel('Depth')
        plt.legend()
        # print("ini depth spdepth bos", setpointdepth_sim)

        plt.subplot(4, 1, 3)
        plt.plot(time[0:self.time], self.pitch_angle_sim[0:self.time], label='Pitch Angle')
        plt.plot(time[0:self.time],self.setpointpitch_angle_sim[0:self.time], color='r',
                    linestyle='--', label='Setpoint Pitch')
        # plt.xlabel('Time (s)')
        plt.ylabel('Pitch Angle')
        plt.legend()

        plt.subplot(4, 1, 4)  # Add a new subplot for depth rate
        plt.plot(time[0:self.time], self.depth_rate_sim[0:self.time], label='Depth Rate')
        plt.plot(time[0:self.time], self.setpointdepth_rate_sim[0:self.time], color='r',
                linestyle='--', label='Setpoint Depth Rate')
        plt.xlabel('Time (s)')
        plt.ylabel('Depth Rate')
        plt.legend()

        plt.draw()
        plt.pause(0.05)
        self.time+=1

    # main program
    def mainProgram(self):
        # if not controlled
        # if (self.status_depth != 1 and self.status_pitch != 1):
        #     print("Control is not activated")
        #     return
        self.error_depth = self.depth_sp-self.current_depth

        # updating depth_rate_setpoint
        self.depth_rate_sp = self.veloGenerator.calculate_desVelo(self.error_depth)

        try:
            self.m.solve(disp=True)
            timeshift = 5
            self.publish_my_data()
            with open('output_data.txt', 'a') as output:
                output.write('{}\n'.format(self.m.options.SOLVETIME))
        except:
            print("Solution not found")
            
        # getting optimized input on variables
        opt_moving_mass = self.mr.NEWVAL
        opt_buoyancy_engine_rate = self.br.NEWVAL
        
        # updating the depth value and pitch value
        self.depth.VALUE = self.current_depth*1000
        self.pitch_angle.MEAS = self.current_pitch
        self.depth_rate.MEAS = self.current_depth_rate*1000
        print("Depth Rate Measurement: ", self.depth_rate.VALUE)
        
        # preparing and converting input signal to publish
        BE_input = Float32()
        MM_input = Float64()
        BE_input.data = opt_buoyancy_engine_rate + self.current_be
        print("Current BE:", self.current_be)
        MM_input.data = opt_moving_mass + self.current_mm
        print("Current MM:", self.current_mm)

        # batasan input
        BE_input.data = max(min(BE_input.data,700),0)
        MM_input.data = max(min(MM_input.data,500),0)

        if (self.status_depth == 1):
            print("Control Depth Active")
            self.publisher_be.publish(BE_input)
            print(BE_input)

        if (self.status_pitch == 1):
            print("Control Pitch Active")
            self.publisher_mm.publish(MM_input)
            print(MM_input)
        
        if(self.status_depth == 0):
            print("Control Depth not Active!")

        if(self.status_pitch == 0):
            print("Control Pitch not Active!")

        self.R1=0.01
        self.depth_rate_error=self.current_depth_rate - self.depth_rate_sp
        if(abs(self.depth_rate_error)<0.03):
            self.R1=1
        if(abs(self.depth_rate_error)<0.02):
            self.R1=10
        if(abs(self.depth_rate_error)<0.01):
            self.R1=50
        self.R2=0.1
        self.pitch_rate_error = self.current_depth_rate - self.depth_rate_sp
        if(abs(self.pitch_rate_error)<3):
            self.R2=10
        print((self.current_pitch - self.pitch_sp))
        
        # resetting objectives for MPC
        self.m._objectives.clear()
        self.m.Minimize(1*((self.depth_rate - (self.depth_rate_sp*1000)))**2)
        self.m.Minimize(1*((self.pitch_angle - self.pitch_sp))**2)
        self.m.Minimize(self.R1*((self.br/15)**2)+ self.R2*((self.mr/10)**2))
        
        if(self.status_pitch or self.status_depth ==1):
            self.plot()
        
if __name__ == "__main__":
    print("MPC Node Started")
    rospy.init_node("MPCARXnode")
    # instantiate MPC class
    mpc_instance = MPC()
    mpc_instance.initSimProperty()
    # main loop every 1 second
    rate = rospy.Rate(mpc_instance.frequency)
    # main loop
    while not rospy.is_shutdown():
        mpc_instance.mainProgram()
        rate.sleep()
    print("MPC Node Closed")