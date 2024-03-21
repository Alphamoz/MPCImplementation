#!/usr/bin/env python

# controlling pitch using mm #

import rospy
from gb_msgs.msg import Spatial
import time
from std_msgs.msg import Float64, Int32

class ControlMM :
    def __init__(self):
        self.pitch_deg = 0.0
        rospy.Subscriber('/mm_position', Float64, self.callback_mm)
        rospy.Subscriber('/gb_navigation/spatial/Imu', Spatial, self.callback)
        self.pub_des_mm = rospy.Publisher("pitch_actuator_setpoint", Float64, queue_size=10)
        rospy.Subscriber('/pitch_sp', Float64, self.sp_update)
        rospy.Subscriber('/pitch_control_status', Int32, self.update_status_control)


        # Variable control PID for getting the neutral pitch 
        self.kp_p = 0.08
        self.kd_p = 0.0
        self.ki_p = 0.0
        self.curtime_p = 0
        self.pretime_p = 0
        self.elipstime_p = 0
        self.error_p= 0
        self.lasterror_p = 0
        self.sp_p = 0.0
        self.cumerror_p = 0
        self.rateerror_p= 0
        self.delta_error_p= 0
        self.Ts_p = 0.025      
        self.pitch_deg = 0.0 
	
	self.netral = 10.0

	self.out_p = 0.0
	self.updatepos = 0.0
        try:
            file = open("/home/ubuntu/catkin_ws/src/learning_joy/memory/curPos.txt", "r")
            self.curpos = int(file.read())  # CurrrentPosition # ReadFromFile
            file.close()
            print("Ini Cur Pos")
            print(self.curpos)
        except:
            self.curpos = 0

        try:
            file = open("/home/ubuntu/catkin_ws/src/learning_joy/memory/NPos.txt", "r")
            self.NPos = int(file.read())  # Banyak Posisi # ReadFromFile
            file.close()
            print("Ini N Pos")
            print(self.NPos)
        except:
            self.NPos = 2000

        #self.curpos = 0.0
        self.status_ = 0

    def update_status_control(self, status_data) :
        self.status_ = status_data.data

    def sp_update(self, data_sp) :
        self.sp_p = data_sp.data

    def computePID(self) :
        # Discrete PID #
        self.curtime_p = time.time()
        self.elipstime_p = self.curtime_p - self.pretime_p

        if self.elipstime_p >= self.Ts_p :
            self.error_p = self.sp_p - self.pitch_deg
            self.delta_error_p = self.error_p - self.lasterror_p
            self.cumerror_p += self.error_p
            # self.cumerror += self.error * self.elipstime
            #self.rateerror = self.delta_error/self.elipstime

            self.out_p = self.kp_p * self.error_p + (self.ki_p* self.Ts_p) * self.cumerror_p + (self.kd_p/self.Ts_p) * self.delta_error_p
            
            self.lasterror_p = self.error_p
            self.pretime_p = self.curtime_p



    def callback_mm(self, data_mm) :
	self.current_pos_mm = data_mm.data


    def update_pos_mass(self) :
        self.updatepos = self.curpos + self.out_p
        self.updatepos = round(self.updatepos,0)
        
        if self.updatepos < 0:
            self.updatepos = 0.0
        elif self.updatepos >= self.NPos:
            self.updatepos = self.NPos

	
    def callback(self, data) :
        #self.roll_deg  = (data.RPY.roll)*(180/3.14)
        self.pitch_deg = (data.RPY.pitch)*(180/3.14)

    def callback_mm(self, data1) :
        #self.roll_deg  = (data.RPY.roll)*(180/3.14)
        self.curpos = data1.data

    def publish_mm(self) :
        dataPitch = Float64()
        dataPitch.data = self.updatepos
        self.pub_des_mm.publish(dataPitch)


    def loop(self) :
        if self.status_ == 1 :
            self.computePID()
            self.update_pos_mass()
	    self.publish_mm()
            time.sleep(.1)
            print('Hasil PID : ', self.updatepos)
            print('Hasil error PID : ', self.error_p)
            print('Nilai sp pitch : ', self.sp_p)
        else :
            self.updatepos = self.curpos
            print(self.updatepos)
            #self.publish_mm()
            #time.sleep(2)
            #print("status control pitch is not active")
            #print('Nilai sp pitch : ', self.sp_p)

if __name__ == '__main__':
    try: 
        rospy.init_node('ControlLinearMM')
        MovMass = ControlMM()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            MovMass.loop()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

