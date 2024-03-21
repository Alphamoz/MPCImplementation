#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64, Float32, Float64, Int32
from gb_msgs.msg import Valeport_Altimeter
from cola2_msgs.msg import LinkquestDvl
from gb_msgs.msg import Spatial, Gui
from sensor_msgs.msg import NavSatFix
from auv_msgs.msg import NavSts, RPY
from bow_thruster.msg import bow_msg
from stern_thruster.msg import stern_msg
from surge_actuator.msg import surge_msg
try:
    from glider import Glider
except:
    print(Exception)

import math as mat
import time

class Gather:

    def __init__(self, isSim = False):
        # DVL #
        self.vellocity_earth_ = [0.0, 0.0, 0.0]
        self.vellocity_inst_ = [0.0, 0.0, 0.0]
        self.x_velo, self.y_velo, self.z_velo = 0.0, 0.0, 92.34
        self.u_velo, self.v_velo, self.w_velo = 0.0, 0.0, 0.0
        self.dvl_roll, self.dvl_pitch, self.dvl_yaw = 0.0, 0.0, 0.0
        self.altitude_ = 0.0
        self.cur_surge = 0.0

        self.depth_data = 0.0
        
        self.roll_deg, self.pitch_deg, self.yaw_deg = 0.0, 0.0, 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.lat_, self.long_ = 0.0, 0.0
        self.gps_status = 0
        
        self.ned_north = 0.0
        self.ned_east = 0.0
        self.ned_depth = 0.0
        self.uvw_u = 0.0
        self.uvw_v = 0.0
        self.uvw_w = 0.0
        
        # kompartemen 4 #
        self.volt_, self.cur_ = 0.0, 0.0
        self.humidity, self.temperature = 0.0, 0.0
        self.pressure, self.temperature2 = 0.0, 0.0

        # moving mass #
        # pitch #
        self.pitch_sp = 0
        self.mm_des, self.mm_actual = 0, 250
        # roll #
        self.sp_roll = 0.0
        self.servo_roll_position = 2450

        # buoyancy engine #
        self.desired_internal_volume = 0.0
        self.internal_volume = 350
        self.pressure_be = 0.0
        self.depth_rate = 0.0
        self.depth_sp = 0.0

        #  yaw #
        self.sp_yaw = 0.0
        self.bow_percentage = 0.0
        self.stern_percentage = 0.0
        self.surge_percentage = 0.0
        self.surge_sp = 0.0
        self.netral_volume= 0.0

        self.Tnow = time.time()
        self.Tlast = time.time()
        self.isSim = isSim

        self.pub = rospy.Publisher("/gather_gui", Gui, queue_size=10)
        if(self.isSim):
            self.publisher_depth_rate = rospy.Publisher("/smooth_depth_rate", Float32, queue_size=10)

        rospy.Subscriber('/gb_navigation/DVL', LinkquestDvl, self.callback_dvl)
        rospy.Subscriber('/dvl_rpy', RPY, self.callback_dvl_rpy)
        rospy.Subscriber('/gb_navigation/altimeter',Valeport_Altimeter, self.callback_alti)
        rospy.Subscriber('/gb_navigation/spatial/Imu', Spatial, self.callback_imu)
        rospy.Subscriber('/gb_navigation/spatial/Gps',NavSatFix, self.callback_gps)

        # rospy.Subscriber('/gb_navigation/nav_sts',NavSts, self.callback_navigation)
        rospy.Subscriber('/PFNavigation',NavSts, self.callback_navigation)

        rospy.Subscriber('/voltage_reading',Float32, self.callback_voltage)
        rospy.Subscriber('/current_reading',Float32, self.callback_current)
        rospy.Subscriber('/humidity_reading',Float32, self.callback_humidity)
        rospy.Subscriber('/temperature_reading',Float32, self.callback_temperature)
        rospy.Subscriber('/pressure_reading',Float32, self.callback_pressure)
        rospy.Subscriber('/temperature2_reading',Float32, self.callback_temperature2)

        rospy.Subscriber('/pitch_sp', Float64, self.callback_pitch_sp)

        rospy.Subscriber('/pitch_actuator_setpoint', Float64, self.mm_desired)
        rospy.Subscriber('/mm_position', Float64, self.mm_cur)
        rospy.Subscriber('/roll_sp', Float64, self.sp_roll_update)
        rospy.Subscriber('/servo_roll_position', Int32, self.callback_servo_roll_position)

        rospy.Subscriber('/desired_internal_volume', Float32, self.callback_desired_internal_volume)
        rospy.Subscriber('/current_internal_volume', Float32, self.callback_internal_volume)
        rospy.Subscriber('/pressure_be', Float32, self.callback_pressure_be)
        rospy.Subscriber('/smooth_depth_rate',Float32, self.callback_depth_rate)
        rospy.Subscriber('/depth_sp', Float64, self.callback_depth_sp)
        
        rospy.Subscriber('/yaw_sp', Float64, self.sp_yaw_update)
        rospy.Subscriber('/bow_actuator_setpoint', bow_msg, self.callback_bow)
        rospy.Subscriber('/stern_actuator_setpoint', stern_msg, self.callback_stern)
        rospy.Subscriber('/surge_actuator_setpoint', surge_msg, self.callback_surge)
        rospy.Subscriber('/surge_sp', Float64, self.callback_surge_sp)
        rospy.Subscriber('/netral_volume', Float32, self.callback_netral_volume)


    def callback_dvl(self, data) :
        self.vellocity_earth_[0] = data.velocityEarth[0]
        self.vellocity_earth_[1] = data.velocityEarth[1]
        self.vellocity_earth_[2] = data.velocityEarth[2]
        self.vellocity_inst_[0] = data.velocityInst[0]
        self.vellocity_inst_[1] = data.velocityInst[1]
        self.vellocity_inst_[2] = data.velocityInst[2]
        self.x_velo = -self.vellocity_earth_[0]
        self.y_velo = -self.vellocity_earth_[1]
        self.z_velo = -self.vellocity_earth_[2]
        self.u_velo = -self.vellocity_inst_[0]
        self.v_velo = -self.vellocity_inst_[1]
        self.w_velo =  self.vellocity_inst_[2]
        self.altitude_ = data.altitude
        self.cur_surge = mat.sqrt(pow((self.x_velo),2) + pow((self.y_velo),2))

    def callback_dvl_rpy(self, data) :
        self.dvl_roll = data.roll
        self.dvl_pitch = data.pitch
        self.dvl_yaw = data.yaw

    def callback_alti(self, data_alti) :
        self.depth_data = data_alti.depth

    def callback_imu(self, data_imu) :
        self.roll_deg = (data_imu.RPY.roll)*(180/3.14)
        self.pitch_deg= (data_imu.RPY.pitch)*(180/3.14)
        self.yaw_deg = (data_imu.RPY.yaw)*(180/3.14)
        self.roll_rate = (data_imu.Imu.angular_velocity.x)*(180/3.14)
        self.pitch_rate = (data_imu.Imu.angular_velocity.y)*(180/3.14)
        self.yaw_rate = (data_imu.Imu.angular_velocity.z)*(180/3.14)

    def callback_gps(self, data_gps) :
        self.lat_ = data_gps.latitude
        self.long_ = data_gps.longitude
        self.gps_status = data_gps.status.status

    def callback_navigation(self, data_navigasi) :
        self.ned_north = data_navigasi.position.north
        self.ned_east = data_navigasi.position.east
        self.ned_depth = data_navigasi.position.depth
        self.uvw_u = data_navigasi.body_velocity.x
        self.uvw_v = data_navigasi.body_velocity.y
        self.uvw_w = data_navigasi.body_velocity.z

    def callback_voltage(self, data_voltage) :
        self.volt_ = data_voltage.data

    def callback_current(self, data_current) :
        self.cur_ = data_current.data

    def callback_humidity(self, data) :
        self.humidity = data.data

    def callback_temperature(self, data) :
        self.temperature = data.data

    def callback_pressure(self, data) :
        self.pressure = data.data

    def callback_temperature2(self, data) :
        self.temperature2 = data.data

    def callback_pitch_sp(self, data) :
        self.pitch_sp = data.data

    def mm_desired(self, data_des_mm) :
        self.mm_des = data_des_mm.data

    def mm_cur(self, data_cur_mm) :
        self.mm_actual = data_cur_mm.data

    def sp_roll_update(self, data_sp_roll) :
        self.sp_roll = data_sp_roll.data

    def callback_servo_roll_position(self, data) :
        self.servo_roll_position = data.data

    def callback_desired_internal_volume(self, data) :
        self.desired_internal_volume = data.data

    def callback_internal_volume(self, data) :
        self.internal_volume = data.data

    def callback_pressure_be(self, data) :
        self.pressure_be = data.data

    def callback_depth_rate(self, data) :
        self.depth_rate = data.data

    def callback_depth_sp(self, data) :
        self.depth_sp = data.data

    def sp_yaw_update(self, data_sp_yaw) :
        self.sp_yaw = data_sp_yaw.data

    def callback_bow(self, data) :
        if(data.set > 1) :
            self.bow_percentage = 1
        elif(data.set < 0) :
            self.bow_percentage = 0
        else :
            self.bow_percentage = data.set

        if(data.dir != 1) :
            self.bow_percentage = -self.bow_percentage

    def callback_stern(self, data) :
        if(data.set > 1) :
            self.stern_percentage = 1
        elif(data.set < 0) :
            self.stern_percentage = 0
        else :
            self.stern_percentage = data.set

        if(data.dir != 1) :
            self.stern_percentage = -self.stern_percentage

    def callback_surge(self, data) :
        if(data.set > 1) :
            self.surge_percentage = 1
        elif(data.set < 0) :
            self.surge_percentage = 0
        else :
            self.surge_percentage = data.set

    def callback_surge_sp(self, data) :
        self.surge_sp = data.data
        
    def callback_netral_volume(self, data):
        self.netral_volume = data.data

    def publishData(self) :
        gui = Gui()
        gui.velocity_x = self.x_velo
        gui.velocity_y = self.y_velo
        gui.velocity_z = self.z_velo
        gui.velocity_u = self.u_velo
        gui.velocity_v = self.v_velo
        gui.velocity_w = self.w_velo
        gui.dvl_roll = self.dvl_roll
        gui.dvl_pitch = self.dvl_pitch
        gui.dvl_yaw = self.dvl_yaw
        gui.altitude = self.altitude_
        gui.surge = self.cur_surge

        gui.depth_data = self.depth_data
        
        gui.roll_data = self.roll_deg
        gui.pitch_data = self.pitch_deg
        gui.yaw_data = self.yaw_deg
        gui.roll_rate_data = self.roll_rate
        gui.pitch_rate_data = self.pitch_rate
        gui.yaw_rate_data = self.yaw_rate
        
        gui.lat = self.lat_
        gui.long = self.long_
        gui.gps_status = self.gps_status
        
        gui.navigasi_north = self.ned_north
        gui.navigasi_east = self.ned_east
        gui.navigasi_depth = self.ned_depth
        gui.navigasi_u = self.uvw_u
        gui.navigasi_v = self.uvw_v
        gui.navigasi_w = self.uvw_w
        
        gui.voltage = self.volt_
        gui.current = self.cur_
        gui.humidity = self.humidity
        gui.temperature = self.temperature
        gui.pressure = self.pressure
        gui.temperature2 = self.temperature2
        
        gui.pitch_sp = self.pitch_sp
        gui.des_mm = self.mm_des
        gui.cur_mm = self.mm_actual
        gui.roll_sp = self.sp_roll
        gui.servo_position = self.servo_roll_position
        
        gui.desired_internal_volume = self.desired_internal_volume
        gui.internal_volume = self.internal_volume
        gui.pressure_be = self.pressure_be
        gui.depth_rate = self.depth_rate
        gui.depth_sp = self.depth_sp
        gui.netral_volume = self.netral_volume 
        
        gui.yaw_sp = self.sp_yaw
        gui.bow_percentage = self.bow_percentage
        gui.stern_percentage = self.stern_percentage#belum
        gui.surge_percentage = self.surge_percentage#belum
        gui.surge_sp = self.surge_sp
        

        self.pub.publish(gui)


    def looping(self) :
        # rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.Tnow = time.time()
            if self.Tnow - self.Tlast > .1 :
                if(self.isSim):
                    print("Current BE internal Volume:", self.internal_volume)
                    self.depth_data, _, depth_rate = glider.depthMechanism(self.internal_volume)
                    dr = Float32()
                    dr.data = depth_rate
                    self.publisher_depth_rate.publish(dr)
                    self.pitch_deg = glider.pitchMechanism(self.mm_actual)
                print(self.Tnow)
                self.Tlast = self.Tnow
                self.publishData()
                #time.sleep(0.7)
                #rate.sleep()



if __name__ == '__main__':
    rospy.init_node('gather_node')
    glider=Glider(timesampling=0.1)
    g = Gather(True)
    g.looping()
    '''
    try :
        g = Gather()
        g.looping()
    except :
        print("program is closed")
    '''
