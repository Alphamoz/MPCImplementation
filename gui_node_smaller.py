#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64, Float32, Float64, Int32, String
from gb_msgs.msg import Valeport_Altimeter
from cola2_msgs.msg import LinkquestDvl
from gb_msgs.msg import Spatial, Gui
from sensor_msgs.msg import NavSatFix
import math as mat
import signal

from PyQt5 import QtTest
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut
from PyQt5.QtCore import Qt
import sys
import os
basedir = os.path.dirname(os.path.abspath(__file__))
PATH = basedir + '/Gambar_GUI'
sys.path.insert(0, PATH)
# import source_rc
from PyQt5.QtWidgets import QMainWindow

class Ui_Form(object):

    def __init__(self) :

        self.gui1 = 1.0 # ndot_dvl
        self.gui2 = 2.0 # edot_dvl
        self.gui3 = 3.0 # ddot_dvl
        self.gui4 = 4.0 # depth
        self.gui5 = 5.0 # roll_imu
        self.gui6 = 6.0 # pitch_imu
        self.gui7 = 7.0 # yaw_imu
        self.gui8 = 8.0 # latitude
        self.gui9 = 9.0 # longitude
        self.gui10 = 10.0 # resultan_velocity_sp
        self.gui11 = 11.0 # yaw_rate
        self.gui12 = 12.0 # pitch_rate
        self.gui13 = 13.0 # heading_from_velocity
        self.gui14 = 14.0 # voltage
        self.gui15 = 15.0 # current
        self.gui16 = 16  # roll_rate
        self.gui17 = 17.0 # internal_volume
        self.gui18 = 18.0 # x_navigation
        self.gui19 = 19.0 # y_navigation
        self.gui20 = 20.0 # z_navigation
        self.gui21 = 21.0 # depth_rate
        self.gui22 = 22 # status_gps
        self.gui23 = 23.0 # humidity
        self.gui24 = 24.0 # temperature
        self.gui25 = 25.0 # pressure
        self.gui26 = 26.0 # u_dvl
        self.gui27 = 27.0 # v_dvl
        self.gui28 = 28.0 # w_dvl
        self.gui29 = 29.0 # u_nav
        self.gui30 = 30.0 # v_nav
        self.gui31 = 31.0 # w_nav
        self.gui32 = 32.0 # depth_sp
        self.gui33 = 33.0 # altimeter
        self.gui34 = 34.0 # roll_dvl
        self.gui35 = 35.0 # pitch_dvl
        self.gui36 = 36.0 # yaw_dvl
        self.gui37 = 37.0 # roll_sp
        self.gui38 = 38.0 # pitch_sp
        self.gui39 = 39.0 # yaw_sp
        self.gui40 = 40.0 # servo_position
        self.gui41 = 41.0 # resultan_velocity
        self.gui42 = 42.0 # main_thruster
        self.gui43 = 43.0 # mm_position
        self.gui44 = 44.0 # des_mm_position
        self.gui45 = 45.0 # bow_act
        self.gui46 = 46.0 # stern_act
        self.gui47 = 47.0 # temperature2
        self.gui48 = 48.0 # desired_internal_volume
        self.gui49 = 49.0 # middle value BE

        self.status_roll = 0
        self.status_pitch = 0
        self.status_yaw = 0
        self.status_surge = 0
        self.status_depth = 0
        self.on_oas = 0
        self.flag_reset = 0
        self.logger_sts = 0

        rospy.Subscriber('/gather_gui', Gui, self.callback_print)
        
        self.pub_sp_roll = rospy.Publisher('/roll_sp', Float64, queue_size=10)
        self.pub_sp_pitch = rospy.Publisher('/pitch_sp', Float64, queue_size=10)
        self.pub_sp_yaw = rospy.Publisher('/yaw_sp', Float64, queue_size=10)
        self.pub_sp_surge = rospy.Publisher('/surge_sp', Float64, queue_size=10)
        self.pub_sp_depth = rospy.Publisher('/depth_sp', Float64, queue_size=10)

        self.pub_roll_status = rospy.Publisher('/roll_control_status', Int32, queue_size=10)
        self.pub_pitch_status = rospy.Publisher('/pitch_control_status', Int32, queue_size=10)
        self.pub_yaw_status = rospy.Publisher('/yaw_control_status', Int32, queue_size=10)
        self.pub_surge_status = rospy.Publisher('/surge_control_status', Int32, queue_size=10)       
        self.pub_depth_status = rospy.Publisher('/depth_control_status', Int32, queue_size=10)
        self.pub_logger_status = rospy.Publisher('/logger_status', Int32, queue_size=10)
        self.pub_oas_status = rospy.Publisher('/oas_status', Int32, queue_size=10)
        self.pub_oas_notif_test = rospy.Publisher('/oas_notif_test', String, queue_size=10)
        self.pub_reset_nav_reference = rospy.Publisher('/update_reference', Int32, queue_size=10)

        # Create A Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)

        # Start the timer and update every second
        self.timer.start(100)
        # self.shortcut_close = QShortcut(QKeySequence(tr("Ctrl+P"))
        # self.shortcut_close.activated.connect(self.signal_handler)
        

    def update(self):
        self.lcdNumber.display(self.gui1)
        self.lcdNumber_2.display(self.gui2) 
        self.lcdNumber_3.display(self.gui3)
        self.lcdNumber_4.display(self.gui4)
        self.lcdNumber_5.display(self.gui5) 
        self.lcdNumber_6.display(self.gui6) 
        self.lcdNumber_7.display(self.gui7)
        self.lcdNumber_8.display(self.gui8)
        self.lcdNumber_9.display(self.gui9)
        self.lcdNumber_10.display(self.gui10)
        self.lcdNumber_11.display(self.gui11)
        self.lcdNumber_12.display(self.gui12)
        self.lcdNumber_13.display(self.gui13)
        self.lcdNumber_14.display(self.gui14)
        self.lcdNumber_15.display(self.gui15)
        self.lcdNumber_16.display(self.gui16)
        self.lcdNumber_17.display(self.gui17)
        self.lcdNumber_18.display(self.gui18)
        self.lcdNumber_19.display(self.gui19)
        self.lcdNumber_20.display(self.gui20)
        self.lcdNumber_21.display(self.gui21)
        self.lcdNumber_22.display(self.gui22)
        self.lcdNumber_23.display(self.gui23)
        self.lcdNumber_24.display(self.gui24)
        self.lcdNumber_25.display(self.gui25)
        self.lcdNumber_26.display(self.gui26)
        self.lcdNumber_27.display(self.gui27)
        self.lcdNumber_28.display(self.gui28)
        self.lcdNumber_29.display(self.gui29)
        self.lcdNumber_30.display(self.gui30)
        self.lcdNumber_31.display(self.gui31)
        self.lcdNumber_32.display(self.gui32)
        self.lcdNumber_33.display(self.gui33)
        self.lcdNumber_34.display(self.gui34)
        self.lcdNumber_35.display(self.gui35)
        self.lcdNumber_36.display(self.gui36)
        self.lcdNumber_37.display(self.gui37)
        self.lcdNumber_38.display(self.gui38)
        self.lcdNumber_39.display(self.gui39)
        self.lcdNumber_40.display(self.gui40)
        self.lcdNumber_41.display(self.gui41)
        self.lcdNumber_42.display(self.gui42)
        self.lcdNumber_43.display(self.gui43)
        self.lcdNumber_44.display(self.gui44)
        self.lcdNumber_45.display(self.gui45)
        self.lcdNumber_46.display(self.gui46)
        self.lcdNumber_47.display(self.gui47)
        self.lcdNumber_48.display(self.gui48)
        self.lcdNumber_49.display(self.gui49)

    def callback_print(self, data) :
        self.gui1 = round(data.velocity_x,2)
        self.gui2 = round(data.velocity_y,2) 
        self.gui3 = round(data.velocity_z,2)
        self.gui4 = round(data.depth_data,2)
        self.gui5 = round(data.roll_data,2) 
        self.gui6 = round(data.pitch_data,2) 
        self.gui7 = round(data.yaw_data,1)
        self.gui8 = round(data.lat,8)
        self.gui9 = round(data.long,8)
        self.gui10 = round(data.surge_sp,2)
        self.gui11 = round(data.yaw_rate_data,3)
        self.gui12 = round(data.pitch_rate_data,3)
        self.gui13 = mat.atan2(self.gui1, self.gui2) * 180/mat.pi
        if self.gui13 < 0 :
                self.gui13 += 360
        self.gui14 = round(data.voltage,2)
        self.gui15 = round(data.current,1)
        self.gui16 = round(data.roll_rate_data,3)
        self.gui17 = round(data.internal_volume,1)
        self.gui18 = round(data.navigasi_north,2)
        self.gui19 = round(data.navigasi_east,2)
        self.gui20 = round(data.navigasi_depth,2)
        self.gui21 = round(data.depth_rate,5)
        self.gui22 = data.gps_status
        self.gui23 = round(data.humidity,2)
        self.gui24 = round(data.temperature,2)
        self.gui25 = round(data.pressure,1)
        self.gui26 = round(data.velocity_u,2)
        self.gui27 = round(data.velocity_v,2) 
        self.gui28 = round(data.velocity_w,2)
        self.gui29 = round(data.navigasi_u,2)
        self.gui30 = round(data.navigasi_v,2) 
        self.gui31 = round(data.navigasi_w,2)
        self.gui32 = round(data.depth_sp,2)
        self.gui33 = round(data.altitude,2)
        self.gui34 = round(data.dvl_roll,2)
        self.gui35 = round(data.dvl_pitch,2)
        self.gui36 = round(data.dvl_yaw,1)
        self.gui37 = round(data.roll_sp,2)
        self.gui38 = round(data.pitch_sp,2)
        self.gui39 = round(data.yaw_sp,1)
        self.gui40 = data.servo_position
        self.gui41 = round(data.surge,2)
        self.gui42 = round(data.surge_percentage,3)
        self.gui43 = round(data.cur_mm,1)
        self.gui44 = round(data.des_mm,1)
        self.gui45 = round(data.bow_percentage,2)
        self.gui46 = round(data.stern_percentage,2)
        self.gui47 = round(data.temperature2,2)
        self.gui48 = round(data.desired_internal_volume,1)
        self.gui49 = round(data.netral_volume)


    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1366, 768)
        Form.setMaximumSize(QtCore.QSize(1366, 768))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        Form.setFont(font)
        Form.setStyleSheet("QWidget{\n"
"    background-color:\"#0D083B\"\n"
"}")
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(10, 20, 151, 61))
        self.label.setStyleSheet("image: url(:/newPrefix/auv-2.png);")
        self.label.setText("")
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(1170, 750, 185, 17))
        self.label_2.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(Form)
        self.label_3.setGeometry(QtCore.QRect(10, 100, 300, 40))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setStyleSheet("QLabel{\n"
"   color:\"white\"\n"
"}")
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(Form)
        self.label_4.setGeometry(QtCore.QRect(10, 160, 81, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(Form)
        self.label_5.setGeometry(QtCore.QRect(10, 200, 91, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(Form)
        self.label_6.setGeometry(QtCore.QRect(10, 240, 81, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_6.setObjectName("label_6")
        self.lcdNumber = QtWidgets.QLCDNumber(Form)
        self.lcdNumber.setGeometry(QtCore.QRect(110, 150, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber.setFont(font)
        self.lcdNumber.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber.setDigitCount(5)
        self.lcdNumber.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber.setObjectName("lcdNumber")
        self.lcdNumber_2 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_2.setGeometry(QtCore.QRect(190, 150, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_2.setFont(font)
        self.lcdNumber_2.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_2.setDigitCount(5)
        self.lcdNumber_2.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_2.setObjectName("lcdNumber_2")
        self.lcdNumber_3 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_3.setGeometry(QtCore.QRect(270, 150, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_3.setFont(font)
        self.lcdNumber_3.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_3.setDigitCount(5)
        self.lcdNumber_3.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_3.setObjectName("lcdNumber_3")
        self.lcdNumber_26 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_26.setGeometry(QtCore.QRect(110, 190, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_26.setFont(font)
        self.lcdNumber_26.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_26.setDigitCount(5)
        self.lcdNumber_26.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_26.setObjectName("lcdNumber_26")
        self.lcdNumber_27 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_27.setGeometry(QtCore.QRect(190, 190, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_27.setFont(font)
        self.lcdNumber_27.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_27.setDigitCount(5)
        self.lcdNumber_27.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_27.setObjectName("lcdNumber_27")
        self.lcdNumber_28 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_28.setGeometry(QtCore.QRect(270, 190, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_28.setFont(font)
        self.lcdNumber_28.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_28.setDigitCount(5)
        self.lcdNumber_28.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_28.setObjectName("lcdNumber_28")
        self.lcdNumber_29 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_29.setGeometry(QtCore.QRect(110, 230, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_29.setFont(font)
        self.lcdNumber_29.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_29.setDigitCount(5)
        self.lcdNumber_29.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_29.setObjectName("lcdNumber_29")
        self.lcdNumber_30 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_30.setGeometry(QtCore.QRect(190, 230, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_30.setFont(font)
        self.lcdNumber_30.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_30.setDigitCount(5)
        self.lcdNumber_30.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_30.setObjectName("lcdNumber_30")
        self.lcdNumber_31 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_31.setGeometry(QtCore.QRect(270, 230, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_31.setFont(font)
        self.lcdNumber_31.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_31.setDigitCount(5)
        self.lcdNumber_31.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_31.setObjectName("lcdNumber_31")
        self.label_7 = QtWidgets.QLabel(Form)
        self.label_7.setGeometry(QtCore.QRect(340, 160, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setStyleSheet("QLabel{\n"
"     color:\"white\"\n"
"}")
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(Form)
        self.label_8.setGeometry(QtCore.QRect(340, 200, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(Form)
        self.label_9.setGeometry(QtCore.QRect(340, 240, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_9.setFont(font)
        self.label_9.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(Form)
        self.label_10.setGeometry(QtCore.QRect(50, 290, 300, 31))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_10.setFont(font)
        self.label_10.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(Form)
        self.label_11.setGeometry(QtCore.QRect(10, 350, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_11.setFont(font)
        self.label_11.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_11.setObjectName("label_11")
        self.lcdNumber_4 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_4.setGeometry(QtCore.QRect(110, 340, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_4.setFont(font)
        self.lcdNumber_4.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_4.setDigitCount(5)
        self.lcdNumber_4.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_4.setObjectName("lcdNumber_4")
        self.lcdNumber_32 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_32.setGeometry(QtCore.QRect(190, 340, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_32.setFont(font)
        self.lcdNumber_32.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_32.setDigitCount(5)
        self.lcdNumber_32.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_32.setObjectName("lcdNumber_32")
        self.lcdNumber_33 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_33.setGeometry(QtCore.QRect(270, 340, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_33.setFont(font)
        self.lcdNumber_33.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_33.setDigitCount(5)
        self.lcdNumber_33.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_33.setObjectName("lcdNumber_33")
        self.label_12 = QtWidgets.QLabel(Form)
        self.label_12.setGeometry(QtCore.QRect(340, 350, 21, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_12.setFont(font)
        self.label_12.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(Form)
        self.label_13.setGeometry(QtCore.QRect(140, 390, 151, 31))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_13.setFont(font)
        self.label_13.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.label_14 = QtWidgets.QLabel(Form)
        self.label_14.setGeometry(QtCore.QRect(10, 470, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_14.setFont(font)
        self.label_14.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_14.setObjectName("label_14")
        self.label_15 = QtWidgets.QLabel(Form)
        self.label_15.setGeometry(QtCore.QRect(10, 510, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_15.setFont(font)
        self.label_15.setStyleSheet("QLabel{\n"
"   color:\"white\"\n"
"}")
        self.label_15.setObjectName("label_15")
        self.label_16 = QtWidgets.QLabel(Form)
        self.label_16.setGeometry(QtCore.QRect(10, 550, 100, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_16.setFont(font)
        self.label_16.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_16.setObjectName("label_16")
        self.lcdNumber_37 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_37.setGeometry(QtCore.QRect(110, 430, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_37.setFont(font)
        self.lcdNumber_37.setStyleSheet("QWidget{\n"
"background-color:\"white\"\n"
"}")
        self.lcdNumber_37.setDigitCount(5)
        self.lcdNumber_37.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_37.setObjectName("lcdNumber_37")
        self.lcdNumber_38 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_38.setGeometry(QtCore.QRect(190, 430, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_38.setFont(font)
        self.lcdNumber_38.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_38.setDigitCount(5)
        self.lcdNumber_38.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_38.setObjectName("lcdNumber_38")
        self.lcdNumber_39 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_39.setGeometry(QtCore.QRect(270, 430, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_39.setFont(font)
        self.lcdNumber_39.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_39.setDigitCount(5)
        self.lcdNumber_39.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_39.setObjectName("lcdNumber_39")
        self.lcdNumber_5 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_5.setGeometry(QtCore.QRect(110, 470, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_5.setFont(font)
        self.lcdNumber_5.setStyleSheet("QWidget{\n"
"background-color:\"white\"\n"
"}")
        self.lcdNumber_5.setDigitCount(5)
        self.lcdNumber_5.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_5.setObjectName("lcdNumber_5")
        self.lcdNumber_6 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_6.setGeometry(QtCore.QRect(190, 470, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_6.setFont(font)
        self.lcdNumber_6.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_6.setDigitCount(5)
        self.lcdNumber_6.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_6.setObjectName("lcdNumber_6")
        self.lcdNumber_7 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_7.setGeometry(QtCore.QRect(270, 470, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_7.setFont(font)
        self.lcdNumber_7.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_7.setDigitCount(5)
        self.lcdNumber_7.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_7.setObjectName("lcdNumber_7")
        self.lcdNumber_34 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_34.setGeometry(QtCore.QRect(110, 510, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_34.setFont(font)
        self.lcdNumber_34.setStyleSheet("QWidget{\n"
"background-color:\"white\"\n"
"}")
        self.lcdNumber_34.setDigitCount(5)
        self.lcdNumber_34.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_34.setObjectName("lcdNumber_34")
        self.lcdNumber_35 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_35.setGeometry(QtCore.QRect(190, 510, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_35.setFont(font)
        self.lcdNumber_35.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_35.setDigitCount(5)
        self.lcdNumber_35.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_35.setObjectName("lcdNumber_35")
        self.lcdNumber_36 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_36.setGeometry(QtCore.QRect(270, 510, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_36.setFont(font)
        self.lcdNumber_36.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_36.setDigitCount(5)
        self.lcdNumber_36.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_36.setObjectName("lcdNumber_36")
        self.lcdNumber_18 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_18.setGeometry(QtCore.QRect(110, 550, 70, 31))
        self.lcdNumber_18.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_18.setDigitCount(5)
        self.lcdNumber_18.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_18.setObjectName("lcdNumber_18")
        self.lcdNumber_19 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_19.setGeometry(QtCore.QRect(190, 550, 70, 31))
        self.lcdNumber_19.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_19.setDigitCount(5)
        self.lcdNumber_19.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_19.setObjectName("lcdNumber_19")
        self.lcdNumber_20 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_20.setGeometry(QtCore.QRect(270, 550, 70, 31))
        self.lcdNumber_20.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_20.setDigitCount(5)
        self.lcdNumber_20.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_20.setObjectName("lcdNumber_20")
        self.label_17 = QtWidgets.QLabel(Form)
        self.label_17.setGeometry(QtCore.QRect(340, 480, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_17.setFont(font)
        self.label_17.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_17.setObjectName("label_17")
        self.label_18 = QtWidgets.QLabel(Form)
        self.label_18.setGeometry(QtCore.QRect(340, 520, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_18.setFont(font)
        self.label_18.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_18.setObjectName("label_18")
        self.label_19 = QtWidgets.QLabel(Form)
        self.label_19.setGeometry(QtCore.QRect(340, 550, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_19.setFont(font)
        self.label_19.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_19.setObjectName("label_19")
        self.label_20 = QtWidgets.QLabel(Form)
        self.label_20.setGeometry(QtCore.QRect(140, 590, 161, 31))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_20.setFont(font)
        self.label_20.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_20.setAlignment(QtCore.Qt.AlignCenter)
        self.label_20.setObjectName("label_20")
        self.label_21 = QtWidgets.QLabel(Form)
        self.label_21.setGeometry(QtCore.QRect(10, 660, 81, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_21.setFont(font)
        self.label_21.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_21.setObjectName("label_21")
        self.label_22 = QtWidgets.QLabel(Form)
        self.label_22.setGeometry(QtCore.QRect(10, 690, 91, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_22.setFont(font)
        self.label_22.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_22.setObjectName("label_22")
        self.label_100 = QtWidgets.QLabel(Form)
        self.label_100.setGeometry(QtCore.QRect(10, 720, 91, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_100.setFont(font)
        self.label_100.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_100.setObjectName("label_100")
        self.lcdNumber_8 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_8.setGeometry(QtCore.QRect(110, 640, 221, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_8.setFont(font)
        self.lcdNumber_8.setStyleSheet("QWidget{\n"
"     background-color:\"white\"\n"
"}\n"
"")
        self.lcdNumber_8.setDigitCount(16)
        self.lcdNumber_8.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_8.setObjectName("lcdNumber_8")
        self.lcdNumber_9 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_9.setGeometry(QtCore.QRect(110, 680, 221, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_9.setFont(font)
        self.lcdNumber_9.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_9.setDigitCount(16)
        self.lcdNumber_9.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_9.setObjectName("lcdNumber_9")

        self.lcdNumber_22 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_22.setGeometry(QtCore.QRect(110, 720, 221, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_22.setFont(font)
        self.lcdNumber_22.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_22.setDigitCount(16)
        self.lcdNumber_22.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_22.setObjectName("lcdNumber_22")

        self.lcdNumber_23 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_23.setGeometry(QtCore.QRect(930, 190, 100, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_23.setFont(font)
        self.lcdNumber_23.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_23.setDigitCount(6)
        self.lcdNumber_23.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_23.setObjectName("lcdNumber_23")

        self.lcdNumber_24 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_24.setGeometry(QtCore.QRect(1040, 190, 100, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_24.setFont(font)
        self.lcdNumber_24.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_24.setDigitCount(6)
        self.lcdNumber_24.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_24.setObjectName("lcdNumber_24")

        self.lcdNumber_25 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_25.setGeometry(QtCore.QRect(930, 230, 100, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_25.setFont(font)
        self.lcdNumber_25.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_25.setDigitCount(6)
        self.lcdNumber_25.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_25.setObjectName("lcdNumber_25")
        self.lcdNumber_47 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_47.setGeometry(QtCore.QRect(1040, 230, 100, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_47.setFont(font)
        self.lcdNumber_47.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_47.setDigitCount(6)
        self.lcdNumber_47.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_47.setObjectName("lcdNumber_47")
        self.label_23 = QtWidgets.QLabel(Form)
        self.label_23.setGeometry(QtCore.QRect(530, 90, 131, 41))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_23.setFont(font)
        self.label_23.setStyleSheet("QLabel{\n"
"   color:\"white\"\n"
"}")
        self.label_23.setAlignment(QtCore.Qt.AlignCenter)
        self.label_23.setObjectName("label_23")
        self.label_24 = QtWidgets.QLabel(Form)
        self.label_24.setGeometry(QtCore.QRect(410, 160, 71, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_24.setFont(font)
        self.label_24.setStyleSheet("QLabel{\n"
"     color:\"white\"\n"
"}")
        self.label_24.setObjectName("label_24")
        self.label_25 = QtWidgets.QLabel(Form)
        self.label_25.setGeometry(QtCore.QRect(410, 200, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_25.setFont(font)
        self.label_25.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_25.setObjectName("label_25")
        self.lcdNumber_13 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_13.setGeometry(QtCore.QRect(490, 720, 70, 31))
        self.lcdNumber_13.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_13.setDigitCount(5)
        self.lcdNumber_13.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_13.setObjectName("lcdNumber_13")
        self.lcdNumber_10 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_10.setGeometry(QtCore.QRect(490, 190, 70, 31))
        self.lcdNumber_10.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_10.setDigitCount(5)
        self.lcdNumber_10.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_10.setObjectName("lcdNumber_10")
        self.lcdNumber_41 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_41.setGeometry(QtCore.QRect(570, 190, 70, 31))
        self.lcdNumber_41.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_41.setDigitCount(5)
        self.lcdNumber_41.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_41.setObjectName("lcdNumber_41")
        self.lcdNumber_42 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_42.setGeometry(QtCore.QRect(650, 190, 70, 31))
        self.lcdNumber_42.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_42.setDigitCount(5)
        self.lcdNumber_42.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_42.setObjectName("lcdNumber_42")
        self.label_26 = QtWidgets.QLabel(Form)
        self.label_26.setGeometry(QtCore.QRect(720, 160, 31, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_26.setFont(font)
        self.label_26.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_26.setObjectName("label_26")
        self.label_27 = QtWidgets.QLabel(Form)
        self.label_27.setGeometry(QtCore.QRect(1160, 240, 70, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_27.setFont(font)
        self.label_27.setStyleSheet("QLabel{\n"
"   color:\"white\"\n"
"}")
        self.label_27.setObjectName("label_27")
        self.label_28 = QtWidgets.QLabel(Form)
        self.label_28.setGeometry(QtCore.QRect(520, 360, 161, 41))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_28.setFont(font)
        self.label_28.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_28.setAlignment(QtCore.Qt.AlignCenter)
        self.label_28.setObjectName("label_28")
        self.label_29 = QtWidgets.QLabel(Form)
        self.label_29.setGeometry(QtCore.QRect(410, 410, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_29.setFont(font)
        self.label_29.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_29.setObjectName("label_29")
        self.label_30 = QtWidgets.QLabel(Form)
        self.label_30.setGeometry(QtCore.QRect(410, 450, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_30.setFont(font)
        self.label_30.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_30.setObjectName("label_30")
        self.lcdNumber_16 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_16.setGeometry(QtCore.QRect(490, 440, 100, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_16.setFont(font)
        self.lcdNumber_16.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_16.setDigitCount(4)
        self.lcdNumber_16.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_16.setObjectName("lcdNumber_16")
        self.lcdNumber_40 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_40.setGeometry(QtCore.QRect(600, 440, 100, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_40.setFont(font)
        self.lcdNumber_40.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_40.setDigitCount(4)
        self.lcdNumber_40.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_40.setObjectName("lcdNumber_40")
        self.label_31 = QtWidgets.QLabel(Form)
        self.label_31.setGeometry(QtCore.QRect(510, 480, 181, 41))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_31.setFont(font)
        self.label_31.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_31.setAlignment(QtCore.Qt.AlignCenter)
        self.label_31.setObjectName("label_31")
        self.label_32 = QtWidgets.QLabel(Form)
        self.label_32.setGeometry(QtCore.QRect(410, 530, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_32.setFont(font)
        self.label_32.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_32.setObjectName("label_32")
        self.label_33 = QtWidgets.QLabel(Form)
        self.label_33.setGeometry(QtCore.QRect(410, 570, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_33.setFont(font)
        self.label_33.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_33.setObjectName("label_33")
        self.lcdNumber_12 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_12.setGeometry(QtCore.QRect(490, 560, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_12.setFont(font)
        self.lcdNumber_12.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_12.setDigitCount(5)
        self.lcdNumber_12.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_12.setObjectName("lcdNumber_12")
        self.lcdNumber_43 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_43.setGeometry(QtCore.QRect(570, 560, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_43.setFont(font)
        self.lcdNumber_43.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_43.setDigitCount(5)
        self.lcdNumber_43.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_43.setObjectName("lcdNumber_43")
        self.lcdNumber_44 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_44.setGeometry(QtCore.QRect(650, 560, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_44.setFont(font)
        self.lcdNumber_44.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_44.setDigitCount(5)
        self.lcdNumber_44.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_44.setObjectName("lcdNumber_44")
        self.label_34 = QtWidgets.QLabel(Form)
        self.label_34.setGeometry(QtCore.QRect(520, 600, 161, 31))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_34.setFont(font)
        self.label_34.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_34.setAlignment(QtCore.Qt.AlignCenter)
        self.label_34.setObjectName("label_34")
        self.label_35 = QtWidgets.QLabel(Form)
        self.label_35.setGeometry(QtCore.QRect(410, 650, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_35.setFont(font)
        self.label_35.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_35.setObjectName("label_35")
        self.label_36 = QtWidgets.QLabel(Form)
        self.label_36.setGeometry(QtCore.QRect(410, 690, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_36.setFont(font)
        self.label_36.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_36.setObjectName("label_36")
        self.lcdNumber_11 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_11.setGeometry(QtCore.QRect(490, 680, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_11.setFont(font)
        self.lcdNumber_11.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_11.setDigitCount(5)
        self.lcdNumber_11.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_11.setObjectName("lcdNumber_11")
        self.lcdNumber_45 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_45.setGeometry(QtCore.QRect(570, 680, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_45.setFont(font)
        self.lcdNumber_45.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_45.setDigitCount(5)
        self.lcdNumber_45.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_45.setObjectName("lcdNumber_45")
        self.lcdNumber_46 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_46.setGeometry(QtCore.QRect(650, 680, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_46.setFont(font)
        self.lcdNumber_46.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_46.setDigitCount(5)
        self.lcdNumber_46.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_46.setObjectName("lcdNumber_46")
        self.label_37 = QtWidgets.QLabel(Form)
        self.label_37.setGeometry(QtCore.QRect(200, 40, 800, 40))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_37.setFont(font)
        self.label_37.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_37.setAlignment(QtCore.Qt.AlignCenter)
        self.label_37.setObjectName("label_37")
        self.lcdNumber_21 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_21.setGeometry(QtCore.QRect(490, 320, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_21.setFont(font)
        self.lcdNumber_21.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_21.setDigitCount(5)
        self.lcdNumber_21.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_21.setObjectName("lcdNumber_21")
        self.label_38 = QtWidgets.QLabel(Form)
        self.label_38.setGeometry(QtCore.QRect(1160, 160, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_38.setFont(font)
        self.label_38.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_38.setObjectName("label_38")
        self.label_39 = QtWidgets.QLabel(Form)
        self.label_39.setGeometry(QtCore.QRect(900, 90, 250, 31))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_39.setFont(font)
        self.label_39.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_39.setAlignment(QtCore.Qt.AlignCenter)
        self.label_39.setObjectName("label_39")
        self.label_40 = QtWidgets.QLabel(Form)
        self.label_40.setGeometry(QtCore.QRect(870, 160, 60, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_40.setFont(font)
        self.label_40.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_40.setObjectName("label_40")
        self.label_41 = QtWidgets.QLabel(Form)
        self.label_41.setGeometry(QtCore.QRect(870, 200, 60, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_41.setFont(font)
        self.label_41.setStyleSheet("QLabel{\n"
"color:\"white\"\n"
"}")
        self.label_41.setObjectName("label_41")
        self.label_42 = QtWidgets.QLabel(Form)
        self.label_42.setGeometry(QtCore.QRect(870, 240, 60, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_42.setFont(font)
        self.label_42.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_42.setObjectName("label_42")
        self.label_43 = QtWidgets.QLabel(Form)
        self.label_43.setGeometry(QtCore.QRect(1160, 200, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_43.setFont(font)
        self.label_43.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_43.setObjectName("label_43")
        self.lcdNumber_14 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_14.setGeometry(QtCore.QRect(930, 150, 100, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_14.setFont(font)
        self.lcdNumber_14.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_14.setDigitCount(5)
        self.lcdNumber_14.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_14.setObjectName("lcdNumber_14")
        self.lcdNumber_15 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_15.setGeometry(QtCore.QRect(1040, 150, 100, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_15.setFont(font)
        self.lcdNumber_15.setStyleSheet("QWidget{\n"
"    Background-color:\"white\"\n"
"}")
        self.lcdNumber_15.setDigitCount(6)
        self.lcdNumber_15.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_15.setObjectName("lcdNumber_15")
        self.label_44 = QtWidgets.QLabel(Form)
        self.label_44.setGeometry(QtCore.QRect(960, 480, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_44.setFont(font)
        self.label_44.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_44.setAlignment(QtCore.Qt.AlignCenter)
        self.label_44.setObjectName("label_44")
        self.pushButton = QtWidgets.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(960, 540, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(Form)
        self.pushButton_2.setGeometry(QtCore.QRect(960, 580, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_2.setObjectName("pushButton_2")
        self.label_45 = QtWidgets.QLabel(Form)
        self.label_45.setGeometry(QtCore.QRect(1070, 540, 21, 20))
        self.label_45.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_45.setText("")
        self.label_45.setObjectName("label_45")
        self.label_46 = QtWidgets.QLabel(Form)
        self.label_46.setGeometry(QtCore.QRect(1070, 580, 21, 20))
        self.label_46.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}\n"
"\n"
"")
        self.label_46.setText("")
        self.label_46.setObjectName("label_46")
        self.doubleSpinBox = QtWidgets.QDoubleSpinBox(Form)
        self.doubleSpinBox.setGeometry(QtCore.QRect(490, 150, 221, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.doubleSpinBox.setFont(font)
        self.doubleSpinBox.setStyleSheet("QWidget{\n"
"background-color:\"white\"\n"
"}")
        self.doubleSpinBox.setObjectName("doubleSpinBox")
        self.doubleSpinBox_2 = QtWidgets.QDoubleSpinBox(Form)
        self.doubleSpinBox_2.setGeometry(QtCore.QRect(490, 400, 221, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.doubleSpinBox_2.setFont(font)
        self.doubleSpinBox_2.setStyleSheet("QWidget{\n"
"background-color:\"white\"\n"
"}")
        self.doubleSpinBox_2.setMinimum(-100.0)
        self.doubleSpinBox_2.setMaximum(100.0)
        self.doubleSpinBox_2.setObjectName("doubleSpinBox_2")
        self.doubleSpinBox_3 = QtWidgets.QDoubleSpinBox(Form)
        self.doubleSpinBox_3.setGeometry(QtCore.QRect(490, 520, 221, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.doubleSpinBox_3.setFont(font)
        self.doubleSpinBox_3.setStyleSheet("QWidget{\n"
"background-color:\"white\"\n"
"}")
        self.doubleSpinBox_3.setMinimum(-100.0)
        self.doubleSpinBox_3.setMaximum(100.0)
        self.doubleSpinBox_3.setObjectName("doubleSpinBox_3")
        self.doubleSpinBox_4 = QtWidgets.QDoubleSpinBox(Form)
        self.doubleSpinBox_4.setGeometry(QtCore.QRect(490, 640, 221, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.doubleSpinBox_4.setFont(font)
        self.doubleSpinBox_4.setStyleSheet("QWidget{\n"
"background-color:\"white\"\n"
"}")
        self.doubleSpinBox_4.setMinimum(-360.0)
        self.doubleSpinBox_4.setMaximum(360.0)
        self.doubleSpinBox_4.setObjectName("doubleSpinBox_4")
        self.pushButton_3 = QtWidgets.QPushButton(Form)
        self.pushButton_3.setGeometry(QtCore.QRect(750, 150, 80, 23))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QtWidgets.QPushButton(Form)
        self.pushButton_4.setGeometry(QtCore.QRect(750, 190, 80, 23))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_4.setFont(font)
        self.pushButton_4.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_5 = QtWidgets.QPushButton(Form)
        self.pushButton_5.setGeometry(QtCore.QRect(750, 410, 80, 23))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_5.setFont(font)
        self.pushButton_5.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_6 = QtWidgets.QPushButton(Form)
        self.pushButton_6.setGeometry(QtCore.QRect(750, 450, 80, 23))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_6.setFont(font)
        self.pushButton_6.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_7 = QtWidgets.QPushButton(Form)
        self.pushButton_7.setGeometry(QtCore.QRect(750, 530, 80, 23))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_7.setFont(font)
        self.pushButton_7.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_7.setObjectName("pushButton_7")
        self.pushButton_8 = QtWidgets.QPushButton(Form)
        self.pushButton_8.setGeometry(QtCore.QRect(750, 570, 80, 23))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_8.setFont(font)
        self.pushButton_8.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_8.setObjectName("pushButton_8")
        self.pushButton_9 = QtWidgets.QPushButton(Form)
        self.pushButton_9.setGeometry(QtCore.QRect(750, 650, 80, 23))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_9.setFont(font)
        self.pushButton_9.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_9.setObjectName("pushButton_9")
        self.pushButton_10 = QtWidgets.QPushButton(Form)
        self.pushButton_10.setGeometry(QtCore.QRect(750, 690, 80, 23))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_10.setFont(font)
        self.pushButton_10.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_10.setObjectName("pushButton_10")
        self.label_47 = QtWidgets.QLabel(Form)
        self.label_47.setGeometry(QtCore.QRect(1200, 20, 161, 41))
        self.label_47.setStyleSheet("image:url(:/newPrefix/logo-white.png)")
        self.label_47.setText("")
        self.label_47.setObjectName("label_47")
        self.label_48 = QtWidgets.QLabel(Form)
        self.label_48.setGeometry(QtCore.QRect(840, 160, 21, 16))
        self.label_48.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_48.setText("")
        self.label_48.setObjectName("label_48")
        self.label_49 = QtWidgets.QLabel(Form)
        self.label_49.setGeometry(QtCore.QRect(840, 190, 21, 16))
        self.label_49.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}\n"
"\n"
"")
        self.label_49.setText("")
        self.label_49.setObjectName("label_49")
        self.label_50 = QtWidgets.QLabel(Form)
        self.label_50.setGeometry(QtCore.QRect(840, 410, 21, 16))
        self.label_50.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_50.setText("")
        self.label_50.setObjectName("label_50")
        self.label_51 = QtWidgets.QLabel(Form)
        self.label_51.setGeometry(QtCore.QRect(840, 450, 21, 16))
        self.label_51.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}\n"
"\n"
"")
        self.label_51.setText("")
        self.label_51.setObjectName("label_51")
        self.label_52 = QtWidgets.QLabel(Form)
        self.label_52.setGeometry(QtCore.QRect(840, 530, 21, 16))
        self.label_52.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_52.setText("")
        self.label_52.setObjectName("label_52")
        self.label_53 = QtWidgets.QLabel(Form)
        self.label_53.setGeometry(QtCore.QRect(840, 570, 21, 16))
        self.label_53.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}\n"
"\n"
"")
        self.label_53.setText("")
        self.label_53.setObjectName("label_53")
        self.label_54 = QtWidgets.QLabel(Form)
        self.label_54.setGeometry(QtCore.QRect(840, 650, 21, 16))
        self.label_54.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_54.setText("")
        self.label_54.setObjectName("label_54")
        self.label_55 = QtWidgets.QLabel(Form)
        self.label_55.setGeometry(QtCore.QRect(840, 690, 21, 16))
        self.label_55.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}\n"
"\n"
"")
        self.label_55.setText("")
        self.label_55.setObjectName("label_55")
        self.label_56 = QtWidgets.QLabel(Form)
        self.label_56.setGeometry(QtCore.QRect(870, 620, 241, 51))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_56.setFont(font)
        self.label_56.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_56.setAlignment(QtCore.Qt.AlignCenter)
        self.label_56.setObjectName("label_56")
        self.pushButton_11 = QtWidgets.QPushButton(Form)
        self.pushButton_11.setGeometry(QtCore.QRect(930, 680, 131, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_11.setFont(font)
        self.pushButton_11.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_11.setObjectName("pushButton_11")
        self.pushButton_12 = QtWidgets.QPushButton(Form)
        self.pushButton_12.setGeometry(QtCore.QRect(930, 710, 131, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_12.setFont(font)
        self.pushButton_12.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_12.setObjectName("pushButton_12")
        self.label_57 = QtWidgets.QLabel(Form)
        self.label_57.setGeometry(QtCore.QRect(1100, 680, 21, 20))
        self.label_57.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_57.setText("")
        self.label_57.setObjectName("label_57")
        self.label_58 = QtWidgets.QLabel(Form)
        self.label_58.setGeometry(QtCore.QRect(1100, 710, 21, 20))
        self.label_58.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_58.setText("")
        self.label_58.setObjectName("label_58")
        self.label_59 = QtWidgets.QLabel(Form)
        self.label_59.setGeometry(QtCore.QRect(960, 360, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_59.setFont(font)
        self.label_59.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_59.setAlignment(QtCore.Qt.AlignCenter)
        self.label_59.setObjectName("label_59")
        self.pushButton_13 = QtWidgets.QPushButton(Form)
        self.pushButton_13.setGeometry(QtCore.QRect(960, 420, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_13.setFont(font)
        self.pushButton_13.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_13.setObjectName("pushButton_13")
        self.pushButton_14 = QtWidgets.QPushButton(Form)
        self.pushButton_14.setGeometry(QtCore.QRect(270, 590, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_14.setFont(font)
        self.pushButton_14.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_14.setObjectName("pushButton_14")
        self.label_60 = QtWidgets.QLabel(Form)
        self.label_60.setGeometry(QtCore.QRect(1070, 420, 21, 20))
        self.label_60.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_60.setText("")
        self.label_60.setObjectName("label_60")
        self.label_61 = QtWidgets.QLabel(Form)
        self.label_61.setGeometry(QtCore.QRect(380, 590, 21, 20))
        self.label_61.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}\n"
"\n"
"")
        self.label_61.setText("")
        self.label_61.setObjectName("label_61")
        self.label_62 = QtWidgets.QLabel(Form)
        self.label_62.setGeometry(QtCore.QRect(520, 240, 200, 41))
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.label_62.setFont(font)
        self.label_62.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_62.setAlignment(QtCore.Qt.AlignCenter)
        self.label_62.setObjectName("label_62")
        self.doubleSpinBox_5 = QtWidgets.QDoubleSpinBox(Form)
        self.doubleSpinBox_5.setGeometry(QtCore.QRect(490, 280, 221, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.doubleSpinBox_5.setFont(font)
        self.doubleSpinBox_5.setStyleSheet("QWidget{\n"
"background-color:\"white\"\n"
"}")
        self.doubleSpinBox_5.setMaximum(250.0)
        self.doubleSpinBox_5.setObjectName("doubleSpinBox_5")
        self.lcdNumber_17 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_17.setGeometry(QtCore.QRect(570, 320, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_17.setFont(font)
        self.lcdNumber_17.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_17.setDigitCount(5)
        self.lcdNumber_17.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_17.setObjectName("lcdNumber_17")
        self.lcdNumber_48 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_48.setGeometry(QtCore.QRect(650, 320, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_48.setFont(font)
        self.lcdNumber_48.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_48.setDigitCount(5)
        self.lcdNumber_48.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_48.setObjectName("lcdNumber_48")
        
        self.lcdNumber_49 = QtWidgets.QLCDNumber(Form)
        self.lcdNumber_49.setGeometry(QtCore.QRect(755, 250, 70, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lcdNumber_49.setFont(font)
        self.lcdNumber_49.setStyleSheet("QWidget{\n"
"    background-color:\"white\"\n"
"}")
        self.lcdNumber_49.setDigitCount(5)
        self.lcdNumber_49.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_49.setObjectName("lcdNumber_49")
        
        self.label_63 = QtWidgets.QLabel(Form)
        self.label_63.setGeometry(QtCore.QRect(410, 290, 71, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_63.setFont(font)
        self.label_63.setStyleSheet("QLabel{\n"
"     color:\"white\"\n"
"}")
        self.label_63.setObjectName("label_63")
        self.label_64 = QtWidgets.QLabel(Form)
        self.label_64.setGeometry(QtCore.QRect(410, 330, 67, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_64.setFont(font)
        self.label_64.setStyleSheet("QLabel{\n"
"    color:\"white\"\n"
"}")
        self.label_64.setObjectName("label_64")
        self.pushButton_15 = QtWidgets.QPushButton(Form)
        self.pushButton_15.setGeometry(QtCore.QRect(750, 290, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_15.setFont(font)
        self.pushButton_15.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_15.setObjectName("pushButton_15")
        self.pushButton_16 = QtWidgets.QPushButton(Form)
        self.pushButton_16.setGeometry(QtCore.QRect(750, 330, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_16.setFont(font)
        self.pushButton_16.setStyleSheet("QWidget{\n"
"    background-color:\"#919090\"\n"
"}")
        self.pushButton_16.setObjectName("pushButton_16")
        self.label_65 = QtWidgets.QLabel(Form)
        self.label_65.setGeometry(QtCore.QRect(840, 290, 21, 16))
        self.label_65.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_65.setText("")
        self.label_65.setObjectName("label_65")
        self.label_66 = QtWidgets.QLabel(Form)
        self.label_66.setGeometry(QtCore.QRect(840, 330, 21, 16))
        self.label_66.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}\n"
"\n"
"")
        self.label_66.setText("")
        self.label_66.setObjectName("label_66")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)



        # callbacks when the push button is clicked #
        self.pushButton.clicked.connect(self.start_logging)
        self.pushButton_2.clicked.connect(self.stop_logging)
        self.pushButton_3.clicked.connect(self.on_surge)
        self.pushButton_4.clicked.connect(self.off_surge)
        self.pushButton_5.clicked.connect(self.on_roll)
        self.pushButton_6.clicked.connect(self.off_roll)
        self.pushButton_7.clicked.connect(self.on_pitch)
        self.pushButton_8.clicked.connect(self.off_pitch)
        self.pushButton_9.clicked.connect(self.on_yaw)
        self.pushButton_10.clicked.connect(self.off_yaw)      
        self.pushButton_11.clicked.connect(self.glider_mode)
        self.pushButton_12.clicked.connect(self.auv_mode)
        self.pushButton_13.clicked.connect(self.on_off_oas)
        self.pushButton_14.clicked.connect(self.reset_nav_reference)
        self.pushButton_15.clicked.connect(self.on_depth) 
        self.pushButton_16.clicked.connect(self.off_depth)



    def start_logging(self) :
        self.label_46.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_45.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")
        self.logger_sts = 1
        self.publishLogger(self.logger_sts)

    def stop_logging(self) :
        self.label_45.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_46.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")
        self.logger_sts = 0
        self.publishLogger(self.logger_sts)

    def on_surge(self) : 
        self.label_49.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_48.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")   
        self.status_surge = 1
        value_surge = self.doubleSpinBox.value()
        self.publishSurge(self.status_surge, value_surge)

    def off_surge(self) :  
        self.label_48.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_49.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")
        self.status_surge = 0
        value_surge = self.doubleSpinBox.value()
        self.publishSurge(self.status_surge, value_surge)

    def on_roll(self) :  
        self.label_51.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_50.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")  

        self.status_roll = 1
        value_roll = self.doubleSpinBox_2.value()
        self.publishRoll(self.status_roll, value_roll)

    def off_roll(self) :
        self.label_50.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_51.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")  

        self.status_roll = 0
        value_roll = self.doubleSpinBox_2.value()
        self.publishRoll(self.status_roll, value_roll)


    def on_pitch(self) :  
        self.label_53.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_52.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")    
        self.status_pitch = 1
        value_pitch = self.doubleSpinBox_3.value()
        self.publishPitch(self.status_pitch, value_pitch)

    def off_pitch(self) :    
        self.label_52.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_53.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")  
        self.status_pitch = 0
        value_pitch = self.doubleSpinBox_3.value()
        self.publishPitch(self.status_pitch, value_pitch)

    def on_yaw(self) :  
        self.label_55.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_54.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")    

        self.status_yaw = 1
        value_yaw = self.doubleSpinBox_4.value()
        self.publishYaw(self.status_yaw, value_yaw)

    def off_yaw(self) :  
        self.label_54.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_55.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}") 
        self.status_yaw = 0
        value_yaw = self.doubleSpinBox_4.value()
        self.publishYaw(self.status_yaw, value_yaw)

    def glider_mode(self) :    
        self.label_58.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_57.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")  

    def auv_mode(self) : 
        self.label_57.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_58.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")  

    def on_off_oas(self) :
        if self.on_oas == 1 :
            self.on_oas = 0
            self.label_60.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        else :
            self.on_oas = 1
            self.label_60.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")
        self.publishOAS(self.on_oas)

    def reset_nav_reference(self) :
        data = Int32()
        data.data = 0
        self.pub_reset_nav_reference.publish(data)
        if self.flag_reset == 1 :
            self.flag_reset = 0
            self.label_61.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")
        else :
            self.flag_reset = 1
            self.label_61.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")

    def on_depth(self) : 
        self.label_66.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_65.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")  


        self.status_depth = 1
        value_depth = self.doubleSpinBox_5.value()
        self.publishDepth(self.status_depth, value_depth)
        print("set is pressed")

    def off_depth(self) : 
        self.label_65.setStyleSheet("QWidget{\n"
"    background-color:\"#F40C0C\"\n"
"}")
        self.label_66.setStyleSheet("QWidget{\n"
"    background-color:\"#40BF5E\"\n"
"}")  


        self.status_depth = 0
        value_depth = self.doubleSpinBox_5.value()
        self.publishDepth(self.status_depth, value_depth)
        print("set is pressed")

    def publishSurge(self, a,b) :
        data_as = Int32()
        data_bs = Float64()
        data_as.data = a
        data_bs.data = b
        self.pub_surge_status.publish(data_as)
        self.pub_sp_surge.publish(data_bs)

    def publishRoll(self, a,b) :
        data_ar = Int32()
        data_br = Float64()
        data_ar.data = a
        data_br.data = b
        self.pub_roll_status.publish(data_ar)
        self.pub_sp_roll.publish(data_br)

    def publishPitch(self, a,b) :
        data_ap = Int32()
        data_bp = Float64()
        data_ap.data = a
        data_bp.data = b
        self.pub_pitch_status.publish(data_ap)
        self.pub_sp_pitch.publish(data_bp)

    def publishYaw(self, a,b) :
        data_ay = Int32()
        data_by = Float64()
        data_ay.data = a
        data_by.data = b
        self.pub_yaw_status.publish(data_ay)
        self.pub_sp_yaw.publish(data_by)

    def publishDepth(self, a,b) :
        data_ad = Int32()
        data_bd = Float64()
        data_ad.data = a
        data_bd.data = b
        self.pub_depth_status.publish(data_ad)
        self.pub_sp_depth.publish(data_bd)

    def publishOAS(self, a) :
        data_a = Int32()
        data_a.data = a
        self.pub_oas_status.publish(data_a)
        if a == 0 :
            data_b = String()
            data_b.data = 'safe'
            self.pub_oas_notif_test.publish(data_b)

    def publishLogger(self,b__) :
        data_log = Int32()
        data_log.data = b__
        self.pub_logger_status.publish(data_log)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Hybrid Autonomous Underwater Glider - GUI"))
        self.label_2.setText(_translate("Form", "supported by SEMONGKO"))
        self.label_3.setText(_translate("Form", "Velocity"))
        self.label_4.setText(_translate("Form", "DVL Earth"))
        self.label_5.setText(_translate("Form", "DVL body"))
        self.label_6.setText(_translate("Form", "NAV body"))
        self.label_7.setText(_translate("Form", "m/s"))
        self.label_8.setText(_translate("Form", "m/s"))
        self.label_9.setText(_translate("Form", "m/s"))
        self.label_10.setText(_translate("Form", "Sensor/SP/Altitude"))
        self.label_11.setText(_translate("Form", "Depth"))
        self.label_12.setText(_translate("Form", "m"))
        self.label_13.setText(_translate("Form", "Attitude"))
        self.label_14.setText(_translate("Form", "IMU"))
        self.label_15.setText(_translate("Form", "DVL"))
        self.label_16.setText(_translate("Form", "NAV Position"))
        self.label_17.setText(_translate("Form", "Deg"))
        self.label_18.setText(_translate("Form", "Deg"))
        self.label_19.setText(_translate("Form", "m"))
        self.label_20.setText(_translate("Form", "GPS"))
        self.label_21.setText(_translate("Form", "Latitude"))
        self.label_22.setText(_translate("Form", "Longitude"))
        self.label_23.setText(_translate("Form", "Surge "))
        self.label_24.setText(_translate("Form", "Set-Point"))
        self.label_25.setText(_translate("Form", "Actual"))
        self.label_26.setText(_translate("Form", "m/s"))
        self.label_27.setText(_translate("Form", "mBar/C"))
        self.label_28.setText(_translate("Form", "MM_Roll"))
        self.label_29.setText(_translate("Form", "Set-Point"))
        self.label_30.setText(_translate("Form", "Actual"))
        self.label_31.setText(_translate("Form", "MM_Pitch"))
        self.label_32.setText(_translate("Form", "Set-Point"))
        self.label_33.setText(_translate("Form", "Position"))
        self.label_34.setText(_translate("Form", "Yaw"))
        self.label_35.setText(_translate("Form", "Set-Point"))
        self.label_36.setText(_translate("Form", "Heading"))
        self.label_37.setText(_translate("Form", "GUI Hybrid Autonomous Underwater Glider"))
        self.label_38.setText(_translate("Form", "Volt/mA"))
        self.label_39.setText(_translate("Form", "Kompartemen 4"))
        self.label_40.setText(_translate("Form", "INA219"))
        self.label_41.setText(_translate("Form", "SHT31"))
        self.label_42.setText(_translate("Form", "MS5803"))
        self.label_43.setText(_translate("Form", "%/C"))
        self.label_44.setText(_translate("Form", "Logger"))
        self.label_100.setText(_translate("Form", "Status"))
        self.pushButton.setText(_translate("Form", "Start"))
        self.pushButton_2.setText(_translate("Form", "Stop"))
        self.pushButton_3.setText(_translate("Form", "On1"))
        self.pushButton_4.setText(_translate("Form", "Off1"))
        self.pushButton_5.setText(_translate("Form", "On2"))
        self.pushButton_6.setText(_translate("Form", "Off2"))
        self.pushButton_7.setText(_translate("Form", "On3"))
        self.pushButton_8.setText(_translate("Form", "Off3"))
        self.pushButton_9.setText(_translate("Form", "On4"))
        self.pushButton_10.setText(_translate("Form", "Off4"))
        self.label_56.setText(_translate("Form", "Autonomous "))
        self.pushButton_11.setText(_translate("Form", "Glider Mode"))
        self.pushButton_12.setText(_translate("Form", "AUV Mode"))
        self.label_59.setText(_translate("Form", "OAS"))
        self.pushButton_13.setText(_translate("Form", "On/Off"))
        self.pushButton_14.setText(_translate("Form", "Reset"))
        self.label_62.setText(_translate("Form", "Buoy_Engine"))
        self.label_63.setText(_translate("Form", "Set-Point"))
        self.label_64.setText(_translate("Form", "Volume"))
        self.pushButton_15.setText(_translate("Form", "On6"))
        self.pushButton_16.setText(_translate("Form", "Off6"))

def eventHandler(e,frame=None,_=None):
        # ctrl+c on terminal
        if(e==2):
                print("Selamat Tinggal!")
                QtWidgets.QApplication.quit()  
                return    
        # Escape on terminal
        if e.key()  == Qt.Key_Escape :
                print("Selamat Tinggal!")
                QtWidgets.QApplication.quit()
        
if __name__ == "__main__":
    try :
        rospy.init_node('gui_node')
        app = QtWidgets.QApplication(sys.argv)
        Form = QtWidgets.QWidget()
        ui = Ui_Form()
        signal.signal(signal.SIGINT, eventHandler)
        ui.setupUi(Form)
        Form.keyPressEvent=eventHandler
        # Form.addAction()
        Form.showFullScreen()
        Form.show()
        sys.exit(app.exec_())

    except KeyboardInterrupt :
        print("keluar")


