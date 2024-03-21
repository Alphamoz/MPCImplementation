#!/usr/bin/env python

from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt
import pickle
# from PID import PID
# import rospy
# from std_msgs.msg import Int64, Float32, Float64, Int32
# from gb_msgs.msg import Valeport_Altimeter
# from cola2_msgs.msg import LinkquestDvl
# from gb_msgs.msg import Spatial, Gui
# from sensor_msgs.msg import NavSatFix
import math
import time as t

with open("p_br_pitch.pickle", "rb") as file:
    p_pitch = pickle.load(file)

with open("p_br_dr.pickle", "rb") as file:
    p_dr = pickle.load(file)


class MPC:
    def __init__(self):
        # rospy.Subscriber("/gather_gui", Gui, self.callback_print)
        self.current_depth_rate = 0
        self.current_pitch = 0
        self.current_be = 0
        self.current_mm = 0
        self.initialization()
        
    def initialization(self):
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
        self.MM = self.m.Var(value=0)
        self.mr = self.m.MV(value=0, lb=-10, ub=10)
        self.mr.STATUS = 1
        self.mr.DCOST = 10
        self.BE = self.m.Var(value=0)
        # BE.STATUS = 1  # Enable control
        # BE.DMAXHI = 15
        # BE.DMAXLO = -15
        # BE.DCOST = 1
        self.br = self.m.MV(value=0, lb=-15, ub=15)
        self.br.STATUS = 1
        self.br.DCOST = 10
        self.depth = self.m.Var(value=0)
        self.depth_rate = self.m.CV(value=0)
        self.m.Equation(self.depth == self.m.integral(self.depth_rate))
        self.m.Equation(self.BE == self.m.integral(self.br))
        self.m.Equation(self.MM == self.m.integral(self.mr))
if __name__ == "__main__":
    mpc_instance = MPC()
    print(vars(mpc_instance))
    print(vars(mpc_instance.m))
    mpc_instance.depth.value = 10
    with open('output_data.txt', 'a') as output:
        output.write(f'{mpc_instance.MM.value} {mpc_instance.mr.value} {mpc_instance.depth.value}\n')
    with open('output_data.txt', 'a') as output:
        output.write(f'{mpc_instance.MM.value} {mpc_instance.mr.value} {mpc_instance.depth.value}\n')
    print("End Of Code!")