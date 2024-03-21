#!/usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import Int64, Float32, Float64, Int32
from gb_msgs.msg import Gui

    
class BEMidVal:
    def __init__(self):
        self.publisherbemidval = rospy.Publisher("/netral_volume", Float32, queue_size=10)
        self.upper=0
        self.lower=0
        self.midval=0
        self.BEarray = np.array([350])
        self.frequency = 1
        self.current_depth = 0
        self.current_depth_sp=0
        self.current_be=0
        rospy.Subscriber("/gather_gui", Gui, self.update_current_state)
        rospy.Subscriber("/depth_sp", Float64, self.update_depth_sp)

    # update state on subscriber callback
    def update_current_state(self, data):
        self.current_be=data.internal_volume
        self.current_depth=data.depth_data
        

    def update_depth_sp(self, data):
        # resetting BE
        if(self.current_depth_sp != data.data):
            self.BEarray=np.array([self.midval])
        self.current_depth_sp = data.data
        
    # main program
    def mainProgram(self):
        # roa 0.8
        # if(abs(self.current_depth - self.current_depth_sp) < 0.8):
        self.BEarray=np.append(self.BEarray,self.current_be)
        self.upper=self.BEarray.max()
        self.lower=self.BEarray.min()
        print("upper, lower", self.upper, self.lower)
        self.midval=(self.upper+self.lower)/2     
        midval = Float32()
        midval.data = self.midval
        self.publisherbemidval.publish(midval)
            
        # make sure bearray not overflow
        if(len(self.BEarray) > 30):
            # slicing the BEarray
            self.BEarray =  self.BEarray[-30:]
        
        
if __name__ == "__main__":
    print("BE mid val calculator")
    rospy.init_node("BEmidval")
    # instantiate MPC class
    bemidval = BEMidVal()
    # main loop every 1 second
    rate = rospy.Rate(bemidval.frequency)
    # main loop
    while not rospy.is_shutdown():
        bemidval.mainProgram()
        rate.sleep()
    print("Node Closed")