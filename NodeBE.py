#!/usr/bin/env python

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

import rospy
import time

from std_msgs.msg import Float32

#BLDC pin
global en_BLDC
en_BLDC = "P8_19"

#Encoder Pin
global encoder_pin
encoder_pin = "P8_16"

#Valve Pin A dan B
global en_valveA, en_valveB
en_valveB = "P9_23"
en_valveA = "P9_27"

#Flowmeter pin
global pin_flow
pin_flow = "P9_12"

#Press Trans pin
global press_pin
press_pin = "P9_40"

class BuoyClass :
    'Node untuk atur gerakan Buoyancy Engine'

    def __init__(self) :
        rospy.Subscriber('/desired_internal_volume', Float32, self.getdesiredvolume)
        
        file = open("/home/ubuntu/catkin_ws/src/hybrid-glider-system/gb_control/src/low_level_control/heave_actuator/scripts/Volume.txt","r")
        self.InternalVolume = float(file.read()) #posisi volume
        file.close()
        print("Current Internal Volume", self.InternalVolume)
        file = open("/home/ubuntu/catkin_ws/src/hybrid-glider-system/gb_control/src/low_level_control/heave_actuator/scripts/Total.txt","r")
        self.TotalVolume = float(file.read()) #posisi volume
        file.close()
        
        self.Frequency = 2

        self.PrevVolume = 0.0
        self.DesiredInternalVolume = self.TotalVolume/2
        self.FlowDirection = 0 # 1 to Internal; -1 to External


        self.Enco = 0
        self.EncoTime = []
        self.EncoData = []
        self.EncoTotal = 0
        self.EncoCounter = 0
        self.RotationPerMinute = 0.0

        self.Time = time.time()
        self.LastTime = self.Time

        self.PublishInternalVolume = rospy.Publisher('/current_internal_volume', Float32, queue_size=10)
        self.PublishRotationPerMinute = rospy.Publisher('/motorBE_RPM', Float32, queue_size=10)

        # Setting for each pins
        ADC.setup()
        GPIO.setup(en_valveA, GPIO.OUT)
        GPIO.setup(en_valveB, GPIO.OUT)
        GPIO.setup(encoder_pin, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(pin_flow, GPIO.IN)

        time.sleep(0.1)
        PWM.start(en_BLDC, 5.0, 50)
        GPIO.output(en_valveA, GPIO.LOW)
        GPIO.output(en_valveB, GPIO.LOW)

        GPIO.add_event_detect(pin_flow, GPIO.FALLING, callback=self.counterflowmeter)
        GPIO.add_event_detect(encoder_pin, GPIO.FALLING, callback=self.counterencoder)

    def getdesiredvolume(self,data) :
        self.DesiredInternalVolume = data.data
        # if self.DesiredInternalVolume < 0.0 :
        #     self.DesiredInternalVolume = 0.0
        # elif self.DesiredInternalVolume > self.TotalVolume :
        #     self.DesiredInternalVolume = self.TotalVolume

    def writefile(self):
        file = open("/home/ubuntu/catkin_ws/src/hybrid-glider-system/gb_control/src/low_level_control/heave_actuator/scripts/Volume.txt","w")
        file.write(str(self.InternalVolume))
        file.close()

    def counterflowmeter(self, channel) :
        'this will be called when there is interrupt'
        if self.FlowDirection == 1 :
            self.InternalVolume += 2.5 #ml
        elif self.FlowDirection == -1 :
            self.InternalVolume -= 2.5 #ml

    def counterencoder(self, channel):
        self.Enco += 1

    def calculaterpm(self):
        self.EncoTime.append(time.time())
        self.EncoData.append(self.Enco)
        self.Enco = 0
        self.EncoTotal += self.EncoData[self.EncoCounter]

        if self.EncoCounter < self.Frequency :
            self.EncoCounter += 1
        else :
            self.EncoTotal -= self.EncoData[0]
            self.EncoData.remove(self.EncoData[0]) 
            self.EncoTime.remove(self.EncoTime[0]) 
        
        if (self.Frequency % 1 == 0) or (self.Frequency < 1):
            self.RotationPerMinute = 60.0 * self.EncoCounter * self.EncoTotal / self.Frequency
        else :
            if self.EncoCounter == 1 :
                self.RotationPerMinute = 60.0 * self.EncoTotal / self.Frequency
            else :
                DeltaTime = self.EncoTime[len(self.EncoTime)-1] - self.EncoTime[0]
                self.RotationPerMinute = 60.0 * (self.EncoTotal -self.EncoData[0]) / DeltaTime
        
        print("Rotation Per Minute : ", self.RotationPerMinute)
        self.publishrotationperminute()

    def calculatepressure(self) :
        self.ADCread = ADC.read("P9_40")
        return self.ADCread * 60.0

    def stopmotor(self) :
        PWM.set_duty_cycle(en_BLDC, 5.0)
        GPIO.output(en_valveA, GPIO.LOW)
        GPIO.output(en_valveB, GPIO.LOW)
        self.FlowDirection = 0

    def tointernal(self) :
        PWM.set_duty_cycle(en_BLDC, 6.8)
        GPIO.output(en_valveA, GPIO.HIGH)
        GPIO.output(en_valveB, GPIO.LOW)
        self.FlowDirection = 1

    def toeksternal(self) :
        PWM.set_duty_cycle(en_BLDC, 6.8)
        GPIO.output(en_valveA, GPIO.LOW)
        GPIO.output(en_valveB, GPIO.HIGH)
        self.FlowDirection = -1

    def publishinternalvolume(self) :
        Volume = Float32()
        Volume.data = self.InternalVolume
        self.PublishInternalVolume.publish(Volume)

    def publishrotationperminute(self) :
        Rpm = Float32()
        Rpm.data = self.RotationPerMinute
        self.PublishRotationPerMinute.publish(Rpm)

    def callibration(self) :
        self.PrevVolume = self.InternalVolume + 5.0
        self.TargetInternalVolume = self.InternalVolume - self.TotalVolume
        self.FlowDirection = -1
        print("Callibrating.....")
        print("Internal to Eksternal until Full")
        time.sleep(0.5)
        self.Enco = 0
        self.toeksternal()
        while self.PrevVolume - self.InternalVolume >  2.5 :
            self.PrevVolume = self.InternalVolume
            time.sleep(2)
            print("RPM Motor : ", self.Enco * 30)
            self.Enco = 0
            print("Internal Volume : ", self.InternalVolume)
            self.publishinternalvolume()
        self.stopmotor()
        print("The external bladder is full by oil")
        time.sleep(0.5)
        print("Setting Internal Volume to Mid Volume")
        self.InternalVolume = 0.0
        self.PrevVolume = 0.0
        self.TargetInternalVolume = self.TotalVolume/2
        self.LastTime = time.time()
        self.Enco = 0
        self.tointernal()
        print("Desired Internal Volume : ", self.TargetInternalVolume)
        while self.TargetInternalVolume-self.InternalVolume > 5.0 :
            self.Time = time.time()
            if self.Time-self.LastTime > 2.0 :
                print("RPM Motor : ", self.Enco * 30)
                self.Enco = 0
                print("Current Internal Volume : ", self.InternalVolume)
                self.publishinternalvolume()
                self.LastTime = self.Time
                if self.PrevVolume == self.InternalVolume :
                    print("Callibration FAILED")
                    print("There is no change in Internal Volume within 2 seconds")
                    break
                else :
                    self.PrevVolume = self.InternalVolume
            time.sleep(0.1)
        self.stopmotor()
        time.sleep(0.5)
        print("Exit Callibration")
        print("Current Internal Volume : ", self.InternalVolume)

    def loop(self) :
        self.calculaterpm()
        self.ErrorVolume = self.DesiredInternalVolume - self.InternalVolume
        Pressure = self.calculatepressure()
        print("Current Internal Volume : ", self.InternalVolume)
        self.publishinternalvolume()

        if Pressure < 30 :
            if self.ErrorVolume > 5.0 :
                self.tointernal()
                print("TO INTERNAL")
            elif self.ErrorVolume < -5.0 :
                self.toeksternal()
                print("TO EXTERNAL")
            else :
                self.stopmotor()
                print("Stop! DesiredInternalVolume reached")
        else :
            self.stopmotor()
            print("Stop! Pump Pressure is more than 30 Bar")

if __name__ == '__main__' :
    rospy.init_node('BuoyancyEngine')
    Buoy = BuoyClass()
    rate = rospy.Rate(Buoy.Frequency)
    Buoy.callibration()
    Buoy.Enco = 0
    while not rospy.is_shutdown():
        Buoy.loop()
        rate.sleep()

    PWM.stop(en_BLDC)
    Buoy.writefile()
    GPIO.output(en_valveA, GPIO.LOW)
    GPIO.output(en_valveB, GPIO.LOW)
    print("Node BE Mati, Valve LOW (closed)")