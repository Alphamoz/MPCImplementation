#!/usr/bin/env python

import rospy
import time
import Adafruit_BBIO.GPIO as GPIO
import GS_timing as timing
from std_msgs.msg import Float64

output_ = Float64()

class LinMM:
    def __init__(self):
        self.enPin = "P9_11" #"P9_13" #OUTPUT
        self.stepPin = "P9_27" #"P9_15"  # OUTPUT
        self.dirPin = "P9_30" #"P9_17"  # OUTPUT

        self.S1Pin = "P8_11" #"P9_27"  # INPUT_PULLUP
        self.S2Pin = "P8_9" #"P9_30"  # INPUT_PULLUP
        self.S3Pin = "P8_17" #"P9_23"  # INPUT_PULLUP
        self.S4Pin = "P8_15"
        # self.ProxPin = "P9_26" # INPUT hitam P9_26 coklat 3.3V? biru GND
        self.indi = False

        GPIO.setup(self.stepPin, GPIO.OUT)
        GPIO.setup(self.dirPin, GPIO.OUT)
        GPIO.setup(self.enPin,GPIO.OUT)
        GPIO.setup("P9_13",GPIO.OUT)
        GPIO.setup("P9_15",GPIO.OUT)
        GPIO.setup("P9_18",GPIO.OUT)
        GPIO.output("P9_13",GPIO.LOW)
        GPIO.output("P9_15",GPIO.LOW)
        GPIO.output("P9_18",GPIO.HIGH)
        GPIO.output(self.enPin,GPIO.HIGH)
        GPIO.setup(self.S1Pin, GPIO.IN) #, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.S2Pin, GPIO.IN) # , pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.S3Pin, GPIO.IN) #, pull_up_down=GPIO.PUD_UP)
        # GPIO.setup(self.ProxPin,GPIO.IN)

        GPIO.add_event_detect(self.S1Pin, GPIO.FALLING, callback=self.back_switch_pressed)
        GPIO.add_event_detect(self.S2Pin, GPIO.FALLING, callback=self.front_switch_pressed)
        GPIO.add_event_detect(self.S3Pin, GPIO.FALLING, callback=self.front_switch_pressed)
        # GPIO.add_event_detect(self.ProxPin, GPIO.FALLING)

        try:
            file = open("/home/ubuntu/catkin_ws/src/learning_joy/memory/curPos.txt", "r")
            self.curPos = int(file.read())  # CurrrentPosition # ReadFromFile
            file.close()
            print("Ini Cur Pos")
            print(self.curPos)
        except:
            self.curPos = 0
        
        try:
            file = open("/home/ubuntu/catkin_ws/src/learning_joy/memory/NPos.txt", "r")
            self.NPos = int(file.read())  # Banyak Posisi # ReadFromFile
            file.close()
            print("Ini N Pos")
            print(self.NPos)
        except:
            self.NPos = 500 

        try:
            file = open("/home/ubuntu/catkin_ws/src/learning_joy/memory/NStep.txt", "r")
            self.NStep = int(file.read())  # JumlahStepUjungKeUjung # ReadFromFile
            file.close()
        except:
            self.NStep = 50881

        self.goalPos = int(self.NPos/2)  # GoalPosition
        self.delStep = int(self.NStep/self.NPos)  # JarakAntarPosisiDalamStep
        self.dus = 1 # delay microsecond (1 > BBB ~200 microsecond)
        self.Error = 0  # Error
        self.lastEr = 0  # LastError

        rospy.Subscriber("pitch_actuator_setpoint",
                         Float64, self.update_goalPos,queue_size = 1)
        
        self.pub_mm_position = rospy.Publisher('/mm_position', Float64, queue_size=10)

    def calibrate(self):
        print("calibrating ....")
        waktu = time.time()
        GPIO.output(self.enPin, GPIO.LOW)
        GPIO.output(self.dirPin, GPIO.LOW)
        if not GPIO.input(self.S1Pin) :
            self.indi == True
        while self.indi == False :
	    GPIO.output(self.stepPin, GPIO.HIGH)
	    timing.delayMicroseconds(self.dus)
	    GPIO.output(self.stepPin, GPIO.LOW)
	    timing.delayMicroseconds(self.dus)
	GPIO.output(self.enPin, GPIO.HIGH)
        # self.backward()
        print(time.time()-waktu)
        self.goalPos = int(self.NPos/2)

    def calculate_step(self):
        toBack = 0
        toFront = 0
        GPIO.output(self.enPin, GPIO.LOW)
        GPIO.output(self.dirPin, GPIO.HIGH)
        while not GPIO.event_detected(self.S3Pin):
            GPIO.output(self.stepPin, GPIO.HIGH)
            timing.delayMicroseconds(self.dus)
            GPIO.output(self.stepPin, GPIO.LOW)
            timing.delayMicroseconds(self.dus)
            toBack += 1
        timing.delay(1000)
        GPIO.output(self.dirPin, GPIO.LOW)
        while not (GPIO.event_detected(self.S1Pin) or GPIO.event_detected(self.S2Pin)):
            GPIO.output(self.stepPin, GPIO.HIGH)
            timing.delayMicroseconds(self.dus)
            GPIO.output(self.stepPin, GPIO.LOW)
            timing.delayMicroseconds(self.dus)
            toFront += 1
        GPIO.output(self.enPin, GPIO.HIGH)
        print("Banyak step ke belakang : ",toBack)
        print("Banyak step ke depan : ",toFront)
        if toFront == 0:
            print("Calculate step failed, calibrating...")
            self.calibrate()
        else:
            file = open("/home/ubuntu/catkin_ws/src/learning_joy/memory/NStep.txt", "w")
            file.write(str(toFront))
            file.close()
            self.NStep = toFront
            self.delStep = int(self.NStep/self.NPos)

    def update_goalPos(self, goal):
        if goal.data > self.NPos:
            self.goalPos = self.NPos
        elif goal.data < 0:
            self.goalPos = 0
        else:
            self.goalPos = goal.data

    def forward(self):
        GPIO.output(self.enPin, GPIO.LOW)
        GPIO.output(self.dirPin, GPIO.HIGH)
        for _ in range(self.delStep*int(abs(self.Error))):
            GPIO.output(self.stepPin, GPIO.HIGH)
            timing.delayMicroseconds(self.dus)
            GPIO.output(self.stepPin, GPIO.LOW)
            timing.delayMicroseconds(self.dus)
            if self.curPos == 0:
                self.curPos += 1
                break
        self.curPos -= 1*int(abs(self.Error))
        if self.curPos < 0 : 
	    self.curPos = 0
	#self.pub_mm_position.publish(output_)
        self.writeFile()
        GPIO.output(self.enPin, GPIO.HIGH)

    def backward(self):
        GPIO.output(self.enPin, GPIO.LOW)
        GPIO.output(self.dirPin, GPIO.LOW)
        for _ in range(self.delStep*int(abs(self.Error))):
            GPIO.output(self.stepPin, GPIO.HIGH)
            timing.delayMicroseconds(self.dus)
            GPIO.output(self.stepPin, GPIO.LOW)
            timing.delayMicroseconds(self.dus)
            if self.curPos == self.NPos:
                self.curPos -= 1
                break
        self.curPos += 1*int(abs(self.Error))
        if self.curPos > self.NPos :
            self.curPos = self.NPos
        #self.pub_mm_position.publish(output_)
        self.writeFile()
        GPIO.output(self.enPin, GPIO.HIGH)

    def writeFile(self):
        file = open("/home/ubuntu/catkin_ws/src/learning_joy/memory/curPos.txt", "w")
        file.write(str(self.curPos))
        #print("pos : ",self.curPos)
        #print("error : ",self.Error)
        file.close()

    def front_switch_pressed(self, channel):
        self.curPos = 0  # DiDepan
	print("o o depan")
        #self.goalPos = 1
        #self.indi = True

    def back_switch_pressed(self, channel):
        self.curPos = self.NPos  # DiBelakang
        print("o O o belakang")
        #self.goalPos = 19
        self.indi = True

    def loop(self):
        self.lastEr = self.Error
        self.Error = self.goalPos - self.curPos
        #print(abs(self.Error))
        print(self.goalPos)

        if self.Error > 0.5:
            self.backward()

        if self.Error < -0.5:
            self.forward()

        output_.data = self.curPos
        self.pub_mm_position.publish(output_)

        if GPIO.input(self.S1Pin):
            if self.curPos == 0:
                self.indi = False
                self.calibrate()

if __name__ == '__main__':
    try: 
        rospy.init_node('LinearMovingMass')
        MovMass = LinMM()
        # MovMass.calculate_step()
        #MovMass.calibrate()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            MovMass.loop()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    GPIO.cleanup()

