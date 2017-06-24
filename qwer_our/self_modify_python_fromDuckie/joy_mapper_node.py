#!/usr/bin/env python
import rospy
import socket
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT
from sensor_msgs.msg import Joy
import time
from __builtin__ import True

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()


        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_joy_override = rospy.Publisher("~joystick_override", BoolStamped, queue_size=1)
        self.pub_parallel_autonomy = rospy.Publisher("~parallel_autonomy",BoolStamped, queue_size=1)
        self.pub_anti_instagram = rospy.Publisher("anti_instagram_node/click",BoolStamped, queue_size=1)
        self.pub_e_stop = rospy.Publisher("wheels_driver_node/emergency_stop",BoolStamped,queue_size=1)
        self.pub_avoidance = rospy.Publisher("~start_avoidance",BoolStamped,queue_size=1)
        self.pub_hello_stop = rospy.Publisher("wheels_driver_node/hello_stop", BoolStamped,queue_size=1)
        self.pub_goturnback = rospy.Publisher("wheels_driver_node/t_back",BoolStamped,queue_size=1)
		
        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        
        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        self.has_complained = False

        self.state_parallel_autonomy = False
        self.state_verbose = False

        pub_msg = BoolStamped()
        pub_msg.data = self.state_parallel_autonomy
        pub_msg.header.stamp = self.last_pub_time
        self.pub_parallel_autonomy.publish(pub_msg)

        #self.socket() # socket function
        self.socket_PhoneCtrl() # socket foe 4G course

    def cbParamTimer(self,event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = self.joy.header.stamp
        car_cmd_msg.v = self.joy.axes[1] * self.v_gain #Left stick V-axis. Up is positive
        if self.bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = self.joy.axes[3] * self.steer_angle_gain
            car_cmd_msg.omega = car_cmd_msg.v / self.simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
        self.pub_car_cmd.publish(car_cmd_msg)

    def socket(self):
        # socket begin
        servo = PWM(0x40)
	    servo.setPWMFreq(60)
        HOST = ''  # Symbolic name meaning all available interfaces
        PORT = 50007 # Arbitrary non-privileged port

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))
        rospy.loginfo('goingggggggggggg to Listen')
        s.listen(1)
        conn, addr = s.accept()
        rospy.loginfo('OKKKKKKKKKKKKK  I GET conn')
        while(1):
            data = conn.recv(1024)
            if data == '1' :
                rospy.loginfo('socket conn to open grip')
                servo.setPWM(1, 0, 350) #the best value(350) to open te grip
                time.sleep(0.3)
            elif data == '2' :
                rospy.loginfo('socket conn to close grip')
                servo.setPWM(1, 0, 510) #the best value(510) to grip can
                time.sleep(0.3)
            elif data == '3' :
                hello_msg = BoolStamped()
                rospy.loginfo('[%s] Now emergency_stop' % self.node_name)
                hello_msg.data = True
                self.pub_hello_stop.publish(hello_msg)
            elif data == '4' :
                rospy.loginfo('close socket conn')
                conn.close()
                break
        rospy.loginfo('exie socket')

    def socket_PhoneCtrl(self):
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.leftMotor = self.motorhat.getMotor(1)
        self.rightMotor = self.motorhat.getMotor(2)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rospy.loginfo("create socket succ!")
        sock.settimeout(20)    # if 20s it's no data received. it will interrupt
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)    #addr can reuse

        sock.bind(('', 50007))
        rospy.loginfo("bind socket succ!")
        sock.listen(3)    #maximum connect 3 clients
        rospy.loginfo("listen success!")
        while True:
            rospy.loginfo("listen for client...")
            (conn, ADDR) = sock.accept()
            rospy.loginfo("get client")
            rospy.loginfo(ADDR)
            #conn.settimeout(5)
            szBuf = conn.recv(1024)
            rospy.loginfo("recv:" + szBuf + "The command is")

            if szBuf == "1\n":
                rospy.loginfo("forward")
                self.leftMotor.setSpeed(100)
                self.rightMotor.setSpeed(100)
                leftMotor.run(Adafruit_MotorHAT.FORWARD)
                rightMotor.run(Adafruit_MotorHAT.FORWARD)
            elif szBuf == "2\n":
                rospy.loginfo("backward")
                self.leftMotor.setSpeed(100)
                self.rightMotor.setSpeed(100)
                leftMotor.run(Adafruit_MotorHAT.BACKWARD)
                rightMotor.run(Adafruit_MotorHAT.BACKWARD)
            elif szBuf == "3\n":
                rospy.loginfo("left")
                self.leftMotor.setSpeed(100)
                self.rightMotor.setSpeed(100)
                leftMotor.run(Adafruit_MotorHAT.BACKWARD)
                rightMotor.run(Adafruit_MotorHAT.FORWARD)
            elif szBuf =='4\n':
                rospy.loginfo("right")
                self.leftMotor.setSpeed(100)
                self.rightMotor.setSpeed(100)
                leftMotor.run(Adafruit_MotorHAT.FORWARD)
                rightMotor.run(Adafruit_MotorHAT.BACKWARD)
            elif szBuf == "5\n":
                hello_msg = BoolStamped()
                rospy.loginfo('[%s] Now emergency_stop' % self.node_name)
                hello_msg.data = True
                self.pub_hello_stop.publish(hello_msg)
                #conn.close()
                #break
            elif szBuf =='6\n':
                self.state_verbose ^= True
                rospy.loginfo('state_verbose = %s' % self.state_verbose)
                rospy.set_param('line_detector_node/verbose', self.state_verbose)

# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7,
# logitek = 8, left joy = 9, right joy = 10

    def processButtons(self, joy_msg):
        servo = PWM(0x40)
        servo.setPWMFreq(60)
        if (joy_msg.buttons[6] == 1): #The back button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = True
            self.pub_joy_override.publish(override_msg)
        elif (joy_msg.buttons[7] == 1): #the start button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = False
            self.pub_joy_override.publish(override_msg)
        elif (joy_msg.buttons[5] == 1): # Right back button
            self.state_verbose ^= True
            rospy.loginfo('state_verbose = %s' % self.state_verbose)
            rospy.set_param('line_detector_node/verbose', self.state_verbose) # bad - should be published for all to hear - not set a specific param

        elif (joy_msg.buttons[4] == 1): #Left back button
            self.state_parallel_autonomy ^= True
            rospy.loginfo('state_parallel_autonomy = %s' % self.state_parallel_autonomy)
            parallel_autonomy_msg = BoolStamped()
            parallel_autonomy_msg.header.stamp = self.joy.header.stamp
            parallel_autonomy_msg.data = self.state_parallel_autonomy
            self.pub_parallel_autonomy.publish(parallel_autonomy_msg)
        elif (joy_msg.buttons[3] == 1):
            anti_instagram_msg = BoolStamped()
            anti_instagram_msg.header.stamp = self.joy.header.stamp
            anti_instagram_msg.data = True
            self.pub_anti_instagram.publish(anti_instagram_msg)
        elif (joy_msg.buttons[8] == 1): #power button (middle)
            e_stop_msg = BoolStamped()
            e_stop_msg.header.stamp = self.joy.header.stamp
            e_stop_msg.data = True # note that this is toggle (actual value doesn't matter)
            self.pub_e_stop.publish(e_stop_msg)
        elif (joy_msg.buttons[9] == 1): #push left joystick button 
            avoidance_msg = BoolStamped()
            rospy.loginfo('start lane following with avoidance mode')
            avoidance_msg.header.stamp = self.joy.header.stamp
            avoidance_msg.data = True 
            self.pub_avoidance.publish(avoidance_msg)
		elif (joy_msg.buttons[0] == 1):
            rospy.loginfo('close the gripper')
            servo.setPWM(1, 0, 510) #the best value(510) to grip can
            time.sleep(0.3)
		elif (joy_msg.buttons[10] == 1):
            hello_msg = BoolStamped()
            rospy.loginfo('go go HelloRanger') #press the Right joy to e_stop (success stop by myself)
            hello_msg.data = True
            self.pub_hello_stop.publish(hello_msg)
		elif (joy_msg.buttons[1] == 1):
            t_back_msg = BoolStamped()
            rospy.loginfo('go go TurnBackRanger')
            t_back_msg.data = False
            self.pub_goturnback.publish(t_back_msg)

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('open the gripper')
                servo.setPWM(1, 0, 350)
                time.sleep(0.3)			
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))
				

if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
	#joy_mapper.socket()
    rospy.spin()
