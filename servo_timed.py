from __future__ import division
import time
import array

import Adafruit_PCA9685

# Debugging logger
# import logging
# logging.basicConfig(level=logging.DEBUG)

pwm  = Adafruit_PCA9685.PCA9685()
freq = 50

servo = []

type  = 2

# Defining the servo database using tuples
servo = [
	( 700, 2100, 180), # A typical servo
	( 700, 2100, 150), # The mid-sized servo
	( 900, 2000, 270)  # ARS-3216HTG+HV
]

def servoTick(f, leng):
	timeTick  = (1000000 / 4096) / f   # Unit of the PWM timer
	numTick   =  int(leng / timeTick)
	print('Number of Ticks = ', numTick)
	return numTick

def servoAngleToTick(f, servoType, degree):
	degreePct = degree / servo[servoType][2]
	timeSpan  = servo[servoType][1] - servo[servoType][0]
	timePulse = timeSpan * degreePct + servo[servoType][0]
	print('timePulse = ', timePulse)
	return servoTick(f, timePulse)

pwm.set_pwm_freq(freq)

print('Servo Type 2 is on Ch 0, press Ctrl-C to quit...')
deg = 0
while(deg < servo[type][2]):
	pwm.set_pwm(0, 0, servoAngleToTick(freq, type, deg))
	time.sleep(0.5)
	deg = deg + 10


