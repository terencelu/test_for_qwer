#!/usr/bin/env python

import time, Rpi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
p1 = GPIO.PWM(18,100)	# channel=17 frequency=100hz
p1.start(5)

for dc in range (5,25,1):
	p1.ChangeDutyCycle(dc)
	time.sleep(0.5);
	
p1.stop()
GPIO.cleanup()	
