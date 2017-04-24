#!/usr/bin/env python

import time, Rpi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
p1 = GPIO.PWM(17,100)	# channel=17 frequency=100hz
p1.start(0)

for dc in range (0,100,5):
	p1.ChangeDutyCycle(dc)
	time.sleep(1);
	
p1.stop()
GPIO.cleanup()	