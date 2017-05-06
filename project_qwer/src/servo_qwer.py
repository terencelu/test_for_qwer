# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# for find file
import os

# Import the PCA9685 module.
import Adafruit_PCA9685

#Gesture
gesture = 'down'

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

#The gripper widths of opening maxium and minium
grip_min = 640	# the gripper will colse in this pulse
grip_max = 350	# the gripper will open to max width in this pulse

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def pwm_control():
	# Move servo on channel 1 between extremes.
	pwm.set_pwm(1, 0, grip_max)
	time.sleep(1)
	pwm.set_pwm(1, 0, grip_min)
	time.sleep(1)

def openfile():
	os.system('bash ~/duckietown/test.sh')
		
print('Moving servo on channel 0, press Ctrl-C to quit...')
if __name__ == "__main__":
	# Set frequency to 60hz, good for servos.
	pwm.set_pwm_freq(60)
	print('try to roslaunch')
#	launch = openfile()
	for x in range(0,5,1):
		pwm_control()


#	if gesture == 'up':
#		pwm.set_pwm(1, 0, 150)
#		time.sleep(2)
#	elif gesture == 'down':
#		pwm_control()
	
	print('end')
	
	
	
