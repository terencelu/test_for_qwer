import socket

import time
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor

mh = Adafruit_MotorHAT()

leftMotor = mh.getMotor(1)
rightMotor = mh.getMotor(2)

def go():
    leftMotor.setSpeed(100)
    rightMotor.setSpeed(100)

def stop():
    leftMotor.setSpeed(0)
    rightMotor.setSpeed(0)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("create socket succ!")
sock.settimeout(20)    # if 20s it's no data received. it will interrupt
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)    #addr can reuse

sock.bind(('', 50007))
print("bind socket succ!")
sock.listen(3)    #maximum connect 3 clients
print("listen success!")

#myStepper.step(100, Adafruit_MotorHAT.FORWARD,  Adafruit_MotorHAT.MICROSTEP)
while True:
    print("listen for client...")
    (conn, ADDR) = sock.accept()
    print("get client")
    print(ADDR)
    #conn.settimeout(5)
    szBuf = conn.recv(1024)
    print("recv:" + szBuf + "The command is")

    if szBuf == "1\n":
        print("forward")
        go()
        leftMotor.run(Adafruit_MotorHAT.FORWARD)
        rightMotor.run(Adafruit_MotorHAT.FORWARD)
    elif szBuf == "2\n":
        print("backward")
        go()
        leftMotor.run(Adafruit_MotorHAT.BACKWARD)
        rightMotor.run(Adafruit_MotorHAT.BACKWARD)
    elif szBuf == "3\n":
        print("left")
        go()
        leftMotor.run(Adafruit_MotorHAT.BACKWARD)
        rightMotor.run(Adafruit_MotorHAT.FORWARD)
    elif szBuf =='4\n':
        print("right")
        go()
        leftMotor.run(Adafruit_MotorHAT.FORWARD)
        rightMotor.run(Adafruit_MotorHAT.BACKWARD)
    elif szBuf == "5\n":
        print("U want to exit !? OK T_T")
        stop()
        conn.close()
        break
    else:
        print("STOP!!")
        stop()

    conn.close()
    print("end of service")
