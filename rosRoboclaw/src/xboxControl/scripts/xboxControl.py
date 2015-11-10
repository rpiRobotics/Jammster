#!/usr/bin/env python
import rospy
import math
import serial
import struct
import time
import sys
from sensor_msgs.msg import Joy
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

xJoy = 0
yJoy = 0
newData = False

def controlCall(data):
    global xJoy, yJoy, newData
    xJoy = data.axes[0]
    yJoy = data.axes[1]
    newData = True


def base_controller():
    global xJoy, yJoy, newData
    rospy.init_node('xboxControl', anonymous=True)
    rospy.Subscriber("/joy", Joy,  controlCall)
    
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)
    
    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
	
    myRoboClaw = RRN.ConnectService('tcp://catsTwist2.local:10000/roboClawController/wheelChairControl')
   #L = width of chair, R = radius of wheel
    L=.57
    R=.9424
    leftV = 0
    rightV = 0
    while True:
        
        #only send commands to move if we are out of the dead zone or if we have data
        if newData == True or abs(leftV) > 5 or abs(rightV) > 5:
            linearV = yJoy / 4
            angularV = xJoy / 2
            leftV =((2*linearV - angularV*L)/(2*R) ) *180
            rightV =((2*linearV + angularV*L)/(2*R) ) *180
            print leftV, rightV
            time.sleep(.01)
            myRoboClaw.setMVelocities(-leftV,rightV)
            newData = False
       

if __name__ == '__main__':
    base_controller()
