#!/usr/bin/env python
import RobotRaconteur as RR
import threading
RRN = RR.RobotRaconteurNode.s
import time
import traceback
import socket
import time
import sys
import serial
import struct
import roboclaw
from PID import PID


# TO USE
# send at least 4 commands a second or the chair will stop automatically
# the max speed is set to be small by default, change this with the
# set dutyMax command


address = 0x80
roboclaw.Open("/dev/ttyACM1",115200)
lastMessageTime = time.time()


class RoboClawState:
    """ test  """
    def __init__(self):
        self.m1Duty = 0
        self.m2Duty = 0
        #largest absolute value that the motors can be set to
        self.dutyMax = 10000
        
        # control mode will be either Duty or Velocity depending on last command sent
        self.controlMode = 'Duty'
        
        self.canMeasureWheelVelocities = True
        
        self.pidControllerR = PID(-50,0,0,0,0,0,0)
        self.pidControllerL = PID(-50,0,0,0,0,0,0)
        
        # try to find and connect to NRF IMU server to get wheel velocities
        try:
            self.imuGateway = RRN.ConnectService('tcp://localhost:39085/arduinoIMU/arduinoIMUData')
        except:
            print "Couldn't find NRF IMU server, unable to accept velocity commands!"
            self.canMeasureWheelVelocities = False
        self._lock = threading.RLock()
        
     
    def setDutyMax(self, newMax):          
        self.dutyMax = newMax

    def setMDuties(self, leftDuty, rightDuty):
        self.controlMode = 'Duty'
        global lastMessageTime
        lastMessageTime = time.time()
        if abs(leftDuty) > self.dutyMax:
            self.m1Duty = self.dutyMax * cmp(leftDuty, 0)
        else:
            self.m1Duty = leftDuty

    	if abs(rightDuty) > self.dutyMax:
            self.m2Duty = self.dutyMax * cmp(rightDuty, 0)
        else:
            self.m2Duty = rightDuty
            
    def setMVelocities(self, leftV,rightV):
        self.controlMode = 'Velocity'
        global lastMessageTime
        lastMessageTime = time.time()
        #print leftV,rightV
        self.pidControllerL.setPoint(leftV)
        self.pidControllerR.setPoint(rightV)


#********************************#
#             FUNCTIONS          #
#********************************#


def get_open_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()
    sock.close()
    time.sleep(3)
    return port[1]


def main():
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName("roboClawController")
    RRN.RegisterTransport(t1)
    

    port = get_open_port()
    print "Connect string: tcp://localhost:" + str(port) +"/roboClawController/wheelChairControl"
    t2 = RR.TcpTransport()
    t2.EnableNodeAnnounce()
    t2.StartServer(port)
    RRN.RegisterTransport(t2)

    with open('roboClaw_service.robodef', 'r') as f:
        service_def = f.read()
    
    myRoboClaw = RoboClawState()

    RRN.RegisterServiceType(service_def)
    RRN.RegisterService("wheelChairControl", "roboClawController.RoboClawState", myRoboClaw)

    # update duty cycle at 20Hz
    # stop wheelchair if a command hasnt been received in the last 1/4 second
    while 1:
    
        if time.time() - lastMessageTime > .250:
            roboclaw.DutyAccelM1(address,30000,0)
            roboclaw.DutyAccelM2(address,30000,0)
            continue
            
        if myRoboClaw.controlMode == 'Duty':
            roboclaw.DutyAccelM1(address,5000,int(myRoboClaw.m1Duty))
            roboclaw.DutyAccelM2(address,5000,int(myRoboClaw.m2Duty))
            
        if myRoboClaw.controlMode == 'Velocity':
            leftSpeed= myRoboClaw.imuGateway.IMU1_read()[5]
            rightSpeed = myRoboClaw.imuGateway.IMU2_read()[5]
            leftCommand = myRoboClaw.pidControllerL.update(leftSpeed) + myRoboClaw.m1Duty 
            rightCommand = myRoboClaw.pidControllerR.update(rightSpeed) + myRoboClaw.m2Duty 
            #print leftCommand
            print rightCommand, rightSpeed
            myRoboClaw.setMDuties(leftCommand, rightCommand)
            roboclaw.DutyAccelM1(address,5000,int(myRoboClaw.m1Duty))
            roboclaw.DutyAccelM2(address,5000,int(myRoboClaw.m2Duty))
            
        
        time.sleep(.05)

    RRN.Shutdown()
  

if __name__ == '__main__':
    main()

