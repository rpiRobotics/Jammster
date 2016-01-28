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
import signal
from PID import PID
import argparse
import rospy
from sensor_msgs.msg import LaserScan


# TO USE
# send at least 4 commands a second or the chair will stop automatically
# the max speed is set to be small by default, change this with the
# set dutyMax command


address = 0x80
minDistance = 0
lastMessageTime = time.time()

# catch control c to allow for use of the same port
def signal_handler(signal, frame):
        print('Ctrl+C shutdown!')
        RRN.Shutdown()
        time.sleep(.4)
        roboclaw.DutyAccelM1(address,30000,0)
        roboclaw.DutyAccelM2(address,30000,0)
        sys.exit(0)

def shutdown():
        roboclaw.DutyAccelM1(address,30000,0)
        roboclaw.DutyAccelM2(address,30000,0)
        RRN.Shutdown()
        sys.exit(0)
    

class RoboClawState:
    """ test  """
    def __init__(self, imuConnectString):
        
        # go to default connect string if no string was specified
        if imuConnectString == None: 
            imuConnectString ='tcp://localhost:10001/arduinoIMU/arduinoIMUData' 
            
        self.m1Duty = 0
        self.m2Duty = 0
        #largest absolute value that the motors can be set to
        self.dutyMax = 5000
        
        # control mode will be either Duty or Velocity depending on last command sent
        self.controlMode = 'Duty'
        
        self.canMeasureWheelVelocities = True
        
        self.pidControllerR = PID(20,0,0,0,0,0,0)
        self.pidControllerL = PID(-20,0,0,0,0,0,0)
        
        # try to find and connect to NRF IMU server to get wheel velocities
        try:
            self.imuGateway = RRN.ConnectService(imuConnectString)
        except:
            print "Couldn't find NRF IMU server, unable to accept velocity commands!"
            self.canMeasureWheelVelocities = False
        self._lock = threading.RLock()
        
     
    def setDutyMax(self, newMax):          
        self.dutyMax = newMax

    # exposed to user through RR (implicitly sets to Duty mode)
    def setMDuties(self, leftDuty, rightDuty):
        print "received command"
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
    
    # used by velocity controller to write motor duties without setting to Duty mode or updating time
    def internalSetDuties(self, leftDuty, rightDuty):
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
        self.pidControllerL.setPoint(-leftV)
        self.pidControllerR.setPoint(rightV)


#********************************#
#             FUNCTIONS          #
#********************************#

def scanCallBack(data):
    global minDistance
    # distance is a list of distances sensed by the kinect
    distances = data.ranges
    distances = filter(lambda a: a > 0 and a<10,  distances)
    minDistance = min(distances)

def main():
    global minDistance
    rospy.init_node('kinect_avoid', anonymous = True) 
    rospy.Subscriber("/scan", LaserScan,  scanCallBack)  

    # accept arguments to command line call
    parser = argparse.ArgumentParser()
    parser.add_argument("--imuConnectString", help="the connect string of the IMU server")
    parser.add_argument("--serverPort", help="port to start this server on")
    parser.add_argument("--roboclawUSBPort", help="USB port that the roboclaw is on")
    args = parser.parse_args()
    connectString = args.imuConnectString
    port = args.serverPort
    roboclawPort = args.roboclawUSBPort
    
    if roboclawPort == None:
        roboclawPort = '/dev/ttyACM2'
        
    roboclaw.Open(roboclawPort,115200)
    
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName("roboClawController")
    RRN.RegisterTransport(t1)
    
    # go to default port if one was not specified by commandline
    if port == None:
        port = 10000
    print "Connect string: tcp://localhost:" + str(port) +"/roboClawController/wheelChairControl"
    t2 = RR.TcpTransport()
    t2.EnableNodeAnnounce()
    t2.StartServer(port)
    RRN.RegisterTransport(t2)

    with open('roboClaw_service.robodef', 'r') as f:
        service_def = f.read()
    
    myRoboClaw = RoboClawState(connectString)

    RRN.RegisterServiceType(service_def)
    RRN.RegisterService("wheelChairControl", "roboClawController.RoboClawState", myRoboClaw)

    # update duty cycle at 20Hz
    # stop wheelchair if a command hasnt been received in the last 1/4 second
    start = time.time()
    lastLeftSpeed = -1
    lastRightSpeed = -1
    # to track the number of times the last reading was repeated
    repeatCount = 0
    lastMeasurement = time.time()
    while 1:
        if time.time() - lastMessageTime > .250:
            myRoboClaw.m1Duty = 0
            myRoboClaw.m2Duty = 0
            roboclaw.DutyAccelM1(address,30000,0)
            roboclaw.DutyAccelM2(address,30000,0)
            continue
            
        if myRoboClaw.controlMode == 'Duty':
            roboclaw.DutyAccelM1(address,5000,int(myRoboClaw.m1Duty))
            roboclaw.DutyAccelM2(address,5000,int(myRoboClaw.m2Duty))
            
        if myRoboClaw.controlMode == 'Velocity':
            leftSpeed = 0
            rightSpeed = 0
            
            # if read fails, stop the wheelchair
            try:
                leftSpeed= myRoboClaw.imuGateway.IMU1_read()[5]
                rightSpeed = myRoboClaw.imuGateway.IMU2_read()[5]
            except:
                print "Couldn't read nrf server!"
                shutdown()
                
            # catch the case when the NRF server gives repeat values to prevent super fast wheelchair
            if leftSpeed == lastLeftSpeed or rightSpeed == lastRightSpeed:
                repeatCount+=1
                if repeatCount > 5:
                    print "Too many repeated values!"
                    shutdown()
            else:
                # dont go forward if there is something or update
                repeatCount = 0
                lastRightSpeed = rightSpeed
                lastLeftSpeed = leftSpeed
                dt = time.time() - lastMeasurement
                leftCommand = myRoboClaw.pidControllerL.update(leftSpeed,dt) + myRoboClaw.m1Duty
                rightCommand = myRoboClaw.pidControllerR.update(rightSpeed,dt) + myRoboClaw.m2Duty
                leftDesiredV = myRoboClaw.pidControllerL.getPoint()
                rightDesiredV = myRoboClaw.pidControllerR.getPoint()
                print "leftCommand: ", -leftDesiredV, "leftSpeed: ", -leftSpeed
                print "rightCommand: ", rightDesiredV, "rightSpeed: ", rightSpeed
                myRoboClaw.internalSetDuties(leftCommand, rightCommand)

                if minDistance < .7 and myRoboClaw.m1Duty > 0 and myRoboClaw.m2Duty > 0:
                    print "Something in the way, stopping"
                    roboclaw.DutyAccelM1(address,30000,0)
                    roboclaw.DutyAccelM2(address,30000,0)
                    myRoboClaw.pidControllerL.setPoint(0)
                    myRoboClaw.pidControllerR.setPoint(0)

                # update, we are totally ok
                else:
                    roboclaw.DutyAccelM1(address,5000,int(myRoboClaw.m1Duty))
                    roboclaw.DutyAccelM2(address,5000,int(myRoboClaw.m2Duty))
                    
                
        lastMeasurement = time.time()
        time.sleep(.05)
        
    RRN.Shutdown()
  
if __name__ == '__main__':
    main()

