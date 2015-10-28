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


# TO USE
# send at least 4 commands a second or the chair will stop automatically
# the max speed is set to be small by default, change this with the
# set dutyMax command


address = 0x80
roboclaw.Open("/dev/ttyACM0",115200)
lastMessageTime = time.time()


class RoboClawState:
    """ test  """
    def __init__(self):
        self.m1Duty = 0
        self.m2Duty = 0
        #largest absolute value that the motors can be set to
        self.dutyMax = 180
        self._lock = threading.RLock()
        
     
    def setDutyMax(self, newMax):          
        self.dutyMax = newMax

    def setM1(self, newDuty):
	global lastMessageTime
	lastMessageTime = time.time()
        if abs(newDuty) > self.dutyMax:
            self.m1Duty = self.dutyMax * cmp(newDuty, 0)
            return

        else:
            self.m1Duty = newDuty

    def setM2(self, newDuty):
	global lastMessageTime
	lastMessageTime = time.time()
	if abs(newDuty) > self.dutyMax:
            self.m2Duty = self.dutyMax * cmp(newDuty, 0)
            return

        else:
            self.m2Duty = newDuty

    def writePulseWidths(self):
        roboclaw.DutyAccelM1(address,5000,int(self.m1Duty))
        roboclaw.DutyAccelM2(address,5000,int(self.m2Duty))



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
        if time.time() - lastMessageTime < .250:
            roboclaw.DutyAccelM1(address,5000,int(myRoboClaw.m1Duty))
            roboclaw.DutyAccelM2(address,5000,int(myRoboClaw.m2Duty))
        
        else:
            roboclaw.DutyAccelM1(address,30000,0)
            roboclaw.DutyAccelM2(address,30000,0)
        
        time.sleep(.05)

    RRN.Shutdown()
  

if __name__ == '__main__':
    main()

