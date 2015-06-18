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


# TO USE
# send at least 4 commands a second or the chair will stop automatically
# the max speed is set to be small by default, change this with the
# set dutyMax command


port = serial.Serial("/dev/ttyACM0", baudrate=38400, timeout=.5)
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
        SetM1DutyAccel(200,  int(self.m1Duty))
        SetM2DutyAccel(200,  int(self.m2Duty))




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

def writebyte(val):
    global checksum
    checksum += val
    return port.write(struct.pack('>B',val));
def writesbyte(val):
    global checksum
    checksum += val
    return port.write(struct.pack('>b',val));
def writeword(val):
    global checksum
    checksum += val
    checksum += (val>>8)&0xFF
    return port.write(struct.pack('>H',val));
def writesword(val):
    global checksum
    checksum += val
    checksum += (val>>8)&0xFF
    return port.write(struct.pack('>h',val));
def writelong(val):
    global checksum
    checksum += val
    checksum += (val>>8)&0xFF
    checksum += (val>>16)&0xFF
    checksum += (val>>24)&0xFF
    return port.write(struct.pack('>L',val));
def writeslong(val):
    global checksum
    checksum += val
    checksum += (val>>8)&0xFF
    checksum += (val>>16)&0xFF
    checksum += (val>>24)&0xFF
    return port.write(struct.pack('>l',val));

def sendcommand(address,command):
    global checksum
    checksum = address
    port.write(chr(address));
    checksum += command
    port.write(chr(command));
    return;

def SetM1DutyAccel(accel,duty):
    sendcommand(128,52)
    writesword(duty)
    writeword(accel)
    writebyte(checksum&0x7F);
    return

def SetM2DutyAccel(accel,duty):
    sendcommand(128,53)
    writesword(duty)
    writeword(accel)
    writebyte(checksum&0x7F);
    return;



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

    #update duty cycle at 20Hz
    while 1:
	if time.time() - lastMessageTime < .250:
            SetM1DutyAccel(200,  int(myRoboClaw.m1Duty))
            SetM2DutyAccel(200,  int(myRoboClaw.m2Duty))
        
        else:
	    SetM1DutyAccel(200,  int(0))
            SetM2DutyAccel(200,  int(0))
        time.sleep(.05)

    RRN.Shutdown()
  

if __name__ == '__main__':
    main()

