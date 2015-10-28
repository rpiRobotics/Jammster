import time
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

def main():
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)
    

    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
	
    # example connect string
    myRoboClaw = RRN.ConnectService('tcp://localhost:54521/roboClawController/wheelChairControl')


    # go forward for 3 seconds, send a command 20 times a second to prevent auto stop
    timeNow = time.time()
    myRoboClaw.setDutyMax(4500)
    while time.time() - timeNow < 3:
        myRoboClaw.setM1(int(4500))
        myRoboClaw.setM2(int(4500))
        time.sleep(.05)
    
    print "stopping"
    myRoboClaw.setM1(0)
    myRoboClaw.setM2(0)

    RRN.Shutdown()
  

if __name__ == '__main__':
    main()

