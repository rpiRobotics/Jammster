import time
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

def main():
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)
    

    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
	
    # example connect string
    myRoboClaw = RRN.ConnectService('tcp://localhost:34314/roboClawController/wheelChairControl')


    # go forward for 3 seconds, send a command 20 times a second to prevent auto stop
    timeNow = time.time()
    while time.time() - timeNow < 10:
        myRoboClaw.setM1(int(10000))
        myRoboClaw.setM2(int(10000))
        time.sleep(.05)
    
    print "stopping"
    myRoboClaw.setM1(0)
    myRoboClaw.setM2(0)

    RRN.Shutdown()
  

if __name__ == '__main__':
    main()

