import time
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

def main():
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)
    

    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
	
    # example connect string
    myRoboClaw = RRN.ConnectService('tcp://localhost:10000/roboClawController/wheelChairControl')


    # go forward for 3 seconds, send a command 20 times a second to prevent auto stop
    timeNow = time.time()
    myRoboClaw.setDutyMax(5000)
    while time.time() - timeNow < 10:
        #myRoboClaw.setMDuties(int(8500),int(10000))
        myRoboClaw.setMVelocities(40,40)
        time.sleep(.02)
    
    print "stopping"

    RRN.Shutdown()
  

if __name__ == '__main__':
    main()

