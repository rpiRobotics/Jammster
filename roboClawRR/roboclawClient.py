import time
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

def main():
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)
    

    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)

    myRoboClaw = RRN.ConnectService('tcp://localhost:57477/roboClawController/wheelChairControl')

    myRoboClaw.setM1(int(-200))
    myRoboClaw.setM2(int(-200))
    
    
    time.sleep(5)
    print "stopping"

    myRoboClaw.setM1(0)
    myRoboClaw.setM2(0)

    RRN.Shutdown()
  

if __name__ == '__main__':
    main()

