#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

minDistance = 0

def scanCallBack(data):
    # distance is a list of distances sensed by the kinect
    distances = data.ranges
    distances = filter(lambda a: a > 0 and a<10,  distances)
    minDistance = min(distances)
    print minDistance

def avoid():
    rospy.init_node('kinect_avoid', anonymous = True)  
    rospy.Subscriber("/scan", LaserScan,  scanCallBack)
    raw_input("press enter to exit")

if __name__ == '__main__':                                                           
    avoid()
