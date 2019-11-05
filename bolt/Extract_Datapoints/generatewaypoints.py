#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import pickle # For reading and writing into binary files
import sys # for sys.exit
import signal # Signal Interrupt Handler
import os
 

class Pixie:
    def __init__(self):
        rospy.init_node('NovatelData', anonymous=True)
        print("in init")
        self.pixiedata=rospy.Subscriber("/navsat/odom", Odometry,self.callback)
        self.distance_between_waypoints=0.5 # in metres
        self.x=0
        self.y=0
        self.prevx=0
        self.prevy=0
        self.ct=0
        self.List=[]
        signal.signal(signal.SIGINT, self.signalInterruptHandler)
    #self.data = Poinr


    def sufficiently_wide(self, x1,y1,x2,y2):
        if (x2-x1)**2 + (y2-y1)**2 > self.distance_between_waypoints:
            return True



    def callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.ct==0:
            
            self.ct=1
            #temp=Float32MultiArray(data=[self.x,self.y])
            temp=[self.x,self.y]
            self.List.append(temp)
        
        else:
            
            if (self.sufficiently_wide(self.x,self.y,self.prevx,self.prevy)):
                #temp=Float32MultiArray(data=[self.x,self.y])
                temp=[self.x,self.y]
                print("Point = ", temp )
                self.List.append(temp)
                self.prevx=self.x
                self.prevy=self.y
        
    
    def signalInterruptHandler(self,signum,frame):
        with open('GeneratedRoute.data', 'wb') as filehandle:
            pickle.dump(self.List,filehandle)

        rospy.signal_shutdown('Ctrl+C caught... writing binary file')
        # Let's try this out and see if this works
        #os.system("python publishwaypoints.py")
        sys.exit()   

if __name__=="__main__":
    a =Pixie()
    rospy.spin()

