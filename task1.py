import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint
import multiprocessing
import threading
#from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
from tactics import Goalie 


#pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

def movebot(id,x,y,state,pub):
    corner=Vector2D(x, y)
    kub=kubs.kubs(id,state,pub)
    kub.update_state(state)
    g_fsm=GoToPoint.GoToPoint()
    # g_fsm = GoToPoint.GoToPoint()
    g_fsm.add_kub(kub)
    # g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
    g_fsm.add_point(corner)
    g_fsm.spin()

def topleft(nodeid):
    pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
    rospy.init_node('node'+str(nodeid),anonymous=False)

    while True:
    	state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("Error ",e)	
        if state:
            #print('lasknfcjscnajnstate',state.stateB.homePos)
            movebot(0,-2500,2500,state.stateB,pub)
            #print('chal ja')
            # break

def bottomleft(nodeid):
    pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
    rospy.init_node('node'+str(nodeid),anonymous=False)

    while True:
    	state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("Error ",e)	
        if state:
            #print('lasknfcjscnajnstate',state.stateB.homePos)
            movebot(3,-2500,-2500,state.stateB,pub)
            #print('chal ja')
            # break

def topright(nodeid):
    pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
    rospy.init_node('node'+str(nodeid),anonymous=False)

    while True:
    	state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("Error ",e)	
        if state:
            #print('lasknfcjscnajnstate',state.stateB.homePos)
            movebot(1,2500,2500,state.stateB,pub)
            #print('chal ja')
            # break

def bottomright(nodeid):
    pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
    rospy.init_node('node'+str(nodeid),anonymous=False)

    while True:
    	state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("Error ",e)	
        if state:
            #print('lasknfcjscnajnstate',state.stateB.homePos)
            movebot(2,2500,-2500,state.stateB,pub)
            #print('chal ja')
            # break

#multiprocessingGFG
bot0=multiprocessing.Process(target=topleft, args=(0,))
bot1=multiprocessing.Process(target=topright, args=(1,))
bot2=multiprocessing.Process(target=bottomright, args=(2,))
bot3=multiprocessing.Process(target=bottomleft, args=(3,))

#Start all the bot movement parallely via multiprocessing
bot0.start()
bot1.start()
bot2.start()
bot3.start()
#end all together
bot0.join()
bot1.join()
bot2.join()
bot3.join()



