#A role to enable the dribbling of the ball by Bot-BOT_ID, 
# --- from point Vector2D(A) to point Vector 2D(B)
# --- Parameters: 

from kubs import kubs, cmd_node
try:
	velocity.run = reload(velocity.run)
except:
	import velocity.run


print("Importing run: --- Imported")

from velocity.run_w import *
import rospy, sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from utils.geometry import Vector2D
from utils.config import *
from krssg_ssl_msgs.srv import *
from utils.functions import *
import math

#Step-1: Approach ball from distance: Only if farther than a threshold -- done before
#Step-2: Proper approach towards ball, 
# --- exactly behind on the mentioned line(min. tangential velocity between bot and ball)
#Step-3: Dribble on: Simply execute GoToPoint

kub = None
start_time = None
GOAL_POINT = None
APPROACH_POINT = None
FLAG_move = False
FLAG_turn = False
FLAG_dribble = False
rotate = 0
BALLPOS = None

#Avoiding rotation of the ball 

rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)
FIRST_CALL = True
vx_end,vy_end = 0,0

prev_state = None
try: 
	prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
	print("Error (in State Update) ", e)

def init(_kub, ballpos, pointB, theta_whiledribble):
	global kub, APPROACH_POINT, BALLPOS, GOAL_POINT, rotate, FLAG_turn, FLAG_dribble, FLAG_move, FIRST_CALL
	kub = _kub
	GOAL_POINT = Vector2D()
	APPROACH_POINT = Vector2D()
	BALLPOS = Vector2D()
	BALLPOS.x = ballpos.x
	BALLPOS.y = ballpos.y
	rotate = theta_whiledribble
	GOAL_POINT.x = pointB.x
	GOAL_POINT.y = pointB.y
	FLAG_move = False
	FLAG_dribble = False
	FLAG_turn = False
	FIRST_CALL = True

def reset():
	global start_time
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

def execute(startTime, DIST_THRESH, avoid_ball=False, factor=0.7):
	global kub, start_time, getState, BALLPOS, APPROACH_POINT, GOAL_POINT, rotate, FLAG_dribble, FLAG_turn, FLAG_move, FIRST_CALL, prev_state, vx_end, vy_end
	
	if FIRST_CALL:
		print("First Call of _Dribble_   ---")
		start_time = start_time
		FIRST_CALL = False
		vx_end, vy_end = 0,0
	
	#finding APPROACH_POINT:
	phi = kub.state.homePos[kub.kubs_id].theta
	APPROACH_POINT = getPointBehindTheBall(BALLPOS, phi, -4)

	if not FLAG_dribble:
		while not (FLAG_move):
			
			try:
				kub.state = getState(prev_state).stateB
			except:
				print("Error",e)
			
			if not(prev_state == kub.state):
				prev_state = kub.state

				t = rospy.Time.now()
				t = t.secs + 1.0*t.nsecs/pow(10,9)

				[vx, vy, vw, REPLANNED] = velocity.run.Get_Vel(start_time, t, kub.kubs_id, APPROACH_POINT, kub.state.homePos, kub.state.awayPos, avoid_ball)
				
				velocity_magnitude = Vector2D(vx,vy).abs(Vector2D(vx,vy))
				if velocity_magnitude > MAX_BOT_SPEED:
					angle_movement = math.atan2(vy,vx)
					# print("_____________Velocity Changed____________")
					vy = MAX_BOT_SPEED*math.sin(angle_movement)
					vx = MAX_BOT_SPEED*math.cos(angle_movement)
				
				if(REPLANNED):
					reset()
				
				if not vx and not vy:
					vx,vy = vx_end,vy_end
				else:
					vx_end,vy_end = vx,vy
				
				if dist(kub.state.homePos[kub.kubs_id], APPROACH_POINT) < BOT_RADIUS*factor :
					kub.move(0,0)
					# print("Distance completed"*200)
					FLAG_move = True
					FLAG_dribble = True
				else:
					# print("Sending velocity",vx,vy)
					kub.move(vx, vy)

				kub.execute()
				yield kub,APPROACH_POINT
	else:
		kub.dribbler(FLAG_dribble)
		kub.execute()
		FLAG_move = False 
		FLAG_turn = False
		while not (FLAG_move and FLAG_turn):
			
			try:
				kub.state = getState(prev_state).stateB
			except:
				print("Error",e)
			
			if not(prev_state == kub.state):
				prev_state = kub.state

				t = rospy.Time.now()
				t = t.secs + 1.0*t.nsecs/pow(10,9)

				[vx, vy, vw, REPLANNED] = velocity.run.Get_Vel(start_time, t, kub.kubs_id, GOAL_POINT, kub.state.homePos, kub.state.awayPos, False)
				
				velocity_magnitude = Vector2D(vx,vy).abs(Vector2D(vx,vy))
				if velocity_magnitude > MAX_BOT_SPEED:
					angle_movement = math.atan2(vy,vx)
					# print("_____________Velocity Changed____________")
					vy = MAX_BOT_SPEED*math.sin(angle_movement)
					vx = MAX_BOT_SPEED*math.cos(angle_movement)
				
				
				vw = Get_Omega(kub.kubs_id,rotate,kub.state.homePos)
			
				if not vw:
					vw = 0
					
				if(REPLANNED):
					reset()
				
				if not vx and not vy:
					vx,vy = vx_end,vy_end
				else:
					vx_end,vy_end = vx,vy

				if abs(normalize_angle(kub.state.homePos[kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
					kub.turn(0)
					# print("Angle completed")
					FLAG_turn = True
				else:
					kub.turn(vw)
				
				if dist(kub.state.homePos[kub.kubs_id], GOAL_POINT) < BOT_RADIUS*factor :
					kub.move(0,0)
					# print("Distance completed"*200)
					FLAG_move = True
				else:
					# print("Sending velocity",vx,vy)
					kub.move(vx, vy)

				kub.execute()
				yield kub,GOAL_POINT

	kub.dribble(FLAG_dribble)	
	kub.execute()

	yield kub,GOAL_POINT

