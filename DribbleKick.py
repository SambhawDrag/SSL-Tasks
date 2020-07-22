from enum import Enum
import behavior
import rospy
from role import GoToPoint, KickToPointP
import math
import kubs
from utils import math_functions
from utils.functions import *
from utils.state_functions import *
from utils.config import *
from velocity.run import *
from krssg_ssl_msgs.srv import *
import krssg_ssl_msgs.msg
import _turnAround_, _GoToPoint_

start_time = None
DIST_THRESH = 75


class DribbleKick(behavior.Behavior):
    """docstring for Dribble-and-Kick-into-Goal"""
    class State(Enum):
        
        normal = 1
        movetoball = 2
        align = 3
        dribble = 4
        set_stance = 5
        goal = 6
    
    def __init__(self,target):

        super(DribbleKick,self).__init__()

        self.name = "Dribble_Kick"

        self.goal_point = target

        self.go_at = None

        self.target_point = None

        self.power = 0        
        
        self.turn = False;

        self.goto_dribble = False;

        self.ready_to_goal = False;

        self.theta = 0

        self.behavior_failed = False

        #States:-

        self.add_state(DribbleKick.State.normal, behavior.Behavior.State.running)

        self.add_state(DribbleKick.State.movetoball, behavior.Behavior.State.running)

        self.add_state(DribbleKick.State.align, behavior.Behavior.State.running)
        
        self.add_state(DribbleKick.State.dribble, behavior.Behavior.State.running)
        
        self.add_state(DribbleKick.State.goal, behavior.Behavior.State.running)


        #Transition Functions:-
        self.add_transition(behavior.Behavior.State.start,
            DribbleKick.State.normal,lambda: True,'immediately')

        self.add_transition(DribbleKick.State.normal,
            DribbleKick.State.movetoball, lambda: self.reachball(),'far_away_from_ball')
        
        self.add_transition(DribbleKick.State.normal,
            DribbleKick.State.align, lambda: self.nearball(),'Close_to_ball_not_aligned.')
        
        self.add_transition(DribbleKick.State.movetoball,
            DribbleKick.State.align, lambda: self.nearball(),'Aligning_')
        
        self.add_transition(DribbleKick.State.align,
            DribbleKick.State.dribble, lambda: self.goto_dribble,'Dribbling_')
        
        self.add_transition(DribbleKick.State.dribble,
            DribbleKick.State.set_stance, lambda: self.at_DBox(), 'At D-Box_')

        self.add_transition(DribbleKick.State.set_stance,
            DribbleKick.State.goal, lambda: self.readytogoal(), 'Goal!!')

        self.add_transition(DribbleKick.State.goal,
			behavior.Behavior.State.completed, lambda: self.done(),'kicked!!!')
        
        #Behavior - Failed !!!
        self.add_transition(DribbleKick.State.movetoball,
            behavior.Behavior.State.failed, lambda: self.behavior_failed, 'failed')
        
        self.add_transition(DribbleKick.State.align,
            behavior.Behavior.State.failed, lambda: self.behavior_failed, 'failed')
            
        self.add_transition(DribbleKick.State.dribble,
            behavior.Behavior.State.failed, lambda: self.behavior_failed, 'failed')

        self.add_transition(DribbleKick.State.set_stance,
            behavior.Behavior.State.failed, lambda: self.behavior_failed, 'failed')
        
        self.add_transition(DribbleKick.State.goal,
            behavior.Behavior.State.failed, lambda: self.behavior_failed, 'failed')
        

    def add_kub(self,kub):
        self.kub = kub
    
    def add_theta(self,theta):
        self.theta = theta
    
    def get_pos_asvec2d(self, point2d):
        return Vector2D(int(point2d.x),int(point2d.y))
    
    def add_target_theta(self,theta):
        self.theta = theta

    def point_on_D_Box(self):
        return self.get_pos_asvec2d((abs(OUR_DBOX_X), 0))
    
    def bot_moving(self):
    	if abs(self.kub.state.homeVel[self.kub.kubs_id].x) != 0 or abs(self.kub.state.homeVel[self.kub.kubs_id].y) != 0:
			return True
        return False
    
    def kicking_power(self):
        return math.sqrt(dist(self.goal_point,self.kub.state.ballPos)/6400.0)*5.0 
    
#LAMBDA FUNCTIONS:-
    def reachball(self):
        ballpos = self.kub.state.ballPos
        theta = angle_diff(ballpos, self.point_on_D_Box())
        go_at = getPointBehindTheBall(ballpos, theta, -2)
        return go_at.dist(self.get_pos_as_vec2d(self.kub.get_pos())) >= DISTANCE_THRESH*0.25

    def nearball(self):
        self.turn = True
        return not self.reachball()

    def at_DBox(self):
        target = self.point_on_D_Box()
        curPos = self.kub.get_pos()
        return target.dist(self.get_pos_asvec2d(curPos)) <= DISTANCE_THRESH*0.1
    
    def readytogoal(self):
        bot_theta = self.kub.get_theta()
        ballpos = self.get_pos_asvec2d(self.kub.state.ballPos)
        totalAngle = angle_diff(ballpos, self.goal_point)

        deltheta = totalAngle-normalize_angle(bot_theta)
        modtheta = min(abs(deltheta),abs((2*math.pi)-deltheta))  #shortest turn
        sign = (normalize_angle(deltheta))/(abs(normalize_angle(deltheta))) 
        
        theta_left = sign * modtheta

        return self.ready_to_goal and (abs(theta_left) < ROTATION_FACTOR/10)
    
    def done(self):
        ballpos = self.get_pos_asvec2d(self.kub.state.ballPos)
        condition = (ballpos.x >= 4500) and (ballpos.y > -300) and (ballpos.y < 300)
        return condition
    
# STATE FUNCTIONS:-
    def on_enter_normal(self):
        pass

    def execute_normal(self):
        pass

    def on_exit_normal(self):
        pass

    def on_enter_movetoball(self):
        print("Entered movetoball")
        self.theta = normalize_angle(angle_diff(self.kub.state.ballPos,self.point_on_D_Box()))
        self.power = 0 
        self.go_at = getPointBehindTheBall(self.kub.state.ballPos, self.theta, -2)
        _GoToPoint_.init(self.kub, self.go_at, self.theta)

    def execute_movetoball(self):
    	
		print("Executing approach ball ... ")
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)  
		print "----------------------------------------------------------------"

		generatingfunction = _GoToPoint_.execute(start_time, DRIBBLER_BALL_THRESH*0.25, True)
		
		for gf in generatingfunction:
			self.kub,target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			theta = angle_diff(self.kub.state.ballPos, self.point_on_D_Box())
			print(self.theta)
			self.kub.kick(self.power)
			#print theta,"------------------------"
			self.go_at = getPointBehindTheBall(self.kub.state.ballPos, theta, -2)
			# #print vicinity_points(self.go_at,target_point,thresh=BOT_BALL_THRESH)
			# if(dist(self.go_at,self.kub.get_pos())<BOT_BALL_THRESH):
			# 	self.behavior_failed = True
			# # 	break
			if not vicinity_points(self.go_at, target_point,thresh=BOT_RADIUS*2.0):
				print "Finally!"
				# self.behavior_failed = True 
				break
			#print self.go_at.x,"1",self.go_at.y,"abc",self.kub.get_pos().x,"2",self.kub.get_pos().y
		print("Reached the ball. Start dribble ... ")

    def on_exit_movetoball(self):
        print("I have reached the ball")
        self.turn = True;
        pass

    def on_enter_align(self):
        print("Entering align: ", self.turn)
        ballpos = self.kub.state.ballPos
        self.theta = angle_diff(ballpos, self.point_on_D_Box())
        self.go_at = getPointBehindTheBall(ballpos, self.theta, -2)
        pass

    def execute_align(self):
        
        READY_TO_DRIBBLE = self.goto_dribble
        theta = self.theta

        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)

        prev_state = None

        while not READY_TO_DRIBBLE: 

            try:
                self.kub.state = getState(prev_state).stateB
            except rospy.ServiceException, e:
                print("Error :: ROS ", e)
            # print(kub.state)
            self.kub.update_state(prev_state)
            if not(prev_state == self.kub.state):
                prev_state = self.kub.state
        
                print(self.kub.get_theta())
                print("Target Alignment : ",theta)
                totalAngle = theta
                MAX_w = (MAX_BOT_OMEGA+MIN_BOT_OMEGA)/1.2
                MIN_w = (MIN_BOT_OMEGA)

                # theta2 = totalAngle :-- target angular alignment ; theta1 = current_bot_theta; New x-axis: Bot_theta_line
                # rotating standard axes
                deltheta = totalAngle-normalize_angle(self.kub.get_theta()) 
                modtheta = min(abs(deltheta),abs((2*math.pi)-deltheta))  #shortest turn
                sign = (normalize_angle(deltheta))/(abs(normalize_angle(deltheta))) 

                print "Remaining angle: ",modtheta,"   ",sign
                
                theta_lft = modtheta * sign  

                if abs(theta_lft)<ROTATION_FACTOR/10:
                    vw = 0.0
                    READY_TO_DRIBBLE=True
                else:
                    READY_TO_DRIBBLE=False
                    vw = 3.0*(theta_lft)/(math.pi)*MAX_w
                    #vw = (theta_lft/2*math.pi)*MAX_w

			self.kub.kick(self.power)
			#print theta,"------------------------"
			self.go_at = getPointBehindTheBall(self.kub.state.ballPos, theta, -2)
			# #print vicinity_points(self.go_at,target_point,thresh=BOT_BALL_THRESH)
			# if(dist(self.go_at,self.kub.get_pos())<BOT_BALL_THRESH):
			# 	self.behavior_failed = True
			# # 	break
			if not vicinity_points(self.go_at, target_point,thresh=BOT_RADIUS*2.0):
				print "Finally!"
				# self.b
                if abs(vw)<1*MIN_w and READY_TO_DRIBBLE==False:
                    vw = 1*MIN_w*(1 if vw>0 else -1)
                
                if abs(vw) > MAX_w and READY_TO_DRIBBLE==False:
                        vw = (vw/abs(vw))*MAX_w

                if vw!=0:
                    print("TURNING")
                    #print("vw Hoon Main =",vw)
                    #print("BOT THETA:",data.homePos[BOT_ID].theta)
                    print "\nAngle Remaining : ",theta_lft
                else:
                    print("DONE !!")
                    
                print "Omega Return: ",vw
                print READY_TO_DRIBBLE
                print "___________________________________________________________________"
                self.kub.reset()
                self.kub.turn(vw)
                self.kub.execute()
    
    def on_exit_align(self):
        self.goto_dribble = True; #Now, Ready to dribble after aligning
        self.turn = False;
        pass

    def on_enter_dribble(self):
        print("Enter dribble")
        self.kub.dribble = True;
        self.go_at = self.point_on_D_Box()
        self.theta = normalize_angle(angle_diff(self.point_on_D_Box(), self.get_pos_asvec2d(self.goal_point)))
        self.kub.kick = 0
        _GoToPoint_.init(self.kub, self.go_at, self.theta)
        pass

    def execute_dribble(self):

        print("Executing dribble ... ")
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)  
        print "----------------------------------------------------------------"
        
        generatingfunction = _GoToPoint_.execute(start_time, DRIBBLER_BALL_THRESH*0.25, True)
		
        for gf in generatingfunction:
			self.kub,target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			theta = angle_diff(self.kub.state.ballPos, self.point_on_D_Box())
			print(self.theta)
			self.kub.kick(self.power)
			#print theta,"------------------------"
			self.go_at = getPointBehindTheBall(self.kub.state.ballPos, theta, -2)
			# #print vicinity_points(self.go_at,target_point,thresh=BOT_BALL_THRESH)
			# if(dist(self.go_at,self.kub.get_pos())<BOT_BALL_THRESH):
			# 	self.behavior_failed = True
			# # 	break
			if not vicinity_points(self.go_at, target_point,thresh=BOT_RADIUS*2.0):
				print "Finally!"
				# self.behavior_failed = True 
				break
			#print self.go_at.x,"1",self.go_at.y,"abc",self.kub.get_pos().x,"2",self.kub.get_pos().y
        print("Reached the D_Box. Prepare to Goal ...")
        pass

    def on_exit_dribble(self):
        print("Dribble executed successfully, now goal. ----------")
        self.kub.dribble = False;
        self.goto_dribble = False;
        self.go_at=None
        self.ready_to_goal = True;
        pass

    def on_enter_setstance(self):
        #check if at d-box
        self.turn = True;
        pass

    def execute_setstance(self):
        pass

    def exit_setstance(self):
        pass

    def on_enter_goal(self):
        print("Enter goal ...")
        self.power=self.kicking_power()
        KickToPointP.init(self.goal_point)
        pass

    def execute_goal(self):
        print("Goaling ... ")
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)  
        print "----------------------------------------------------------------"

        self.kub.reset()
        self.kub.kick(self.power)
        self.execute()

    def on_exit_goal(self):
        self.kub.reset()
        print("GOALLL !!!")
        print("EXITING ROLE")
        pass