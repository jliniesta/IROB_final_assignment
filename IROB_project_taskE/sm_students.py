#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):
        
        self.node_name = "Student SM"
        self.cube_PoseStamped = PoseStamped()

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        
        self.pick_up_srv_nm = rospy.get_param('manipulation_client/pick_srv')
        self.place_srv_nm = rospy.get_param('manipulation_client/place_srv')
        self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        rospy.loginfo("%s: pick srv name = %s", self.node_name, self.pick_srv_nm)
        rospy.loginfo("%s: place srv name = %s", self.node_name, self.place_srv_nm)

        # Subscribe to topics

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(self.pick_up_srv_nm, timeout=30)
        rospy.wait_for_service(self.place_srv_nm, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)


        # rospy.loginfo("%s: Waiting for pickup action server...", self.node_name)

        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.check_states()


    def check_states(self):

        while not rospy.is_shutdown() and self.state != 4:
            
            # State 1:  Tuck arm 
            if self.state == 0:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)

                # Send tucking goal to the robot 
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)

                # Wait for success tucking
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

                if success_tucking:
                    rospy.loginfo("%s: Arm tuck: ", self.play_motion_ac.get_result())
                    self.state = 1
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = 5

                rospy.sleep(1)

            # State 2:  Pick up
            if self.state == 1:
            	try:
                    rospy.loginfo("%s: Trying to pick up the object", self.node_name)

                    # Initialice the service
                    pick_up_srv = rospy.ServiceProxy(self.pick_up_srv_nm, SetBool)
                    # Request to the service
                    pick_up_req = pick_up_srv()
                    
                    # Boolean to see if the pick up is succeeded or not
                    success_pick_up = pick_up_req.success

                    if success_pick_up == True:
                        self.state = 2
                        rospy.loginfo("%s: Object pick up succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Object pick up failed!", self.node_name)
                        self.state = 5

                    rospy.sleep(3)
                
                except rospy.ServiceException, e:
                    print("Service call to pick_up server failed: %s"%e)

            # State 3:  Carry cube to second table
            if self.state == 2:

                rospy.loginfo("%s: Moving towards the second table", self.node_name)

                # Obtain message with the pose of table2
                table2_pose = rospy.wait_for_message(self.place_pose_top, PoseStamped, timeout=5)

                # Send movement goal (table2) to robot base
                goal = MoveBaseGoal()
                goal.target_pose = table2_pose
                self.move_base_ac.send_goal(goal)
                
                # Wait for success navigation
                success_navigation = self.move_base_ac.wait_for_result(rospy.Duration(100.0))

                if success_navigation == True:
                    rospy.loginfo("%s: Navigation to table 2 completed!", self.node_name)
                    self.state = 3
                else: 
                    rospy.loginfo("%s: Navigation to table2 failed!", self.node_name)
                    self.state = 5

                rospy.sleep(1)

            # State 4:  Place object to second table
            if self.state == 3:
                try:
                    rospy.loginfo("%s: Trying to place the object", self.node_name)

                    # Initialice the service
                    place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
                    # Request to the service                 
                    place_req = place_srv()
                    
                    # Boolean to see if the object place is succeeded or not
                    success_place = place_req.success

                    if success_place == True:
                        self.state = 4
                        rospy.loginfo("%s: Object place succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Object place failed!", self.node_name)
                        self.state = 5

                    rospy.sleep(3)
                
                except rospy.ServiceException, e:
                    print("Service call to place server failed: %s"%e)

            # Error handling
            if self.state == 5:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


# import py_trees as pt, py_trees_ros as ptr

# class BehaviourTree(ptr.trees.BehaviourTree):

# 	def __init__(self):

# 		rospy.loginfo("Initialising behaviour tree")

# 		# go to door until at door
# 		b0 = pt.composites.Selector(
# 			name="Go to door fallback", 
# 			children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
# 		)

# 		# tuck the arm
# 		b1 = TuckArm()

# 		# go to table
# 		b2 = pt.composites.Selector(
# 			name="Go to table fallback",
# 			children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
# 		)

# 		# move to chair
# 		b3 = pt.composites.Selector(
# 			name="Go to chair fallback",
# 			children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
# 		)

# 		# lower head
# 		b4 = LowerHead()

# 		# become the tree
# 		tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
# 		super(BehaviourTree, self).__init__(tree)

# 		# execute the behaviour tree
# 		self.setup(timeout=10000)
# 		while not rospy.is_shutdown(): self.tick_tock(1)


# class Counter(pt.behaviour.Behaviour):

# 	def __init__(self, n, name):

# 		# counter
# 		self.i = 0
# 		self.n = n

# 		# become a behaviour
# 		super(Counter, self).__init__(name)

# 	def update(self):

# 		# count until n
# 		while self.i <= self.n:

# 			# increment count
# 			self.i += 1

# 			# return failure :(
# 			return pt.common.Status.FAILURE

# 		# succeed after counter done :)
# 		return pt.common.Status.SUCCESS


# class Go(pt.behaviour.Behaviour):

# 	def __init__(self, name, linear, angular):

# 		# action space
# 		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
# 		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

# 		# command
# 		self.move_msg = Twist()
# 		self.move_msg.linear.x = linear
# 		self.move_msg.angular.z = angular

# 		# become a behaviour
# 		super(Go, self).__init__(name)

# 	def update(self):

# 		# send the message
# 		rate = rospy.Rate(10)
# 		self.cmd_vel_pub.publish(self.move_msg)
# 		rate.sleep()

# 		# tell the tree that you're running
# 		return pt.common.Status.RUNNING


# class TuckArm(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# Set up action client
# 		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

# 		# personal goal setting
# 		self.goal = PlayMotionGoal()
# 		self.goal.motion_name = 'home'
# 		self.goal.skip_planning = True

# 		# execution checker
# 		self.sent_goal = False
# 		self.finished = False

# 		# become a behaviour
# 		super(TuckArm, self).__init__("Tuck arm!")

# 	def update(self):

# 		# already tucked the arm
# 		if self.finished: 
# 			return pt.common.Status.SUCCESS
		
# 		# command to tuck arm if haven't already
# 		elif not self.sent_goal:

# 			# send the goal
# 			self.play_motion_ac.send_goal(self.goal)
# 			self.sent_goal = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# if I was succesful! :)))))))))
# 		elif self.play_motion_ac.get_result():

# 			# than I'm finished!
# 			self.finished = True
# 			return pt.common.Status.SUCCESS

# 		# if I'm still trying :|
# 		else:
# 			return pt.common.Status.RUNNING
		


# class LowerHead(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# server
# 		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
# 		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
# 		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

# 		# execution checker
# 		self.tried = False
# 		self.tucked = False

# 		# become a behaviour
# 		super(LowerHead, self).__init__("Lower head!")

# 	def update(self):

# 		# try to tuck head if haven't already
# 		if not self.tried:

# 			# command
# 			self.move_head_req = self.move_head_srv("down")
# 			self.tried = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# react to outcome
# 		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE


	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
