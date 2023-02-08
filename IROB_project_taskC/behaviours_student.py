# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist, PoseStamped
from actionlib import SimpleActionClient, TerminalState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class cube_localized(pt.behaviour.Behaviour):

    """
    Check if cube is localized.
    Return SUCCESS if it's localized
    Returns FAILURE if it isn't localized
    """

    def __init__(self):

        rospy.loginfo("Initialising cube_localized behaviour.")

        #  Get topic name
        self.marker_pose_topic = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        
        self.cube_pose = PoseStamped()
        # self.localized = False
        self.message_recieved = False
        self.done = False
        self.fail = False
        self.success = False


        # check absolute value of pose (distance to the cube)
        # fail_threshold = 1 # 1 meter
        # if self.message_recieved:
        #     if (abs(self.cube_pose.pose.position.x) < fail_threshold):
        #         self.localized = True
        #         rospy.loginfo("Cube localized")



        # become a behaviour
        super(cube_localized, self).__init__("cube_localized?")

    def update(self):

        if self.done:
            # rospy.loginfo("cube_localized: done!")
            if self.success:
                return pt.common.Status.SUCCESS
            elif self.fail:
                rospy.loginfo("cube_localized: fail!")
                return pt.common.Status.FAILURE


        rospy.loginfo("cube_localized: Trying to localize cube")

        if not self.done:
            try:
                self.cube_pose = rospy.wait_for_message(self.marker_pose_topic, PoseStamped, timeout=5)
                self.message_recieved = True
                rospy.loginfo("cube_localized: Message recieved")
            except rospy.ROSException, e:
                self.message_recieved = False
                rospy.loginfo("cube_localized: Message not recieved")

        # already localized the cube
        if self.message_recieved: 
            rospy.loginfo("cube_localized: SUCCESS")
            self.done = True
            self.success = True
            return pt.common.Status.SUCCESS
        elif (not self.message_recieved) and (not self.done):
            self.fail = True
            rospy.loginfo("cube_localized: FAILURE")
            return pt.common.Status.FAILURE


class pick_up_cube(pt.behaviour.Behaviour):

    """
    Return SUCCESS once the pick_up is done
    Returns FAILURE if it isn't picked up
    """

    def __init__(self):

        rospy.loginfo("Initialising pick_up cube behaviour.")

        #  Get topic name
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')

        self.done = False

        # become a behaviour
        super(pick_up_cube, self).__init__("pick up the cube!")

    def update(self):

        if self.done:
            # rospy.loginfo("pick_up_cube: done!")
            return pt.common.Status.SUCCESS

        rospy.loginfo("pick_up_cube: Trying to pick up the object")

        try:

            # # Publish pose of the cube
            # self.aruco_pos_pub.publish(self.cube_PoseStamped)

            # Initialice the service
            pick_up_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
            # Request to the service
            pick_up_req = pick_up_srv()

            # Boolean to see if the pick up is succeeded or not
            success_pick_up = pick_up_req.success

            if success_pick_up == True:
                rospy.loginfo("pick_up_cube: Object pick up succeded!")
                self.done = True
                return pt.common.Status.SUCCESS
            else:
                rospy.loginfo("pick_up_cube: Object pick up failed!")
                return pt.common.Status.FAILURE
        
        except rospy.ServiceException, e:
            print("Service call to pick_up server failed: %s"%e)
            return pt.common.Status.FAILURE


class place_cube(pt.behaviour.Behaviour):

    """
    Return SUCCESS once the place_cube is done
    Returns FAILURE if it isn't placed
    """

    def __init__(self):

        rospy.loginfo("Initialising place cube behaviour.")

        #  Get topic name
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')

        self.done = False
        self.succeded = False

        # become a behaviour
        super(place_cube, self).__init__("place the cube!")

    def update(self):

        if self.done:
            # rospy.loginfo("place_cube: done!")
            if self.succeded:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

        rospy.loginfo("place_cube: Trying to place the object")

        try:

            # Initialice the service
            place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
            # Request to the service
            place_req = place_srv()

            # Boolean to see if the place is succeeded or not
            success_place = place_req.success

            if success_place == True:
                rospy.loginfo("place_cube: Object place succeded!")
                self.done = True
                self.succeded = True
                return pt.common.Status.SUCCESS
            else:
                rospy.loginfo("place_cube: Object place failed!")
                self.done = True
                self.succeded = False
                return pt.common.Status.FAILURE
        
        except rospy.ServiceException, e:
            print("Service call to place server failed: %s"%e)
            self.done = True
            self.succeded = False
            return pt.common.Status.FAILURE

class moveb(pt.behaviour.Behaviour):

    """
    Move robot (post Twist to /cmd_vel_topic)
    Returns SUCCESS when movement is done
    """

    def __init__(self, angular, linear, ticks, name):

        rospy.loginfo("Move initialized")

        self.rate = rospy.Rate(10)
        self.ticks = ticks

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Create message
        self.move_msg = Twist()
        self.move_msg.angular.z = angular
        self.move_msg.linear.x = linear

        # Success variables
        self.done = False

        # become a behaviour
        super(moveb, self).__init__(name)

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS

        cnt = 0
        while not rospy.is_shutdown() and cnt < self.ticks:
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            cnt = cnt + 1

        self.move_msg = Twist()
        self.cmd_vel_pub.publish(self.move_msg) # Stop movement
        self.rate = rospy.Rate(2)
        self.rate.sleep() # Wait for 1/2 seconds

        self.done = True
        return pt.common.Status.SUCCESS


# class move_baseb(pt.behaviour.Behaviour):

#     """
#     Moves the robot to table 1 or 2
#     Returns running
#     """

#     def __init__(self, direction_table):
#         rospy.loginfo("Initialising move_base")

#         self.direction_table = direction_table
#         rospy.loginfo("self.direction: %s", self.direction_table)

#         # Set up action client
#         # self.move_base_robot_b = SimpleActionClient("/move_base", MoveBaseAction)
#         # rospy.loginfo("Set up action client move_base_robot: %s", self.move_base_robot_b)

#         # self.move_base_robot_b.wait_for_server()

#         # if not self.move_base_robot_b.wait_for_server(rospy.Duration(30)):
#         #     rospy.logerr("%s: Could not connect to /move_base_robot_b action server", self.move_base_robot_b)
#         #     exit()
#         # rospy.loginfo("%s: Connected to move_base_robot_b action server", self.move_base_robot_b)

#         # self.move_base_simple_robot = SimpleActionClient("/move_base_simple", MoveBaseAction)
#         # self.move_base_simple_robot.wait_for_server()
#         # rospy.loginfo("move_base_robot set up")

#         self.move_base_simple_nm = rospy.get_param(rospy.get_name() + )
#         self.move_base_simple_pub = rospy.Publisher(self.move_base_simple_nm , PoseStamped, queue_size=10)


#         # self.map_frame = rospy.get_param(rospy.get_name() + '/map_frame')

#         # Select topic to get target from
#         if (self.direction_table == "table_2"):
#             self.topic_name = rospy.get_param(rospy.get_name() + '/place_pose_topic')
#             # self.goal_pose = rospy.get_param(rospy.get_name() + '/place_pose')
#         elif (self.direction_table == "table_1"):
#             self.topic_name = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
#             # self.goal_pose = rospy.get_param(rospy.get_name() + '/pick_pose')
       
#         # Get target
#         self.move_base_goal = rospy.wait_for_message(self.topic_name, PoseStamped, timeout=10)
#         rospy.loginfo("move_base_goal recieved")

#         # Create goal message
#         self.move_base_goal = MoveBaseGoal()
#         self.move_base_goal.target_pose = self.move_base_goal  
#         # self.move_base_simple_goal = MoveBaseActionGoal()
#         # self.move_base_simple_goal.goal.target_pose = self.move_base_goal  
        
#         # Success variable
#         self.at_goal = False
#         self.failed = False
#         self.sent_goal = False

#         # become a behaviour
#         super(move_baseb, self).__init__("Move robot base!")

#     def goal_result(self, state, result):
#         rospy.loginfo("result recieved, %s", state)
#         if TerminalState.SUCCEEDED == state:
#             rospy.loginfo("result recieved, Target reached")
#             self.at_goal = True
#         else:
#             self.failed = True

#     def update(self):

#         # Get target again?
#         # self.move_base_goal = rospy.wait_for_message(self.topic_name, PoseStamped, timeout=10)


#         rospy.loginfo("%s: Moving towards the second table", self.node_name)

#         # Obtain message with the pose of table2
#         # table2_pose = rospy.wait_for_message(self.place_pose_top, PoseStamped, timeout=5)

#         move_msg = Twist()
#         move_msg.angular.z = -0.5

#         rate = rospy.Rate(10)
#         cnt = 0
#         rospy.loginfo("%s: Rotating towards table", self.node_name)
#         while not rospy.is_shutdown() and cnt < 62:
#             self.cmd_vel_pub.publish(move_msg)
#             rate.sleep()
#             cnt = cnt + 1

#         move_msg.linear.x = 0.5
#         move_msg.angular.z = 0
#         cnt = 0
#         rospy.loginfo("%s: Approaching table", self.node_name)

#         while not rospy.is_shutdown() and cnt < 20:
#             self.cmd_vel_pub.publish(move_msg)
#             rate.sleep()
#             cnt = cnt + 1

#         self.state = 3
#         rospy.loginfo("%s: Table reached", self.node_name)
#         rospy.sleep(1)
#         if not self.sent_goal:
#             self.move_base_goal.target_pose.header.stamp = rospy.Time.now()
#             self.move_base_robot_b.send_goal(self.move_base_goal, done_cb=self.goal_result)
#             # self.move_base_simple_goal.target_pose.header.stamp = rospy.Time.now()
#             # self.move_base_simple_robot.send_goal(self.move_base_simple_goal, done_cb=self.goal_result)

#             self.sent_goal = True

#         # At goal
#         if self.at_goal: 
#             rospy.loginfo("Move Success")
#             return pt.common.Status.SUCCESS
#         # move_base_goal failed
#         elif self.failed:
#             rospy.loginfo("Move failed")
#             return pt.common.Status.FAILURE
#         # Still running
#         else:
#             # rospy.loginfo("Move running")
#             # return pt.common.Status.RUNNING
#             return pt.common.Status.FAILURE