# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib import SimpleActionClient, TerminalState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState

import numpy as np

# Global status variables
tuck_arm_status = False
holding_cube_status = False
robot_head_status = "Unknown"
robot_location_belief = "Unknown"
robot_localization_is_uncertain = True
cube_on_table_2_belief = False

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
    Tuck the arm
    Sends a goal to the tuck arm action server.
    Returns RUNNING
    Set is_tuck when successfully tucked, Return SUCCESS once
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        self.play_motion_ac.wait_for_server(rospy.Duration(10))

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        # self.sent_goal = False
        # self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):
        global tuck_arm_status
        
        # command to tuck arm if haven't already
        # if not self.sent_goal:
        rospy.loginfo("Tucking arm!")

        # send the goal
        self.play_motion_ac.send_goal(self.goal)

        # Wait for response
        play_motion_response = self.play_motion_ac.wait_for_result()


        # if I was succesful! :)))))))))
        if play_motion_response:
            # Set status variable
            tuck_arm_status = True
            play_motion_response = False
            return pt.common.Status.SUCCESS
        
        # if failed
        if not play_motion_response:
            # do nothing
            pass

        # tell the tree you're running :D
        return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raises the head of the robot.
    Returns Running
    Set robot_head_status when done
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

        # become a behaviour
        super(movehead, self).__init__("Move head " +direction+"!")

    def update(self):
        global robot_head_status

        rospy.loginfo("Move head to " + self.direction + "!")

        # try if not tried
        if not self.tried:
            # Call service
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

        # if service was succesful
        if self.move_head_req.success:
            # Reset for next call
            self.tried = False

            # Set global status variable
            robot_head_status = self.direction
            # Return SUCCESS
            return pt.common.Status.SUCCESS
        # if service failed
        else:
            # Reset to try again
            self.tried = False
            # return fail?

        return pt.common.Status.RUNNING

class cube_localized(pt.behaviour.Behaviour):

    """
    Check if cube is localized.
    Return SUCCESS if it is in view
    Returns FAILURE if it is not in view
    """

    global try_again

    def __init__(self):

        rospy.loginfo("Initialising cube_localized behaviour.")

        #  Get topic name
        self.marker_pose_topic = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        
        self.cube_pose = PoseStamped()
        # self.localized = False

        # check absolute value of pose (distance to the cube)
        # fail_threshold = 1 # 1 meter
        # if self.message_recieved:
        #     if (abs(self.cube_pose.pose.position.x) < fail_threshold):
        #         self.localized = True
        #         rospy.loginfo("Cube localized")



        # become a behaviour
        super(cube_localized, self).__init__("cube_localized?")

    def update(self):
        

        rospy.loginfo("cube_localized: Trying to localize cube")

        try:
            self.cube_pose = rospy.wait_for_message(self.marker_pose_topic, PoseStamped, timeout=5)
            rospy.loginfo("Cube located!")
            # rospy.loginfo("cube_localized: Message recieved")
            return pt.common.Status.SUCCESS
        except rospy.ROSException, e:
            # rospy.loginfo("cube_localized: Message not recieved")
            rospy.loginfo("Cube NOT located!")
            return pt.common.Status.FAILURE

class pick_up_cube(pt.behaviour.Behaviour):

    """
    Tries to pick up the cube
    Return SUCCESS or FAILURE
    Sets global status variables tuck_arm_status, holding_cube_status
    """

    def __init__(self):

        rospy.loginfo("Initialising pick_up cube behaviour.")

        #  Get topic name
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')

        self.done = False

        # become a behaviour
        super(pick_up_cube, self).__init__("pick up the cube!")

    def update(self):
        global holding_cube_status
        global tuck_arm_status

        rospy.loginfo("pick_up_cube: Trying to pick up the object")

        try:

            # # Publish pose of the cube
            # self.aruco_pos_pub.publish(self.cube_PoseStamped)

            # Initialice the service
            pick_up_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
            # Request to the service
            pick_up_req = pick_up_srv()
            
            # Set arm status
            tuck_arm_status = False

            # Boolean to see if the pick up is succeeded or not
            success_pick_up = pick_up_req.success

            if success_pick_up == True:
                rospy.loginfo("pick_up_cube: Object pick up succeded!")
                holding_cube_status = True
                return pt.common.Status.SUCCESS

            else:
                rospy.loginfo("pick_up_cube: Object pick up failed!")
                return pt.common.Status.FAILURE
        
        except rospy.ServiceException, e:
            print("Service call to pick_up server failed: %s"%e)
            return pt.common.Status.FAILURE

        # Tell the tree you're still running
        return pt.common.Status.RUNNING

class place_cube(pt.behaviour.Behaviour):

    """
    Return SUCCESS or FAILURE 
    Set is_tuck False
    Set holding_cube_status False
    If place success: Set cube_on_table_2_belief True
    If place fail: Set cube_on_table_2_belief False, return Fail
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
        global tuck_arm_status
        global holding_cube_status
        global cube_on_table_2_belief

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
                holding_cube_status = False
                cube_on_table_2_belief = True
                tuck_arm_status = False
                return pt.common.Status.SUCCESS

            else:
                rospy.loginfo("place_cube: Object place failed!")
                holding_cube_status = False
                cube_on_table_2_belief = False
                tuck_arm_status = False
                return pt.common.Status.FAILURE
        
        except rospy.ServiceException, e:
            print("Service call to place server failed: %s"%e)
            holding_cube_status = False
            cube_on_table_2_belief = False
            tuck_arm_status = False
            return pt.common.Status.FAILURE

        return pt.common.Status.RUNNING

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
        
class relocalize_robot(pt.behaviour.Behaviour):

    """
    Perform global localization of robot
    Rotates the robot to converge estimate
    Returns SUCCESS once localization has converged (normalized covariance < self.tol)
    """

    def __init__(self):

        rospy.loginfo("relocalize_robot initializing")

        # Access rosparam
        self.topic_name = rospy.get_param(rospy.get_name() + '/amcl_estimate')

        # Set tolerance
        self.tol = 0.02

        # Get service
        global_localization_srv_nm= rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.global_localization_srv = rospy.ServiceProxy(global_localization_srv_nm, Empty)
        rospy.wait_for_service(global_localization_srv_nm, timeout=30)

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Create message
        self.move_msg = Twist()

        # Success variable
        self.success = False

        # become a behaviour
        super(relocalize_robot, self).__init__("relocalize_robot!")

    def update(self):
        rospy.loginfo("relocalize_robot: Update called")

        # Reset success
        self.success = False

        
        # Do global localization (Spread particles)
        try:
            self.global_localization_srv()
        except rospy.ROSException, e:
            rospy.loginfo("Global localization failed")

        rospy.loginfo("relocalize_robot: Rotate")
        r = rospy.Rate(4)
        cnt = 0
        while not self.success:
            # Rotate robot (to update particle filter)
            self.move_msg.angular.z = 0.5
            self.cmd_vel_pub.publish(self.move_msg)

            # Recieve message from localization
            msg = rospy.wait_for_message(self.topic_name, PoseWithCovarianceStamped, timeout=10)
            # rospy.loginfo("relocalize_robot: msg recieved")

            # Normalize cov matrix
            cov_norm = np.linalg.norm(msg.pose.covariance)
            rospy.loginfo("relocalize_robot: cov normed = %s", cov_norm)

            # If filter doesn't converge in reasonable time...
            cnt = cnt+1
            if cnt > 100:
                cnt = 0
                rospy.loginfo("relocalize_robot: Did not converge. Retrying.")
                # Do global localization (Spread particles)
                try:
                    self.global_localization_srv()
                except rospy.ROSException, e:
                    rospy.loginfo("Global localization failed")


            # Check if success
            if cov_norm < self.tol:
                self.success = True

            r.sleep()

        # Stop rotation
        self.move_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_msg)
        

        # Return
        if self.success:
            rospy.loginfo("relocalize_robot succeeded")
            rospy.Rate(1).sleep()
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("relocalize_robot: Failure")
            rospy.Rate(1).sleep()
            return pt.common.Status.FAILURE

        # Tell the tree you're still running
        return pt.common.Status.RUNNING

class is_robot_localized(pt.behaviour.Behaviour):

    """
    Check if localized, if covariance < tol
    If robot_localization_is_uncertain it will rotate for 2s to help the particle filter converge/diverge before measuring
    Returns SUCCESS if localized
    Returns FAILURE if not localized
    """

    def __init__(self):
        # Set tollerance
        self.tol = 0.04 # Slightly larger than tolerance for global localization

        rospy.loginfo("is_robot_localized initialized")

        # Access rospaself.first_time = Falserams
        self.topic_name = rospy.get_param(rospy.get_name() + '/amcl_estimate')

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Create message
        self.move_msg = Twist()

        # Counter for not printing every time
        self.counter = 0

        # become a behaviour
        super(is_robot_localized, self).__init__("is localized?")

    def update(self):
        # rospy.loginfo("is_robot_localized: Update called")
        global robot_localization_is_uncertain

        if robot_localization_is_uncertain:
            # Rotate to update particle filter
            self.move_msg.angular.z = -1
            r = rospy.Rate(5)
            cnt = 0
            while not rospy.is_shutdown() and cnt < 30:
                self.cmd_vel_pub.publish(self.move_msg)
                cnt = cnt + 1
                r.sleep()

            # Stop rotation
            self.move_msg = Twist()
            self.cmd_vel_pub.publish(self.move_msg)
            robot_localization_is_uncertain = False


        # Recieve message from localization
        msg = rospy.wait_for_message(self.topic_name, PoseWithCovarianceStamped, timeout=10)
        # rospy.loginfo("is_robot_localized: PF msg recieved")

        # Normalize cov matrix
        cov_norm = np.linalg.norm(msg.pose.covariance)
        
        # Check if success
        if cov_norm < self.tol:
            # rospy.loginfo("is_robot_localized: Success")
            rospy.loginfo("Robot is localized: cov normed = %.3f", cov_norm)
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("Robot is NOT localized: cov normed = %.3f", cov_norm)
            return pt.common.Status.FAILURE    

class move_base_to_table(pt.behaviour.Behaviour):
    
    """
    Moves the robot to table 1 or 2
    Returns RUNNING
    when AC succeded, SUCCESS
    Any other result from AC, FAILURE

    Set robot_location_belief when done
    """

    def __init__(self, direction_table):
        rospy.loginfo("Initialising move_base_to_" + direction_table)

        self.direction_table = direction_table
        # rospy.loginfo("self.direction: %s", self.direction_table)

        # Set up action client
        self.move_base_robot_b = SimpleActionClient("/move_base", MoveBaseAction)
        # rospy.loginfo("Set up action client move_base_robot: %s", self.move_base_robot_b)
        self.move_base_robot_b.wait_for_server(rospy.Duration(20))

        # Select topic to get target from
        if (self.direction_table == "table_2"):
            self.topic_name = rospy.get_param(rospy.get_name() + '/place_pose_topic')
            # self.goal_pose = rospy.get_param(rospy.get_name() + '/place_pose')
        elif (self.direction_table == "table_1"):
            self.topic_name = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
            # self.goal_pose = rospy.get_param(rospy.get_name() + '/pick_pose')
       
        # Get target
        self.move_base_goal = rospy.wait_for_message(self.topic_name, PoseStamped, timeout=10)
        # rospy.loginfo("move_base_goal recieved")


        # Success variable
        self.at_goal = False
        self.failed = False
        self.sent_goal = False
        self.is_reset = True
        self.state = -1

        # become a behaviour
        super(move_base_to_table, self).__init__("Move robot base to " + self.direction_table + "!")

    def goal_result(self, state, result):
        global robot_location_belief

        rospy.loginfo("result recieved, %s", state)

        self.state = state

        if TerminalState.SUCCEEDED == state:
            rospy.loginfo("Action returned succeeded, Target reached")

        elif TerminalState.RECALLED == state:
            rospy.loginfo("Action returned recalled")
        elif TerminalState.REJECTED == state:
            rospy.loginfo("Action returned rejected")
        elif TerminalState.PREEMPTED == state:
            rospy.loginfo("Action returned preempted")
        elif TerminalState.ABORTED == state:
            # This happens when no path is found
            # Will repeat if no costmap update, or relocalization
            rospy.loginfo("Action returned aborted")
        elif TerminalState.LOST == state:
            rospy.loginfo("Action returned lost")

    def goal_feedback(self, feedback):
        pass
        # This is the location of the robot in pose stamped, map frame
        # rospy.loginfo("Feedback = "+str(feedback))

    def update(self):
        global robot_location_belief
        global robot_localization_is_uncertain
        # if try_again == True and self.is_reset:
        #     self.sent_goal = False
        #     self.failed = False
        #     self.at_goal = False
        #     # self.is_reset = True
        robot_location_belief = "Unknown"

        if self.sent_goal == False:
            # Create goal message
            self.move_base_robot_b.cancel_all_goals()
            rospy.Rate(1).sleep()
            goal = MoveBaseGoal()
            goal.target_pose = self.move_base_goal
            self.move_base_robot_b.send_goal(goal, done_cb = self.goal_result, feedback_cb=self.goal_feedback) 
            self.sent_goal = True
            rospy.loginfo("move_base_to_" + self.direction_table + " Goal Sent")

            # Set global status variable
            robot_location_belief = "Unknown"
            # rospy.loginfo("move_base_to_table: sent_goal = True")

        rospy.loginfo("move_base_to_" + self.direction_table + " State: " + str(self.state))

        # If aborted
        if self.state == TerminalState.ABORTED:
            # Set global state variable (to trigger rotation in Is robot localized)
            rospy.loginfo("robot_localization_is_uncertain = True")
            robot_localization_is_uncertain = True

        # Fail once if action client was not successfull
        if self.state != -1 and self.state != TerminalState.SUCCEEDED:
            rospy.loginfo("move_base_to_" + self.direction_table + " Return FAILURE")
            # Reset state variable
            self.state = -1
            self.sent_goal = False
            # Set global state variable
            robot_location_belief = "Unknown"
            return pt.common.Status.FAILURE

        # Reset state if succeeded
        if self.state == TerminalState.SUCCEEDED:
            rospy.loginfo("move_base_to_" + self.direction_table + " Return SUCCESS")
            
            # Reset for more attempts
            self.sent_goal = False
            self.state = -1

            # Set global state variable
            robot_location_belief = self.direction_table
            return pt.common.Status.SUCCESS

        # Tell table youre still RUNNINGN
        return pt.common.Status.RUNNING

class is_robot_at_table(pt.behaviour.Behaviour):

    """
    Check location of robot.
    Return SUCCESS if location == direction
    Returns FAILURE if not == direction
    """

    def __init__(self, direction):

        rospy.loginfo("init Is robot at " + direction + "?")

        self.direction = direction

        # become a behaviour
        super(is_robot_at_table, self).__init__("Is robot at " + direction + "?")

    def update(self):
        global robot_location_belief
        if self.direction == robot_location_belief:
            rospy.loginfo("Robot is at " + self.direction + "!")
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("Robot is NOT at " + self.direction + "!")
            return pt.common.Status.FAILURE

class is_head(pt.behaviour.Behaviour):

    """
    Check status of head.
    Return SUCCESS if status == direction
    Returns FAILURE if status not == direction
    """

    def __init__(self, direction):

        rospy.loginfo("init Is head " + direction + "?")

        self.direction = direction

        # become a behaviour
        super(is_head, self).__init__("Is head " + direction + "?")

    def update(self):
        global robot_head_status
        if self.direction == robot_head_status:
            rospy.loginfo("Head is at " + self.direction + "!")
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("Head is NOT at " + self.direction + "!")
            return pt.common.Status.FAILURE

class is_holding_cube(pt.behaviour.Behaviour):

    """
    Check status of holding_cube_status.
    Return SUCCESS if True
    Returns FAILURE if False
    """

    def __init__(self):

        rospy.loginfo("init Is holding cube?")

        # become a behaviour
        super(is_holding_cube, self).__init__("Is holding cube?")

    def update(self):
        global holding_cube_status
        if holding_cube_status:
            rospy.loginfo("Robot is holding cube!")
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("Robot is NOT holding cube!")
            return pt.common.Status.FAILURE

class is_arm_tuck(pt.behaviour.Behaviour):

    """
    Check status of "Is arm tuck?".
    Return SUCCESS if True
    Returns FAILURE if False
    """

    def __init__(self):

        rospy.loginfo("init Is arm tuck?")

        # become a behaviour
        super(is_arm_tuck, self).__init__("Is arm tuck?")

    def update(self):
        global tuck_arm_status
        if tuck_arm_status:
            rospy.loginfo("Arm is tucked!")
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("Arm is NOT tucked!")
            return pt.common.Status.FAILURE

class respawn_cube(pt.behaviour.Behaviour):

    """
    Politely ask for help
    Return FAILURE
    """
    def __init__(self):

        rospy.loginfo("init respawn_cube NOT AUTOMATED YET!!!!!!")

        rospy.wait_for_service('/gazebo/set_model_state', timeout=20)
        self.set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.cube_at_start_possition = ModelState()
        self.cube_at_start_possition.model_name = 'aruco_cube'
        self.cube_at_start_possition.pose.position.x = -1.130530
        self.cube_at_start_possition.pose.position.y = -6.653650
        self.cube_at_start_possition.pose.position.z = 0.88
        self.cube_at_start_possition.twist.linear.z = 2
        self.cube_at_start_possition.twist.angular.x = 14
        self.cube_at_start_possition.reference_frame = "map"

        # become a behaviour
        super(respawn_cube, self).__init__("Respawn cube!")

    def update(self):
        
        rospy.loginfo("respawn_cube: Help! Please respawn the cube!")
        resp = self.set_model_state_srv(self.cube_at_start_possition)
        rospy.sleep(1)
        
        if resp.success:
            rospy.loginfo("Bottleflip SUCCESS, Status: " + resp.status_message)
            return pt.common.Status.SUCCESS

        rospy.loginfo("respawn_cube: Failed, Status: " + resp.status_message)
        return pt.common.Status.FAILURE

class is_cube_on_table_2_belief(pt.behaviour.Behaviour):

    """
    Check status of cube_on_table_2_belief.
    Return SUCCESS if True
    Returns FAILURE if False
    """

    def __init__(self):

        rospy.loginfo("init is_cube_on_table_2_belief")

        # become a behaviour
        super(is_cube_on_table_2_belief, self).__init__("Is cube on table 2? (belief)")

    def update(self):
        global cube_on_table_2_belief
        if cube_on_table_2_belief:
            rospy.loginfo("Robot believes cube is on table 2!")
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("Robot believes cube is NOT on table 2!")
            return pt.common.Status.FAILURE

class reset_cube_belief(pt.behaviour.Behaviour):

    """
    Set cube_on_table_2_belief to False
    Returns SUCCESS
    """

    def __init__(self):

        rospy.loginfo("init reset_cube_belief")

        # become a behaviour
        super(reset_cube_belief, self).__init__("reset_cube_belief")

    def update(self):
        global cube_on_table_2_belief
        rospy.loginfo("reset_cube_belief")
        cube_on_table_2_belief = False

        return pt.common.Status.SUCCESS
