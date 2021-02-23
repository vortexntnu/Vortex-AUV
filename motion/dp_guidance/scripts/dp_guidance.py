#!/usr/bin/env python
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose

class DPGuidance:
    """
    The dynamic positioning guidance system is very simple.
    It does not perform any calculations, but simply passes
    the setpoints directly to the DP controller in the
    /controller/dp node.

    The dp guidance node is different from the LOS node in the sense
    that it will not communicate with the controller via a topic.
    This is done because, as mentioned above, the dp guidance node
    does not perform any calculations. Therefore, we have no continuous
    data we need to send, only a discrete goal that will be updated 
    in the FSM.

    One could argue that there is no need for the dp_guidance node;
    and you would be right. However, it is introduced as a means of
    eliminating any crosstalk directly between the fsm and the controller.
    This will facilitate a cleaner, more modular system in the future.
    """

    def __init__(self):
        """
        Create the ROS node dp and set constants, as well as the action
        server that the fsm connects to. Connect to the move_base action
        server in the dp controller. The guidance and controller communicate
        through this server.
        """

        rospy.init_node('dp')

        self.timeout = rospy.get_param('~guidance_dp_timeout', 90)

        self.action_server = actionlib.SimpleActionServer(name='dp_action_server', ActionSpec=MoveBaseAction, auto_start=False)
        self.action_server.register_goal_callback(self.goalCB)  # Called whenever guidance_interface sends a new goal for the dp system
        self.action_server.register_preempt_callback(self.preemptCB)
        self.action_server.start()

        self.should_publish_setpoint = False
        self.setpoint_pub = rospy.Publisher('/reference_model/input', Pose, queue_size=1)

        self.goal_pose = Pose()

    def publish_setpoint(self):
        if self.should_publish_setpoint:
            self.setpoint_pub.publish(self.goal_pose)

    def goalCB(self):
        """
        Accept a goal from the guidance interface and update internal state
        """

        goal = self.action_server.accept_new_goal()
        self.goal_pose = goal.target_pose.pose
        self.should_publish_setpoint = True


    def preemptCB(self):
        """
		The preempt callback for the action server.
		"""

        if self.action_server.is_preempt_requested():
            rospy.loginfo("Preempted requested by dp guidance client!")
            self.action_server.set_preempted()

if __name__ == '__main__':

    try:
        dp_guidance = DPGuidance()
        
        while not rospy.is_shutdown():
            dp_guidance.publish_setpoint()


    except rospy.ROSInternalException as e:
        rospy.logerr(e)