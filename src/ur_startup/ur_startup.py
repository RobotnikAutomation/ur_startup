#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import math
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from ur_dashboard_msgs.msg import RobotMode

class UrStartup(RComponent):

    def __init__(self):

        self.arm_program_running = False
        self.trigger_program = False
        self.last_time = rospy.get_time()

        RComponent.__init__(self)

    def ros_read_params(self):

        self.connect_service_name = rospy.get_param('~connect_service_name', 'ur_hardware_interface/dashboard/connect')
        self.run_arm_program_service_name = rospy.get_param('~run_arm_program_service_name', 'ur_hardware_interface/dashboard/play')
        self.robot_mode_topic_name = rospy.get_param('~robot_mode_topic_name', 'ur_hardware_interface/robot_mode')
        self.robot_program_running_topic_name = rospy.get_param('~robot_program_running_topic_name', 'ur_hardware_interface/robot_program_running')
        self.timeout = rospy.get_param('~timeout', 5)

        RComponent.ros_read_params(self)

    def ros_setup(self):

        # Services

        self.connect_client = rospy.ServiceProxy(self.connect_service_name, Trigger)
        self.run_arm_program_client = rospy.ServiceProxy(self.run_arm_program_service_name, Trigger)

        # Topics

        self.robot_mode_sub = rospy.Subscriber(self.robot_mode_topic_name, RobotMode, self.robot_mode_cb)

        self.arm_program_running = rospy.Subscriber(self.robot_program_running_topic_name, Bool, self.arm_program_running_cb)

        RComponent.ros_setup(self)

        return 0

    def init_state(self):

        return RComponent.init_state(self)

    def ready_state(self):

        if self.trigger_program:

            elapsed_time = rospy.get_time() - self.last_time

            if elapsed_time > self.timeout: 

                try:
                    rospy.logwarn("Connect to Polyscope...")
                    response = self.connect_client(TriggerRequest())
                    rospy.logwarn("Play program in Polyscope")
                    response = self.run_arm_program_client(TriggerRequest())
                except:
                    rospy.logerr("Error playing program, is Polyscope in remote mode?")

                self.trigger_program = False
                self.last_time = rospy.get_time()
        else:
            self.last_time = rospy.get_time() 

        return RComponent.ready_state(self)

    def emergency_state(self):

        self.switch_to_state(State.READY_STATE)

    def shutdown(self):

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):

        return RComponent.switch_to_state(self, new_state)

    def robot_mode_cb(self, msg):

        rospy.logwarn("robot_mode: " + str(msg.mode))

        if msg.mode == 7 and self.arm_program_running == False:
            rospy.logwarn("Robot arm is ready, autostart running!")
            self.trigger_program = True 
        else:
            self.trigger_program = False

        return

    def arm_program_running_cb(self, msg):

        self.arm_program_running = msg.data
        rospy.logwarn("arm_program: " + str(msg.data))
