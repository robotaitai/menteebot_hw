#!/usr/bin/env python3

from std_msgs.msg import String
import motor_watchdog
import yaml, rospy

import logging
logger = logging.getLogger(__name__)

class WatchdogHandler:
    def __init__(self, conf):
        self.joints = []
        self.conf = conf
        self.robot_name = self.conf.robot_config["robot_name"]
        self.subscriber = rospy.Subscriber(self.robot_name + "/motors_watchdog", String, self.ros_actions)
        parsed_joint_params = conf.joints_params
        for dof in parsed_joint_params:
            self.joints.append(motor_watchdog.MotorWatchdog(parsed_joint_params[dof]))

    def ros_actions(self, msg):
        print(msg)


    def run(self):
        rospy.spin()

