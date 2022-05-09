import time

import yaml, rospy
import motors_abstractor
from std_msgs.msg import String


import logging
logger = logging.getLogger("motors_handler")

class MotorsHandler:
    def __init__(self, conf):
        self.joints = {}
        self.dofs = []
        self.conf = conf
        self.robot_name = self.conf.robot_config["robot_name"]
        self.frequency = self.conf.communication["frequency"]
        self.sleep_between_motors = self.conf.communication["sleep_time_between_motors"]
        self.subscriber = rospy.Subscriber(self.robot_name +"/motors_handler", String, self.ros_actions)
        parsed_joint_params = conf.joints_params
        for dof in parsed_joint_params:
            self.dofs.append(dof)
        for dof in self.dofs:
            self.joints[dof] = motors_abstractor.MotorsAbstractor(parsed_joint_params[dof], parsed_joint_params[dof].init_pos)

    def init_motors(self):
        logger.info("Init motors Command Received")
        for dof in self.dofs:
            # self.joints[dof].motor_set_zero()
            # time.sleep(0.002)
            self.joints[dof].motor_init()
        rospy.Timer(rospy.Duration(1.0/self.frequency), self.publish_all_motors)


    def send_zeros(self):
        for dof in self.dofs:
            self.joints[dof].send_zeros()

    # def set_zero_pos_motors(self):
    #     print("Zero motors Command Received")
    #     for dof in self.dofs:
    #         self.joints[dof].motor_set_zero()

    def disable_motors(self):
        logger.info("Stop motors Command Received")
        for dof in self.dofs:
            self.joints[dof].motor_stop()


    def publish_all_motors(self, event=None):
        # for joint in self.joints:
        #     joint.send_position_and_update_status(joint.des_angle)  # TODO Demo
        for dof in self.dofs:
            self.joints[dof].send_position_and_update_status(self.joints[dof].motor_desire_command.des_pos) #TODO
            rospy.sleep(self.sleep_between_motors)

    def ros_actions(self, msg):
        logger.info(f"ROS actiosn {msg}")
        if msg == "init":
            self.init_motors()
        # elif msg == "zero":
        #     self.set_zero_pos_motors()
        elif msg == "stop":
            self.disable_motors()

    def run(self):
        rospy.spin()