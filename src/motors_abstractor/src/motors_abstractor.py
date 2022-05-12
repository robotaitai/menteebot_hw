#!/usr/bin/env python3

import rospy, json, time
from utilities import utils
import can_motor_controller as mot_con
from std_msgs.msg import String, Float64

import logging
logger = logging.getLogger(__name__)

global msg_id
msg_id = 0
ROBOT_NAME = "/simha/"

class MotorsAbstractor:
    #the update is made here.... I need to figure out how to get the same class object here too
    # in which function the update needs to be done?
    #
    def __init__(self, parsed_joint_params, new_angle=0, simulation=False):
        self.joint_name = parsed_joint_params['name']
        # self.motor_name = parsed_joint_params['name']
        self.can_params = utils.CanParams(parsed_joint_params['can_id'],parsed_joint_params['can_socket'])
        self.motor_desire_command = utils.MotorCommand(new_angle,0,0,0,0)
        '''
        Motos Config
        '''
        self.motor_type = parsed_joint_params['motor_type']
        self.motor_params = utils.MotorParams(self.motor_type).get_motor_params()
        self.motor_controller = mot_con.CanMotorController(self.can_params, self.motor_params, self.joint_name, simulation=simulation)
        self.is_demo = simulation
        '''
        ROS entities
        '''
        self.ros_params = utils.RosParams("/watchdog/" + self.joint_name, ROBOT_NAME + self.joint_name, ROBOT_NAME + self.joint_name + "/error")
        self.publisher = rospy.Publisher(self.ros_params.publishing_as, String, queue_size=10)
        self.error_publisher = rospy.Publisher(self.ros_params.publishing_err, String, queue_size=10)
        self.subscriber = rospy.Subscriber(self.ros_params.subscribing_to, String, self.update_desired_command)


        logger.info(f"New Motor: {self.joint_name} Subs to: {self.ros_params.subscribing_to} and Pubs to: {self.ros_params.publishing_as}")
        logger.info("Created new Motor for: {} CAN ID: {} Subs to: {} and Pubs to: {}".format(self.joint_name, self.can_params.can_id,
                                                                                        self.ros_params.subscribing_to,
                                                                                        self.ros_params.publishing_as))
    def send_zeros(self, status_handler):
        self.send_position_and_update_status(0.0, status_handler)



    def send_position_and_update_status(self, desired_position, SPEED_VALUE =0, KP_VALUE=200, KD_VALUE=0.1, TORQUE_VALUE=0, status_handler = None):
        '''
        Send desired position over can
        and update the received status
        '''
        if self.is_demo:
            status_handler.update_joint(self.can_params.can_id,[self.can_params.can_id,self.motor_desire_command.des_pos, self.motor_desire_command.des_vel,
                                                              self.motor_desire_command.des_torque])
            return 1

        new_status = self.motor_controller.send_rad_command(self.motor_desire_command.des_pos, self.motor_desire_command.des_vel,
                                                            self.motor_desire_command.des_kp, self.motor_desire_command.des_kd,
                                                              self.motor_desire_command.des_torque)
        # new_status = None
        if new_status == None: #TODO we do not know where's the error
            self.error_publisher.publish("TIMEOUT EXCEPTION: Got NONE message from motor") #TODO this is not accurate
            return 0
        else:
            #This is where I get the new status from the motor
            # self.motor_status.update_from_motor(new_status)
            if int(new_status[0], 2) != 0:
                status_handler.update_joint(int(new_status[0], 2), new_status)

            # motor_id, pos, vel, curr = motor_statu

            return 1

    def publish_latest_status(self, status_handler):
        '''

        '''
        global msg_id
        msg_id += 1
        # new_msg = {'message_id': msg_id, 'name': self.joint_name, 'time_stamp': time.time(),
        #            'pos': self.motor_status.act_pos, 'vel': self.motor_status.act_vel, 'torque': self.motor_status.act_torque}
        latest_pos, latest_vel, latest_torque, t_stamp = status_handler.get_status(self.can_params.can_id)

        new_msg = {'message_id': msg_id, 'name': self.joint_name, 'time_stamp': t_stamp,
                   'pos': latest_pos, 'vel': latest_vel, 'torque': latest_torque}
        new_msg_json = json.dumps(new_msg)
        self.publisher.publish(new_msg_json)




    def motor_init(self):
        if self.is_demo:
            return 1

        status = self.motor_controller.enable_motor()
        # status = None
        if status == None:
            logger.warning(f"Motor Init Failed for: {self.joint_name} can id: {self.can_params.can_id}") #TODO do we want a warning here?
            return 0
        else:
            # self.motor_status.update_from_motor(status)
            return 1

    def motor_set_zero(self):
        if self.is_demo:
            return 0
        status = self.motor_controller.set_zero_position()
        if status == None:
            logger.warning(f"Zero Position Failed for: {self.joint_name}")
            return 0
        else:
            # self.motor_status.update_from_motor(status)
            return 1

    def motor_stop(self):
        if self.is_demo:
            return 0
        status = self.motor_controller.disable_motor()
        if status == None:
            logger.warning(f"Motor Disable Failed for: {self.joint_name}")
            return 0
        else:
            # self.motor_status.update_from_motor(status)
            return 1

    def update_desired_command(self, msg):
        command = json.loads(msg.data)
        self.motor_desire_command.des_pos = command['position']
        self.motor_desire_command.des_vel = command['velocity']
        self.motor_desire_command.des_torque = command['torque']
        self.motor_desire_command.des_kp = command['kp']
        self.motor_desire_command.des_kd = command['kd']

    # def update_from_motor(self, status):
    #     self.motor_status.mot_id, self.motor_status.act_pos, self.motor_status.act_vel ,self.motor_status.act_torque = status

    """
    Debugging Section
    """
    def create_rqt_pub(self):
        self.rqt_pub = rospy.Publisher("/rqt_pos_pub/"+self.joint_name, Float64, queue_size=10)

    def rqt_pos_pub(self, status_handler):
        latest_pos, latest_vel, latest_torque, t_stamp = status_handler.get_status(self.can_params.can_id)
        self.rqt_pub.publish(latest_pos)

