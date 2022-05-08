#!/usr/bin/env python3
import math

import rospy, yaml, rospkg
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState

JOINT_PARAMS_PATH = "/mnt/nvme0n1p1/PycharmProjects/taio_ws/src/motors_watchdog/src/parameters/joint_params"
verbose = True

pub_error = rospy.Publisher("/soma/watchdog_error", String, queue_size=10)
import logging
logger = logging.getLogger(__name__)




def msg_validator(msg, arg):
    publisher = arg[1]
    # verboseprint(arg[0]['name'])
    limited_msg = JointState()
    limited_msg.header = msg.header
    limited_msg.name = arg[0]['name'] +'/limited'
    limited_msg.header.stamp = rospy.Time.now()
    limited_msg.position.append(msg.position[0])
    limited_msg.velocity.append(msg.velocity[0])
    limited_msg.effort.append(msg.effort[0])

    # Limited Values
    min_position = arg[0]['min_position']
    max_position = arg[0]['max_position']
    max_velocity = arg[0]['max_velocity']
    max_effort = arg[0]['max_effort']

    if msg.position[0] < min_position:
<<<<<<< HEAD:src/motors_watchdog/src/motors-watchdog-node.py
        logger.warning("CAN Watchdog Alert for: {} Desired Pos: {} is Lower that the min allowed: {}".format(arg[0]['name'],msg.position[0],min_position))
        limited_msg.position[0] = min_position

    if msg.position[0] > max_position:
        logger.warning("CAN Watchdog Alert for: {}  Desired Pos: {} is Higher that the max allowed: {}".format(arg[0]['name'],msg.position[0],max_position))
        limited_msg.position[0] = max_position

    if msg.velocity[0] > max_velocity:
        logger.warning("CAN Watchdog Alert for: {}  Desired Velocity: {} is Higher that the max allowed: {}".format(arg[0]['name'],msg.velocity[0],max_velocity))
        limited_msg.velocity[0] = max_velocity

    if msg.effort[0] > max_effort:
        logger.warning("CAN Watchdog Alert for: {}  Desired Effort: {} is Higher that the max allowed: {}".format(arg[0]['name'],msg.effort[0],max_effort))
=======
        verboseprint("CAN Watchdog Alert for: {} Desired Pos: {} is Lower that the min allowed: {}".format(arg[0]['name'],msg.position[0],min_position))
        limited_msg.position[0] = min_position

    if msg.position[0] > max_position:
        verboseprint("CAN Watchdog Alert for: {}  Desired Pos: {} is Higher that the max allowed: {}".format(arg[0]['name'],msg.position[0],max_position))
        limited_msg.position[0] = max_position

    if msg.velocity[0] > max_velocity:
        verboseprint("CAN Watchdog Alert for: {}  Desired Velocity: {} is Higher that the max allowed: {}".format(arg[0]['name'],msg.velocity[0],max_velocity))
        limited_msg.velocity[0] = max_velocity

    if msg.effort[0] > max_effort:
        verboseprint("CAN Watchdog Alert for: {}  Desired Effort: {} is Higher that the max allowed: {}".format(arg[0]['name'],msg.effort[0],max_effort))
>>>>>>> a1c6fdde9f169c96802779a876bacf2315599982:src/can_watchdog/src/canWatchdogNode.py
        limited_msg.effort[0] = max_effort

    publisher.publish(limited_msg)
    # print(limited_msg)


def init_can_watchdog(joint_params):
    parsed_joint_params = yaml.load(open(joint_params), Loader=yaml.FullLoader)
    # print(parsed_joint_params)
    #TODO take care of default config and error in the yaml

    for joint in parsed_joint_params:
<<<<<<< HEAD:src/motors_watchdog/src/motors-watchdog-node.py
        print("starting new topic for: /happiness/" + joint)
        pub = rospy.Publisher("/happiness/"+joint+"/limited", JointState, queue_size=10)
        rospy.Subscriber('/happiness/' + joint, JointState, msg_validator, [parsed_joint_params[joint],pub])
=======
        print("starting new topic for: /soma/" + joint)
        pub = rospy.Publisher("/soma/"+joint+"/limited", JointState, queue_size=10)
        rospy.Subscriber('/soma/' + joint, JointState, msg_validator, [parsed_joint_params[joint],pub])
>>>>>>> a1c6fdde9f169c96802779a876bacf2315599982:src/can_watchdog/src/canWatchdogNode.py
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('actuators_watchdog', anonymous=True)
    try:
        init_can_watchdog(JOINT_PARAMS_PATH)
    except rospy.ROSInterruptException:
        pass
