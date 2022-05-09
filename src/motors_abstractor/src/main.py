#!/usr/bin/env python
import time, logging,yaml
import motors_handler
import rospy
from hydra import compose, initialize
from omegaconf import OmegaConf




JOINT_PARAMS_PATH = "/home/taio/Documents/menteebot_hw/src/menteebot_hw/config/joint_params"
logger = logging.getLogger("main_ma")


if __name__ == '__main__':
    logger.addHandler(logging.StreamHandler())
    logger.info("Starting motors abstractor node")
    rospy.init_node('motors_abstractor', anonymous=True)

    default = "../../menteebot_hw/config/hw_config.yaml"
    # Configuring hydra
    yaml_path_split = default.split("/")
    config_path = "/".join(yaml_path_split[:-1])
    config_name = yaml_path_split[-1][:-5]
    with initialize(config_path=config_path, job_name="mentor_app"):
        yaml_conf = compose(config_name=config_name, overrides=["hydra.run.dir=/tmp"])
        # Struct to normal :)
        yaml_conf = OmegaConf.to_container(yaml_conf)
        yaml_conf = OmegaConf.create(yaml_conf)

    motors_handler = motors_handler.MotorsHandler(yaml_conf)
    time.sleep(0.1)
    motors_handler.init_motors()
    # rate = rospy.Rate(COMM_FREQ)
    try:
        motors_handler.run()
    except rospy.ROSInterruptException:
        logger.error("EXCEPTION EXCEPTION")
        pass
