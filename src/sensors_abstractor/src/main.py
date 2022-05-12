#!/usr/bin/env python
import argparse
import time, logging, yaml
import rospy
import sensors_handler
from hydra import compose, initialize
from omegaconf import OmegaConf

logger = logging.getLogger(__name__)

if __name__ == '__main__':
    logger.addHandler(logging.StreamHandler())
    logger.info("Starting sensors abstractor node")
    rospy.init_node('sensoss_abstractor', anonymous=True)

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

    # cfg = yaml_conf
    sensors_handler = sensors_handler.SensorsHandler(yaml_conf)

    time.sleep(0.1)
    # sensors_handler.init_motors()
    # rate = rospy.Rate(COMM_FREQ)
    try:
        sensors_handler.run()
    except rospy.ROSInterruptException:
        logger.error("EXCEPTION EXCEPTION")
        pass
