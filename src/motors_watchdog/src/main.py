#!/usr/bin/env python
import time, logging, yaml,argparse
import rospy
import watchdog_handler
from hydra import compose, initialize
from omegaconf import OmegaConf

logger = logging.getLogger(__name__)

if __name__ == '__main__':

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

    logger.addHandler(logging.StreamHandler())
    logger.info("Starting motors watchdog node")
    rospy.init_node('motors_watchdog', anonymous=True)


    watchdog_handler = watchdog_handler.WatchdogHandler(yaml_conf)
    time.sleep(0.1)
    # watchdog_handler.init_motors()
    # rate = rospy.Rate(COMM_FREQ)
    try:
        watchdog_handler.run()
    except rospy.ROSInterruptException:
        logger.error("EXCEPTION EXCEPTION")
        pass
