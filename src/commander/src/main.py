#!/usr/bin/env python
import time, logging,yaml
import commands_handler
import rospy
from hydra import compose, initialize
from omegaconf import OmegaConf




logger = logging.getLogger("main_ma")


if __name__ == '__main__':
    logger.addHandler(logging.StreamHandler())
    logger.info("Starting motors abstractor node")
    rospy.init_node('commander', anonymous=True)

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
    commander = commands_handler.CommandsHandler(yaml_conf)
    # rate = rospy.Rate(COMM_FREQ)
    try:
        commander\
            .run()
    except rospy.ROSInterruptException:
        logger.error("EXCEPTION EXCEPTION")
        pass
