import time

import rospy, logging, json
from std_msgs.msg import String
from sensor_msgs.msg import Joy

logger = logging.getLogger("commands_handler")


class CommandsHandler:
    def __init__(self, conf):
        self.conf = conf
        self.dofs = []
        self.joints_pub = {}
        self.joints_sub = {}
        self.init_pos = {}
        self.current_pos = {}

        self.joints_params = conf.joints_params
        self.robot_name = self.conf.robot_config["robot_name"]
        self.frequency = self.conf.communication["frequency"]
        self.rqt_debug = self.conf.debug["rqt_pos"]
        self.simulation = self.conf.debug["simulate_motors"]
        self.Publisher = rospy.Publisher(self.robot_name + "/command_controller", String,  queue_size=10)
        self.motors_handler_pub = rospy.Publisher(self.robot_name + "/motors_handler", String,  queue_size=10)
        self.joy_subs = rospy.Subscriber('/joy', Joy, self.joy_callback)
        parsed_joint_params = conf.joints_params
        for dof in parsed_joint_params:
            self.dofs.append(dof)
        for dof in self.dofs:
            self.joints_pub[dof] = rospy.Publisher(self.robot_name + "/"+dof +"/command", String,  queue_size=10)
            self.joints_sub[dof] = rospy.Subscriber(self.robot_name + "/" + dof, String, self.joint_sub_callback)
            self.init_pos[dof] = self.joints_params[dof].init_pos

        self.active_motor = 0


    def joy_callback(self, joy_teleop):
        if joy_teleop.buttons[1] == 1:
            command = [{'command_type': "start", "policy_name": "simha_standing"}]
            print("Commander: START")
            self.Publisher.publish(json.dumps(command))

        if joy_teleop.buttons[2] == 1:
            command = [{'command_type': "stop", "policy_name": "simha_standing"}]
            self.Publisher.publish(json.dumps(command))
            print("Commander: STOP")

        if joy_teleop.buttons[0] == 1:
            self.all_to_init_pos()


        if joy_teleop.buttons[4] == 1:
            self.motors_handler_pub.publish("stop")
            print("Motors: STOP")


        if joy_teleop.buttons[6] == 1:
            self.motors_handler_pub.publish("start")
            print("Motors: START")

        if joy_teleop.axes[0] != 0:
            self.active_motor += joy_teleop.axes[0]
            self.active_motor = int(self.active_motor%len(self.dofs))
            print(f"Motor {self.dofs[self.active_motor]} is now Selected")

        if joy_teleop.buttons[3] == 1 and joy_teleop.buttons[5] == 1:
            self.motors_handler_pub.publish(self.dofs[self.active_motor])
            print(f"Setting motor {self.dofs[self.active_motor]} to zero")

    def do_zero(self):
        ...

    def all_to_init_pos(self):
        print("go to init mother fuckerrrrssss")
        if self.current_pos == {}:
            print("No Current position from Motors")
        for dof in self.dofs:
            print(f"{dof} current pos: {self.current_pos[dof]} and init pos: {self.init_pos[dof]}")
            iter = 0
            while abs(self.current_pos[dof] - self.init_pos[dof]) > 0.05 and iter < 10:
                delta = self.init_pos[dof] - self.current_pos[dof]
                desired_pos = self.init_pos[dof] - 0.6*delta
                print(f"{dof} delta: {delta}")
                command = {'pos': desired_pos, 'kp': 500, 'kd': 1, 'vel': 0.0, 'tor': 0.0}
                self.joints_pub[dof].publish(json.dumps(command))
                time.sleep(1)
                iter +=1
                if(iter > 9):
                    print("Retries Timeout...")
            print(f"Commander: {dof} within desired zero")
            print("Moving to the next Motor")
            time.sleep(3)
        print("Done Initiating Motors")



    def joint_sub_callback(self, state):
        new_state = json.loads(state.data)
        joint_name = new_state['name']
        joint_pos = new_state['pos']
        self.current_pos[joint_name] = joint_pos

    def run(self):
        rospy.spin()