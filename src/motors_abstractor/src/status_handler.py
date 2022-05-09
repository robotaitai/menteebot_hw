from dataclasses import dataclass



@dataclass
class MotorStatus:
    act_pos: float
    act_vel: float
    act_torque: float
    des_pos: float
    mot_id: int

    def update_status(self, new_pos, new_vel, new_torque):
        self.act_pos = new_pos
        self.act_vel = new_vel
        self.act_torque = new_torque


    def update_from_motor(self,status):
        correct_id, self.act_pos, self.act_vel ,self.act_torque = status


class StatusHandler:
    def __init__(self):
        self.statuses_dict = {}
        self.new_stuff = 1

    def add_joint(self, can_id):
        self.statuses_dict[can_id] = MotorStatus(0, 0, 0, 0, can_id)

    def get_status(self, can_id):
        return self.statuses_dict[can_id].act_pos, self.statuses_dict[can_id].act_vel, self.statuses_dict[can_id].act_torque


    def update_joint(self, can_id, new_status):
        self.statuses_dict[can_id].update_from_motor(new_status)