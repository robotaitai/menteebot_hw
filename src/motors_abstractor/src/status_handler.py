import time
from dataclasses import dataclass



@dataclass
class MotorStatus:
    act_pos: float
    act_vel: float
    act_torque: float
    des_pos: float
    mot_id: int
    t_stamp: 0

    def update_status(self, new_pos, new_vel, new_torque):
        self.act_pos = new_pos
        self.act_vel = new_vel
        self.act_torque = new_torque
        self.t_stamp = time.time()


    def update_from_motor(self,status):
        correct_id, self.act_pos, self.act_vel ,self.act_torque = status
        self.t_stamp = time.time()


class StatusHandler:
    def __init__(self, freq):
        self.statuses_dict = {}
        self.new_stuff = 1
        self.freq = freq

    def add_joint(self, can_id):
        self.statuses_dict[can_id] = MotorStatus(0, 0, 0, 0, can_id, time.time())

    def get_status(self, can_id):
        return self.statuses_dict[can_id].act_pos, self.statuses_dict[can_id].act_vel, self.statuses_dict[can_id].act_torque, self.statuses_dict[can_id].t_stamp


    def update_joint(self, can_id, new_status):
        # delta_pos = new_status[1] - self.statuses_dict[can_id].act_pos
        # print(f"{can_id} - delta pos = {delta_pos}" )
        # new_vel = (new_status[1] - self.statuses_dict[can_id].act_pos)*self.freq
        # print(f"{can_id} vel from mot {new_status[2]} calculated vel: {new_vel}")
        self.statuses_dict[can_id].update_from_motor(new_status)

        #TODO add dead motor recognition
