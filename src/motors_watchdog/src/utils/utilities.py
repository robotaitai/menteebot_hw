from dataclasses import dataclass

@dataclass
class MotorLimits:
    min_pos: float
    max_pos: float
    max_vel: float
    max_torque: float

@dataclass
class RosParams:
    subscribing_to: str
    publishing_as: str
    publishing_err: str