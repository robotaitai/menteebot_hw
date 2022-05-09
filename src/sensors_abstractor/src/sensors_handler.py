import rospy, json
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Vector3
from dataclasses import dataclass
import numpy as np

NO_IMU = False
LAB_GRAVITY_ACCEL = True
GRAVITY = np.array([0, 0, -9.81])

@dataclass
class ImuData:
    time_stamp: float
    lin_a: [float, float, float]
    ang_v: [float, float, float]
    quat: [float, float, float, float]

    def update_status(self, new_imu_read):
        self.time_stamp = new_imu_read.header.stamp.to_nsec()
        self.ang_v = [new_imu_read.angular_velocity.x, new_imu_read.angular_velocity.y, new_imu_read.angular_velocity.z]
        self.lin_a = [new_imu_read.linear_acceleration.x, new_imu_read.linear_acceleration.y, new_imu_read.linear_acceleration.z]
        self.quat = [new_imu_read.orientation.x, new_imu_read.orientation.y, new_imu_read.orientation.z, new_imu_read.orientation.w]
        # print(new_imu_read)

        if LAB_GRAVITY_ACCEL:
            self.lin_a = self.lin_acc_corr(self.lin_a, self.quat)

    def json_status(self):
        res_dict = {'t_stamp': self.time_stamp, 'ang_v': self.ang_v, 'lin_a': self.lin_a, 'quat': self.quat}
        return json.dumps(res_dict)

    def lin_acc_corr(self, lin_acc, quat):
        lin_acc = np.array(lin_acc)
        # quat = np.array(quat)
        # rot_mat = Rotation.from_quat(quat).as_matrix()
        # inv_rot_mat = np.linalg.inv(rot_mat)
        # # acc_lab = np.array(lin_acc) + inv_rot_mat.dot(GRAVITY)
        # acc_lab = rot_mat.dot(lin_acc) + GRAVITY

        acc_lab = lin_acc - GRAVITY
        return acc_lab.tolist()


def convert_accel(int_val):
    return (float(int_val) / 32768.0) * 19.6


imu_recording = {}

class SensorsHandler:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/imu/data", Imu, self.update_imu, queue_size=10)
        self.publisher = rospy.Publisher("/simha/pelvis", String, queue_size=10)
        self.current_status = ImuData(0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])

    def update_imu(self, msg):
        #TODO gravity rotation matrix

        self.current_status.update_status(msg)
        if NO_IMU:
            self.current_status = ImuData(0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) #TODO

        # print(self.current_status.json_status)
        self.publisher.publish(self.current_status.json_status())

    def run(self):
        rospy.spin()


