from collections import deque  # 用于创建双端队列
import math
import numpy as np


class ControllerCFG:
    def __init__(self):
        # PID控制器参数
        self.lateral_KP = 0.9
        self.lateral_KI = 0.0
        self.lateral_KD = 0.0
        self.longitudinal_KP = 0.05
        self.longitudinal_KI = 0.0
        self.longitudinal_KD = 0.0
        # 限制
        self.max_angle_error = 100.0
        self.max_angle_derivative = 150.0
        self.max_steer_diff = 1.5
        self.recovery_lateral_KP = 0.6
        self.recovery_lateral_KD = 0.1
        self.recovery_longitudinal_KP = 0.1 


class VehiclePIDController(object):  # pylint: disable=too-few-public-methods
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal)
    to perform the low level control a vehicle from client side
    """
    def __init__(self):
        self.cfg = ControllerCFG()
        self._lon_controller = PIDLongitudinalController()  # 纵向PID控制器
        self._lat_controller = PIDLateralController()  # 横向PID控制器

    def run_step(self, target_speed, current_speed, current_pose, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint at a given target_speed.
        """
        throttle = self._lon_controller.run_step(target_speed, current_speed)  # 油门控制
        steering = self._lat_controller.run_step(current_pose, waypoint)  # 转向控制

        return steering, throttle


class PIDLongitudinalController(object):  # pylint: disable=too-few-public-methods
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """
    def __init__(self):
        self.cfg = ControllerCFG()
        self.error = 0.0  # 误差
        self.error_integral = 0.0  # 积分项误差
        self.error_derivative = 0.0  # 微分项误差

    def run_step(self, target_speed, current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations
        """
        previous_error = self.error
        self.error = target_speed - current_speed
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -40.0, 40.0)
        self.error_derivative = self.error - previous_error
        output = (self.cfg.longitudinal_KP * self.error +
                  self.cfg.longitudinal_KI * self.error_integral +
                  self.cfg.longitudinal_KD * self.error_derivative)
        return np.clip(output, -1.0, 1.0)


class PIDLateralController(object):  # pylint: disable=too-few-public-methods
    """
    PIDLateralController implements lateral control using a PID.
    """
    def __init__(self):
        self.cfg = ControllerCFG()
        self._e_buffer = deque(maxlen=10)
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0
        self.last_output = 0.0
        self.last_steer = 0.0

    def run_step(self, current_pose, waypoint):
        
        # if self._in_recovery_mode:  # 需要增加状态传递
        #     kp = self.cfg.recovery_lateral_KP
        #     kd = self.cfg.recovery_lateral_KD
        # else:
        #     kp = self.cfg.lateral_KP
        #     kd = self.cfg.lateral_KD
        """
        Estimate the steering angle of the vehicle based on the PID equations
        """
        v_begin = np.array(current_pose[:2])
        yaw = current_pose[2]
        v_end = np.array([v_begin[0] + math.cos(yaw), v_begin[1] + math.sin(yaw)])

        v_vec = v_end - v_begin  # 当前航向向量
        w_vec = np.array(waypoint) - v_begin  # 目标航向向量
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))  # 计算两向量夹角

        _cross = np.cross(v_vec, w_vec)  # 计算两向量叉乘，用于判断转向方向
        if _cross < 0:
            _dot *= -1.0

        previous_error = self.error
        present_error = _dot
        # restrict integral term to avoid integral windup
        error_integral = np.clip(self.error_integral + present_error, -400.0, 400.0)
        error_derivative = present_error - previous_error
        output = (self.cfg.lateral_KP * present_error +
                  self.cfg.lateral_KI * error_integral +
                  self.cfg.lateral_KD * error_derivative)
        steer = np.clip(output, -1.0, 1.0)
        if (abs(present_error) > math.radians(self.cfg.max_angle_error) or
                abs(error_derivative) > math.radians(self.cfg.max_angle_derivative) or
                abs(steer - self.last_steer) > self.cfg.max_steer_diff):
            return self.last_steer
        else:
            self.error = present_error
            self.error_integral = error_integral
            self.error_derivative = error_derivative
            self.last_output = output
            self.last_steer = steer
            return steer
