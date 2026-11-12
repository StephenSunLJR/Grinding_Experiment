#!/usr/bin/env python3
"""
Gazebo 外力注入节点
==================

通过 /gazebo/apply_body_wrench 服务向 Franka 末端 link 真实注入外力。
取代之前 A_DLS.cpp 内只计算不应用的"虚拟外力"——现在 Gazebo 物理引擎
会实际感受到外力，GMO 观测器也能正确观测到。

参数 (rosparam ~):
    start_time   [s]  力作用起始时间（节点启动后相对计时）
    end_time     [s]  力作用结束时间
    magnitude    [N]  力幅值（绝对值, 实际方向由 direction 决定）
    direction    int  0=+X, 1=+Y, 2=+Z (将取负号: -X/-Y/-Z 压下方向)
    ramp_time    [s]  上升/下降沿持续时间 (默认 0.2)
    body_name    str  Gazebo 中 link 完整路径 (默认 panda::panda_link7)
    rate         Hz   服务调用频率 (默认 20)
"""

import os
import sys
import rospy
from gazebo_msgs.srv import (
    ApplyBodyWrench, ApplyBodyWrenchRequest,
    BodyRequest, BodyRequestRequest
)
from geometry_msgs.msg import Wrench, Vector3


class ForceInjector:
    def __init__(self):
        rospy.init_node('adls_force_injector', anonymous=True)

        self.start_time = float(rospy.get_param('~start_time', 3.0))
        self.end_time = float(rospy.get_param('~end_time', 5.0))
        self.magnitude = float(rospy.get_param('~magnitude', 25.0))
        self.direction = int(rospy.get_param('~direction', 2))
        self.direction2 = int(rospy.get_param('~direction2', -1))  # >=0: 与 direction 合成 45°
        self.ramp_time = float(rospy.get_param('~ramp_time', 0.2))
        self.body_name = rospy.get_param('~body_name', 'panda::panda_link7')
        self.rate_hz = float(rospy.get_param('~rate', 20.0))

        if self.magnitude <= 1e-6:
            rospy.loginfo('[force_injector] magnitude = 0, exiting (no force needed)')
            return

        rospy.loginfo(f'[force_injector] body  = {self.body_name}')
        rospy.loginfo(f'[force_injector] force = {self.magnitude} N in direction {self.direction}')
        rospy.loginfo(f'[force_injector] window = [{self.start_time}, {self.end_time}] s')

        # 等待并连接 Gazebo 服务
        rospy.loginfo('[force_injector] Waiting for /gazebo/apply_body_wrench ...')
        rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=60.0)
        self.apply_wrench = rospy.ServiceProxy(
            '/gazebo/apply_body_wrench', ApplyBodyWrench)
        rospy.loginfo('[force_injector] Waiting for /gazebo/clear_body_wrenches ...')
        rospy.wait_for_service('/gazebo/clear_body_wrenches', timeout=60.0)
        self.clear_wrenches = rospy.ServiceProxy(
            '/gazebo/clear_body_wrenches', BodyRequest)
        rospy.loginfo('[force_injector] Connected.')

        self.t_initial = None
        self.was_in_window = False
        self.run()

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        period = 1.0 / self.rate_hz

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.t_initial is None:
                self.t_initial = now

            t_elapsed = (now - self.t_initial).to_sec()
            in_window = (self.start_time <= t_elapsed <= self.end_time)

            if in_window:
                ramp = 1.0
                dt_in = t_elapsed - self.start_time
                if dt_in < self.ramp_time:
                    ramp = dt_in / self.ramp_time
                dt_out = self.end_time - t_elapsed
                if dt_out < self.ramp_time and dt_out > 0:
                    ramp = min(ramp, dt_out / self.ramp_time)

                self._send_wrench(self.magnitude * ramp, period * 2.0)

                if not self.was_in_window:
                    rospy.loginfo(f'[force_injector] Force ON at t={t_elapsed:.2f}s')
                self.was_in_window = True

            elif self.was_in_window:
                # 退出力窗口: 清除残余 wrench
                try:
                    req = BodyRequestRequest()
                    req.body_name = self.body_name
                    self.clear_wrenches(req)
                    rospy.loginfo(f'[force_injector] Force OFF at t={t_elapsed:.2f}s')
                except rospy.ServiceException as e:
                    rospy.logwarn(f'[force_injector] clear failed: {e}')
                self.was_in_window = False

            rate.sleep()

    def _send_wrench(self, force_mag: float, duration_s: float):
        # 每次调用前清除上次的（避免堆叠）
        try:
            req = BodyRequestRequest()
            req.body_name = self.body_name
            self.clear_wrenches(req)
        except rospy.ServiceException:
            pass

        # 应用新力 (沿 -direction 方向, 模拟"按压"动作)
        req = ApplyBodyWrenchRequest()
        req.body_name = self.body_name
        req.reference_frame = ''  # world frame
        req.wrench = Wrench()
        force_vec = [0.0, 0.0, 0.0]
        if self.direction2 >= 0:                 # 45° 斜向: 按 √2 拆到两轴
            comp = -force_mag / (2.0 ** 0.5)
            force_vec[self.direction] = comp
            force_vec[self.direction2] = comp
        else:
            force_vec[self.direction] = -force_mag   # negative = pressing
        req.wrench.force = Vector3(*force_vec)
        req.wrench.torque = Vector3(0, 0, 0)
        req.start_time = rospy.Time(0, 0)  # immediate
        req.duration = rospy.Duration(duration_s)
        try:
            self.apply_wrench(req)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, f'[force_injector] apply failed: {e}')


if __name__ == '__main__':
    try:
        ForceInjector()
    except rospy.ROSInterruptException:
        pass
