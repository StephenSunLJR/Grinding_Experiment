#!/usr/bin/env python3
"""
A-DLS 实验诊断数据记录器
=========================

订阅 /adls/diagnostics 话题，将数据写入 CSV 文件以供后续分析。

用法:
    rosrun variable_stiffness_controllers record_diagnostics.py \
        _output:=/tmp/exp1_M4.csv _duration:=10.0

或在 launch 文件里:
    <node name="diag_recorder" pkg="variable_stiffness_controllers"
          type="record_diagnostics.py" output="screen">
        <param name="output" value="/tmp/exp1_M4.csv"/>
        <param name="duration" value="10.0"/>
    </node>
"""

import os
import sys
import csv
import rospy
from std_msgs.msg import Float64MultiArray


class DiagnosticRecorder:
    def __init__(self):
        rospy.init_node('adls_diag_recorder')

        self.output_path = rospy.get_param('~output', '/tmp/adls_diag.csv')
        self.duration = float(rospy.get_param('~duration', 30.0))
        # 等不到第一帧数据的最长墙钟时间，防止控制器没起来时无限挂起
        self.no_data_timeout = float(rospy.get_param('~no_data_timeout', 60.0))
        self.node_start = rospy.Time.now()
        self.start_time = None
        self.records = []
        self.received_first = False
        self.expected_cols = None

        # Ensure output dir exists
        out_dir = os.path.dirname(self.output_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        self.sub = rospy.Subscriber('/adls/diagnostics', Float64MultiArray,
                                    self.callback, queue_size=1000)

        rospy.loginfo(f'[recorder] Output:   {self.output_path}')
        rospy.loginfo(f'[recorder] Duration: {self.duration} s')
        rospy.loginfo(f'[recorder] Waiting for /adls/diagnostics ...')

        # Background timer to stop after duration
        self.timer = rospy.Timer(rospy.Duration(0.5), self.check_done)

    def callback(self, msg: Float64MultiArray):
        if self.start_time is None:
            self.start_time = rospy.Time.now()
            self.expected_cols = len(msg.data)
            self.received_first = True
            rospy.loginfo(f'[recorder] First message received ({self.expected_cols} cols). Recording starts.')

        if len(msg.data) != self.expected_cols:
            rospy.logwarn_throttle(5.0,
                f'[recorder] Inconsistent message size: got {len(msg.data)}, expected {self.expected_cols}')
            return

        self.records.append(list(msg.data))

    def check_done(self, _):
        if self.start_time is None:
            waited = (rospy.Time.now() - self.node_start).to_sec()
            if waited >= self.no_data_timeout:
                rospy.logerr(f'[recorder] No data on /adls/diagnostics after '
                             f'{waited:.0f}s (controller not publishing?). Aborting.')
                rospy.signal_shutdown('No data timeout')
            return
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed >= self.duration:
            rospy.loginfo(f'[recorder] Duration reached. Saving {len(self.records)} samples...')
            self.save_and_exit()

    def save_and_exit(self):
        try:
            with open(self.output_path, 'w', newline='') as f:
                writer = csv.writer(f)
                header = ['col_' + str(i) for i in range(self.expected_cols)]
                writer.writerow(header)
                writer.writerows(self.records)
            rospy.loginfo(f'[recorder] Saved {len(self.records)} samples to {self.output_path}')
        except Exception as e:
            rospy.logerr(f'[recorder] Save failed: {e}')
        rospy.signal_shutdown('Recording complete')


if __name__ == '__main__':
    try:
        node = DiagnosticRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
