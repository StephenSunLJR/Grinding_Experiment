#!/usr/bin/env python3
"""Persist bidirectional controller diagnostics outside the real-time loop."""
import csv
import datetime
import os
import time

import rospy
from variable_stiffness_controllers.msg import BidirectionalDiagnostics


class Recorder:
    def __init__(self):
        self.directory = os.path.expanduser(rospy.get_param(
            '~directory', os.path.join(os.environ.get('ROS_HOME', '~/.ros'), 'bidirectional_logs')))
        os.makedirs(self.directory, exist_ok=True)
        self.file = None
        self.writer = None
        self.run_id = None
        self.last_cycle = 0
        self.last_flush = time.monotonic()
        self.columns = ['stamp_sec', 'stamp_nsec']
        self.fields = []
        for name, kind in zip(BidirectionalDiagnostics.__slots__, BidirectionalDiagnostics._slot_types):
            if name == 'header':
                continue
            if '[' in kind:
                count = int(kind.split('[')[1].split(']')[0])
                self.columns.extend('{}_{}'.format(name, i + 1) for i in range(count))
            else:
                self.columns.append(name)
            self.fields.append((name, '[' in kind))
        self.subscriber = rospy.Subscriber(
            '/bidirectional_energy_flow_control/diagnostics',
            BidirectionalDiagnostics, self.callback, queue_size=10000, tcp_nodelay=True)
        rospy.on_shutdown(self.close)
        rospy.loginfo('Bidirectional diagnostics recorder ready: %s', self.directory)

    def close(self):
        if self.file:
            self.file.flush()
            self.file.close()
            self.file = None

    def callback(self, msg):
        try:
            if msg.run_id != self.run_id or msg.cycle <= self.last_cycle:
                self.close()
                stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                path = os.path.join(self.directory, 'bidirectional_{}_run{}.csv'.format(stamp, msg.run_id))
                self.file = open(path, 'x', newline='', buffering=1)
                self.writer = csv.writer(self.file)
                self.writer.writerow(self.columns)
                self.run_id = msg.run_id
                rospy.loginfo('Recording bidirectional diagnostics to %s', path)
            self.last_cycle = msg.cycle
            row = [msg.header.stamp.secs, msg.header.stamp.nsecs]
            for name, is_array in self.fields:
                value = getattr(msg, name)
                row.extend(value if is_array else [value])
            self.writer.writerow(row)
            if time.monotonic() - self.last_flush >= 0.25:
                self.file.flush()
                self.last_flush = time.monotonic()
        except (OSError, ValueError) as error:
            rospy.logfatal('Bidirectional diagnostics logging failed: %s', error)
            rospy.signal_shutdown('diagnostics logging failed')


if __name__ == '__main__':
    rospy.init_node('bidirectional_diagnostics_recorder')
    Recorder()
    rospy.spin()
