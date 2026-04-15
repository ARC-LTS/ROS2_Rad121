#!/usr/bin/env python3
"""
Starts ros2 bag record as a child process and provides a /rosbag/stop service
(std_srvs/Trigger) that stops it gracefully.

On stop:
  - SIGTERM is sent to the recorder process only (not the whole process group).
  - The node waits indefinitely for the recorder to exit so the bag is fully
    flushed and finalized before shutdown — no timeout, no SIGKILL.

Usage (from any terminal while recording):
    ros2 service call /rosbag/stop std_srvs/srv/Trigger '{}'
"""
import os
import signal
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RosbagStopper(Node):
    def __init__(self):
        super().__init__('rosbag_stopper')

        self.declare_parameter('exclude_regex', '')
        exclude_regex = self.get_parameter('exclude_regex').value

        cmd = ['ros2', 'bag', 'record', '--all']
        if exclude_regex:
            cmd += ['--exclude', exclude_regex]

        self._recorder = subprocess.Popen(cmd)
        self.get_logger().info(
            f'Started ros2 bag record (PID {self._recorder.pid})'
        )

        self._stopping = False
        self._srv = self.create_service(Trigger, '/rosbag/stop', self._stop_cb)
        self.get_logger().info(
            "Ready. Call 'ros2 service call /rosbag/stop std_srvs/srv/Trigger \"{}\"' to stop."
        )

    def _stop_cb(self, _request, response):
        if self._stopping:
            response.success = False
            response.message = 'Stop already in progress — waiting for bag to finalize.'
            return response

        self._stopping = True
        pid = self._recorder.pid
        response.success = True
        response.message = (
            f'Sent SIGTERM to recorder (PID {pid}). '
            'Waiting for bag to finalize — this may take a while for large bags.'
        )
        self.get_logger().info(response.message)

        # Wait in a background thread so the service response is delivered first.
        threading.Thread(target=self._wait_for_recorder, daemon=True).start()
        return response

    def _wait_for_recorder(self):
        try:
            os.kill(self._recorder.pid, signal.SIGTERM)
        except ProcessLookupError:
            self.get_logger().warn('Recorder process already gone.')

        # Block until the recorder fully exits — no timeout, no SIGKILL.
        # rclcpp (used by ros2 bag record) handles SIGTERM and flushes the bag
        # before exiting, so this will return once finalization is complete.
        self._recorder.wait()
        self.get_logger().info('Bag finalized. Shutting down.')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = RosbagStopper()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
