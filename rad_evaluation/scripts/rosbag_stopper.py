#!/usr/bin/env python3
"""
Provides a /rosbag/stop service (std_srvs/Trigger) that gracefully stops the
rosbag recording launched alongside it.

Usage (from any terminal while recording):
    ros2 service call /rosbag/stop std_srvs/srv/Trigger '{}'

The service returns immediately, then waits 1 second before sending SIGINT to
the process group so that ros2 bag record has time to flush and compress the
final chunk before exiting.
"""
import os
import signal
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RosbagStopper(Node):
    def __init__(self):
        super().__init__('rosbag_stopper')
        self._srv = self.create_service(Trigger, '/rosbag/stop', self._stop_cb)
        self.get_logger().info(
            "Ready. Call 'ros2 service call /rosbag/stop std_srvs/srv/Trigger \"{}\"' to stop."
        )

    def _stop_cb(self, _request, response):
        response.success = True
        response.message = 'Stopping rosbag recording in 1 s...'
        self.get_logger().info('Stop requested — sending SIGINT to process group in 1 s')
        # Delay so the service response is delivered before the process is killed.
        self.create_timer(1.0, self._send_sigint)
        return response

    def _send_sigint(self):
        self.get_logger().info('Sending SIGINT — bag will flush and compress before exit.')
        os.killpg(os.getpgid(os.getpid()), signal.SIGINT)


def main():
    rclpy.init()
    node = RosbagStopper()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
