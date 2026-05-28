#!/usr/bin/env python3
"""Reads known radiation source locations from a YAML file and publishes them
as a RadZoneArray on a configurable topic.  Publishes periodically so nodes
that start later still receive the data without needing transient-local QoS."""

import sys
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Point
from ros2_rad_map.msg import RadZone, RadZoneArray


class ZonePublisherNode(Node):
    def __init__(self):
        super().__init__('zone_publisher_node')

        self.declare_parameter('zones_file', '')
        self.declare_parameter('zones_topic', '/radiation_zones')
        self.declare_parameter('publish_rate_hz', 1.0)

        zones_file  = self.get_parameter('zones_file').get_parameter_value().string_value
        zones_topic = self.get_parameter('zones_topic').get_parameter_value().string_value
        rate_hz     = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        if not zones_file:
            self.get_logger().fatal('zones_file parameter is required but not set.')
            sys.exit(1)

        self._zones_msg = self._load_zones(zones_file)

        # Transient-local so any subscriber that connects after startup gets
        # the current set immediately without waiting for the next publish tick.
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(RadZoneArray, zones_topic, qos)

        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Zone publisher ready: {len(self._zones_msg.zones)} zone(s) '
            f'→ {zones_topic} @ {rate_hz} Hz'
        )

    def _load_zones(self, path: str) -> RadZoneArray:
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except OSError as e:
            self.get_logger().fatal(f'Cannot open zones file "{path}": {e}')
            sys.exit(1)
        except yaml.YAMLError as e:
            self.get_logger().fatal(f'YAML parse error in "{path}": {e}')
            sys.exit(1)

        raw_zones = data.get('zones', [])
        if not raw_zones:
            self.get_logger().warn(f'No zones found in "{path}" (expected a "zones:" list).')

        msg = RadZoneArray()
        for entry in raw_zones:
            z = RadZone()
            z.label    = str(entry.get('label', ''))
            z.center   = Point()
            z.center.x = float(entry.get('x', 0.0))
            z.center.y = float(entry.get('y', 0.0))
            z.center.z = 0.0
            msg.zones.append(z)
            self.get_logger().info(
                f'  Loaded zone "{z.label}" at ({z.center.x:.2f}, {z.center.y:.2f})'
            )

        return msg

    def _publish(self):
        self._zones_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._zones_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZonePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
