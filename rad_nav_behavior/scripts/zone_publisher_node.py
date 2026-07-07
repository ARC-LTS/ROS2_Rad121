#!/usr/bin/env python3
"""Reads known radiation source locations from a YAML file and publishes them
as a RadZoneArray on a configurable topic.  Publishes periodically so nodes
that start later still receive the data without needing transient-local QoS.

Supports two file formats:
  - zones list  (rad_nav_behavior/params/known_sources.yaml):
        zones:
          - label: "Source_A"
            x: 1.0
            y: 2.0

  - rad_field_server saved sources (rad_simulation/params/saved_sources.yaml):
        rad_field_server:
          ros__parameters:
            known_sources: [x, y, intensity, x, y, intensity, ...]

The ~/sync_from_field_server service reloads zones from the saved_sources_file
at runtime so surveillance mode can be updated after an experiment without
restarting any nodes.
"""

import sys
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_srvs.srv import Trigger

from geometry_msgs.msg import Point
from ros2_rad_map.msg import RadZone, RadZoneArray
from ros2_rad_map.srv import GoToZone, OrbitZone


class ZonePublisherNode(Node):
    def __init__(self):
        super().__init__('zone_publisher_node')

        self.declare_parameter('zones_file', 'none')
        self.declare_parameter('zones_topic', '/radiation_zones')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('saved_sources_file', '')

        zones_file         = self.get_parameter('zones_file').get_parameter_value().string_value
        zones_topic        = self.get_parameter('zones_topic').get_parameter_value().string_value
        rate_hz            = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._saved_sources_file = self.get_parameter('saved_sources_file').get_parameter_value().string_value

        if zones_file and zones_file.lower() != 'none':
            self._zones_msg = self._load_zones_file(zones_file)
        else:
            self.get_logger().info(
                'No zones_file set — starting empty. '
                'Call ~/sync_from_field_server to load zones at runtime.'
            )
            self._zones_msg = RadZoneArray()

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(RadZoneArray, zones_topic, qos)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

        self._sync_srv = self.create_service(
            Trigger, '~/sync_from_field_server', self._sync_from_field_server)

        self._go_to_zone_srv = self.create_service(
            GoToZone, '~/go_to_zone', self._go_to_zone)
        self._next_zone_srv = self.create_service(
            Trigger, '~/next_zone', self._next_zone)
        self._prev_zone_srv = self.create_service(
            Trigger, '~/prev_zone', self._prev_zone)

        self._current_zone_idx = 0

        self._clear_queue_client = self.create_client(
            Trigger, 'zone_explorer_node/clear_queue')
        self._orbit_zone_client = self.create_client(
            OrbitZone, 'zone_explorer_node/orbit_zone')

        self.get_logger().info(
            f'Zone publisher ready: {len(self._zones_msg.zones)} zone(s) '
            f'→ {zones_topic} @ {rate_hz} Hz'
        )
        if self._saved_sources_file:
            self.get_logger().info(
                f'sync_from_field_server service ready (reads "{self._saved_sources_file}")'
            )
        else:
            self.get_logger().warn(
                'saved_sources_file not set — ~/sync_from_field_server will have nothing to load.'
            )

    # -------------------------------------------------------------------------

    def _load_zones_file(self, path: str) -> RadZoneArray:
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except OSError as e:
            self.get_logger().fatal(f'Cannot open file "{path}": {e}')
            sys.exit(1)
        except yaml.YAMLError as e:
            self.get_logger().fatal(f'YAML parse error in "{path}": {e}')
            sys.exit(1)

        # Detect format by top-level key.
        if 'zones' in data:
            return self._parse_zones_format(data['zones'], path)
        elif 'rad_field_server' in data:
            return self._parse_saved_sources_format(data, path)
        else:
            self.get_logger().warn(f'Unrecognised format in "{path}" — no zones loaded.')
            return RadZoneArray()

    def _parse_zones_format(self, raw_zones: list, path: str) -> RadZoneArray:
        """Parse the zones-list format used by known_sources.yaml."""
        if not raw_zones:
            self.get_logger().warn(f'No zones found in "{path}".')
        msg = RadZoneArray()
        for entry in raw_zones:
            z = RadZone()
            z.label    = str(entry.get('label', ''))
            z.center   = Point()
            z.center.x = float(entry.get('x', 0.0))
            z.center.y = float(entry.get('y', 0.0))
            z.center.z = 0.0
            msg.zones.append(z)
            self.get_logger().info(f'  Loaded "{z.label}" at ({z.center.x:.2f}, {z.center.y:.2f})')
        return msg

    def _parse_saved_sources_format(self, data: dict, path: str) -> RadZoneArray:
        """Parse the flat triplet list written by rad_field_server (x, y, intensity, ...)."""
        try:
            triplets = data['rad_field_server']['ros__parameters']['known_sources']
        except (KeyError, TypeError):
            self.get_logger().warn(f'Could not find known_sources in "{path}".')
            return RadZoneArray()

        if not triplets or len(triplets) % 3 != 0:
            self.get_logger().warn(
                f'known_sources in "{path}" has {len(triplets)} values; expected a multiple of 3.'
            )
            return RadZoneArray()

        msg = RadZoneArray()
        for i, idx in enumerate(range(0, len(triplets), 3)):
            z = RadZone()
            z.label    = f'Source_{i + 1}'
            z.center   = Point()
            z.center.x = float(triplets[idx])
            z.center.y = float(triplets[idx + 1])
            z.center.z = 0.0
            msg.zones.append(z)
            self.get_logger().info(f'  Loaded "{z.label}" at ({z.center.x:.2f}, {z.center.y:.2f})')
        return msg

    # -------------------------------------------------------------------------

    def _sync_from_field_server(self, _req, response):
        if not self._saved_sources_file:
            response.success = False
            response.message = 'saved_sources_file parameter is not set.'
            return response

        self.get_logger().info(f'Syncing zones from "{self._saved_sources_file}"...')
        new_msg = self._load_zones_file(self._saved_sources_file)

        if not new_msg.zones:
            response.success = False
            response.message = f'No sources found in "{self._saved_sources_file}".'
            return response

        self._zones_msg = new_msg

        # Clear the zone explorer's queue so the fresh zones are picked up
        # immediately rather than being dropped by label-based deduplication.
        if self._clear_queue_client.service_is_ready():
            self._clear_queue_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn(
                'zone_explorer_node/clear_queue not available — '
                'new zones may be filtered by deduplication.'
            )

        # Publish immediately rather than waiting for the next timer tick.
        self._publish()

        response.success = True
        response.message = (
            f'Loaded {len(self._zones_msg.zones)} zone(s) from "{self._saved_sources_file}".'
        )
        self.get_logger().info(response.message)
        return response

    def _next_zone(self, _req, response):
        return self._step_zone(1, response)

    def _prev_zone(self, _req, response):
        return self._step_zone(-1, response)

    def _step_zone(self, direction: int, response):
        if not self._zones_msg.zones:
            response.success = False
            response.message = 'No zones loaded.'
            return response
        n = len(self._zones_msg.zones)
        self._current_zone_idx = (self._current_zone_idx + direction) % n
        zone = self._zones_msg.zones[self._current_zone_idx]
        return self._send_orbit(zone, response)

    def _go_to_zone(self, request, response):
        label = request.label.strip()
        match = next(
            ((i, z) for i, z in enumerate(self._zones_msg.zones) if z.label == label),
            None)

        if match is None:
            known = ', '.join(f'"{z.label}"' for z in self._zones_msg.zones) or 'none loaded'
            response.success = False
            response.message = f'Unknown label "{label}". Known zones: {known}'
            self.get_logger().warn(response.message)
            return response

        self._current_zone_idx, zone = match
        return self._send_orbit(zone, response)

    def _send_orbit(self, zone, response):
        if not self._orbit_zone_client.service_is_ready():
            response.success = False
            response.message = 'zone_explorer_node/orbit_zone service not available.'
            self.get_logger().warn(response.message)
            return response

        req = OrbitZone.Request()
        req.label = zone.label
        req.x = zone.center.x
        req.y = zone.center.y
        self._orbit_zone_client.call_async(req)

        n = len(self._zones_msg.zones)
        response.success = True
        response.message = (
            f'Going to "{zone.label}" ({self._current_zone_idx + 1}/{n}) '
            f'at ({zone.center.x:.2f}, {zone.center.y:.2f}).'
        )
        self.get_logger().info(response.message)
        return response

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
