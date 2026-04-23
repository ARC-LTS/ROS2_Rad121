import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger, Empty
from nav2_msgs.srv import ClearEntireCostmap


class SystemResetNode(Node):
    def __init__(self):
        super().__init__('system_reset_node')

        cg = ReentrantCallbackGroup()

        self._slam_client = self.create_client(
            Empty, '/rtabmap/reset', callback_group=cg)
        self._global_cm_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
            callback_group=cg)
        self._local_cm_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
            callback_group=cg)
        self._battery_client = self.create_client(
            Trigger, '/battery_simulator/reset', callback_group=cg)
        self._heatmap_client = self.create_client(
            Trigger, '/reset_rad_heatmap', callback_group=cg)
        self._explore_client = self.create_client(
            Trigger, '/explore_node/reset', callback_group=cg)

        self.create_service(
            Trigger, '~/reset_slam', self._reset_slam_cb, callback_group=cg)
        self.create_service(
            Trigger, '~/reset_costmaps', self._reset_costmaps_cb, callback_group=cg)
        self.create_service(
            Trigger, '~/reset_battery', self._reset_battery_cb, callback_group=cg)
        self.create_service(
            Trigger, '~/reset_heatmap', self._reset_heatmap_cb, callback_group=cg)
        self.create_service(
            Trigger, '~/reset_explore', self._reset_explore_cb, callback_group=cg)
        self.create_service(
            Trigger, '~/reset_all', self._reset_all_cb, callback_group=cg)

        self.get_logger().info('SystemResetNode ready.')

    # -------------------------------------------------------------------------
    # Helpers — use threading.Event so we never nest spin loops inside callbacks
    # -------------------------------------------------------------------------

    def _call_async_wait(self, client, request, name, timeout=5.0):
        """Send an async request and block the calling thread (not the executor)."""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'{name}: service not available')
            return None

        event = threading.Event()
        result_holder = [None]

        def _done(future):
            result_holder[0] = future.result()
            event.set()

        future = client.call_async(request)
        future.add_done_callback(_done)

        if not event.wait(timeout=timeout):
            self.get_logger().error(f'{name}: call timed out')
            return None

        return result_holder[0]

    def _call_trigger(self, client, name):
        result = self._call_async_wait(client, Trigger.Request(), name)
        if result is None:
            return False, f'{name} failed'
        return result.success, result.message

    def _call_empty(self, client, name):
        result = self._call_async_wait(client, Empty.Request(), name)
        if result is None:
            return False, f'{name} failed'
        return True, f'{name} ok'

    def _call_clear_costmap(self, client, name):
        result = self._call_async_wait(client, ClearEntireCostmap.Request(), name)
        if result is None:
            return False, f'{name} failed'
        return True, f'{name} ok'

    # -------------------------------------------------------------------------
    # Individual service callbacks
    # -------------------------------------------------------------------------

    def _reset_slam_cb(self, _req, response):
        ok, msg = self._call_empty(self._slam_client, 'reset_slam')
        if ok:
            self.get_logger().info('SLAM map cleared.')
        response.success = ok
        response.message = msg
        return response

    def _reset_costmaps_cb(self, _req, response):
        ok1, msg1 = self._call_clear_costmap(self._global_cm_client, 'global_costmap')
        ok2, msg2 = self._call_clear_costmap(self._local_cm_client, 'local_costmap')
        response.success = ok1 and ok2
        response.message = f'{msg1}; {msg2}'
        if response.success:
            self.get_logger().info('Costmaps cleared.')
        return response

    def _reset_battery_cb(self, _req, response):
        ok, msg = self._call_trigger(self._battery_client, 'reset_battery')
        if ok:
            self.get_logger().info('Battery reset.')
        response.success = ok
        response.message = msg
        return response

    def _reset_heatmap_cb(self, _req, response):
        ok, msg = self._call_trigger(self._heatmap_client, 'reset_heatmap')
        if ok:
            self.get_logger().info('Radiation heatmap reset.')
        response.success = ok
        response.message = msg
        return response

    def _reset_explore_cb(self, _req, response):
        ok, msg = self._call_trigger(self._explore_client, 'reset_explore')
        if ok:
            self.get_logger().info('Explorer reset.')
        response.success = ok
        response.message = msg
        return response

    # -------------------------------------------------------------------------
    # Master reset
    # -------------------------------------------------------------------------

    def _reset_all_cb(self, _req, response):
        results = []

        ok, msg = self._call_trigger(self._explore_client, 'reset_explore')
        results.append(('explore', ok, msg))

        ok, msg = self._call_empty(self._slam_client, 'reset_slam')
        results.append(('slam', ok, msg))

        ok1, msg1 = self._call_clear_costmap(self._global_cm_client, 'global_costmap')
        ok2, msg2 = self._call_clear_costmap(self._local_cm_client, 'local_costmap')
        results.append(('costmaps', ok1 and ok2, f'{msg1}; {msg2}'))

        ok, msg = self._call_trigger(self._heatmap_client, 'reset_heatmap')
        results.append(('heatmap', ok, msg))

        ok, msg = self._call_trigger(self._battery_client, 'reset_battery')
        results.append(('battery', ok, msg))

        all_ok = all(r[1] for r in results)
        summary = ', '.join(f'{r[0]}:{"ok" if r[1] else "FAIL"}' for r in results)
        response.success = all_ok
        response.message = summary
        self.get_logger().info(f'reset_all: {summary}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SystemResetNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
