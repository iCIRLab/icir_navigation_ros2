#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


def main():
    parser = argparse.ArgumentParser(description='Send one Nav2 NavigateToPose goal (map frame).')
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('yaw', type=float, nargs='?', default=0.0, help='[rad]')
    parser.add_argument('--ns', default='ns1', help='Nav2 namespace (default: ns1)')
    parser.add_argument('--timeout', type=float, default=180.0, help='result timeout [s]')
    options, _ = parser.parse_known_args()

    rclpy.init()
    node = Node('icir_nav2_goal', parameter_overrides=[Parameter('use_sim_time', value=True)])
    action = '/%s/navigate_to_pose' % options.ns.strip('/')
    client = ActionClient(node, NavigateToPose, action)
    code = 1
    try:
        if not client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('%s did not become available in 30 s (is Nav2 active?)' % action)

        deadline = time.monotonic() + 5.0
        while node.get_clock().now().nanoseconds == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds == 0:
            raise RuntimeError('/clock is not available (is the simulation running?)')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = options.x
        goal.pose.pose.position.y = options.y
        goal.pose.pose.orientation.z = math.sin(options.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(options.yaw / 2.0)

        print('NavigateToPose -> (%.2f, %.2f, %.2f) on %s' % (options.x, options.y, options.yaw, action))
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('goal was not accepted')

        handle = future.result()
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=options.timeout)
        if not result.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
            raise RuntimeError('goal timed out; cancellation requested')

        status = result.result().status
        print('NavigateToPose result: status=%d (SUCCEEDED=%d)' % (status, GoalStatus.STATUS_SUCCEEDED))
        code = 0 if status == GoalStatus.STATUS_SUCCEEDED else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
