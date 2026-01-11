#!/usr/bin/env python3
"""
Warehouse Navigation Action Server

This node provides an action server that can:
1. Load maps dynamically
2. Navigate to named waypoints (e.g., "loading_dock", "aisle_1")
3. Navigate to specific (x, y, theta) coordinates

It wraps Nav2's NavigateToPose action and provides a higher-level interface.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from std_srvs.srv import Empty

from action_msgs.msg import GoalStatus

import math
import yaml
import os


def euler_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle (radians) to quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WarehouseNavServer(Node):
    """
    Action server for warehouse navigation tasks.

    Services:
        - /warehouse_nav/load_map: Load a new map file
        - /warehouse_nav/go_to_waypoint: Navigate to a named waypoint

    This node maintains a dictionary of named waypoints and provides
    a simple interface for navigating between them.
    """

    def __init__(self):
        super().__init__('warehouse_nav_server')

        self.callback_group = ReentrantCallbackGroup()

        # Predefined waypoints in the warehouse (x, y, yaw in radians)
        # These should be calibrated to your specific map
        self.waypoints = {
            'home': (-2.0, 0.0, 0.0),
            'loading_dock': (0.0, 0.0, 0.0),
            'aisle_1_start': (-1.0, 2.0, 1.57),
            'aisle_1_end': (-1.0, -2.0, -1.57),
            'aisle_2_start': (1.0, 2.0, 1.57),
            'aisle_2_end': (1.0, -2.0, -1.57),
            'charging_station': (-3.0, 0.0, 3.14),
        }

        # Nav2 NavigateToPose action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Service to load a new map
        self.load_map_client = self.create_client(
            LoadMap,
            '/map_server/load_map',
            callback_group=self.callback_group
        )

        # Service to clear costmaps
        self.clear_costmaps_client = self.create_client(
            Empty,
            '/global_costmap/clear_entirely_global_costmap',
            callback_group=self.callback_group
        )

        # Services this node provides
        self.create_service(
            LoadMap,
            '/warehouse_nav/load_map',
            self.load_map_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Warehouse Navigation Server initialized')
        self.get_logger().info(
            f'Available waypoints: {list(self.waypoints.keys())}')

        # Current navigation goal handle
        self._current_goal_handle = None

    def load_map_callback(self, request, response):
        """
        Service callback to load a new map.

        This wraps the Nav2 map_server's load_map service.
        """
        self.get_logger().info(f'Loading map: {request.map_url}')

        if not self.load_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map server load_map service not available')
            response.result = 1  # Error
            return response

        future = self.load_map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            response.result = future.result().result
            if response.result == 0:
                self.get_logger().info('Map loaded successfully')
                # Clear costmaps after loading new map
                self.clear_costmaps()
            else:
                self.get_logger().error(
                    f'Failed to load map: result={response.result}')
        else:
            self.get_logger().error('Map loading timed out')
            response.result = 1

        return response

    def clear_costmaps(self):
        """Clear both global and local costmaps."""
        if self.clear_costmaps_client.wait_for_service(timeout_sec=2.0):
            self.clear_costmaps_client.call_async(Empty.Request())
            self.get_logger().info('Costmaps cleared')

    def navigate_to_pose(self, x: float, y: float, yaw: float) -> bool:
        """
        Navigate to a specific pose.

        Args:
            x: X coordinate in map frame
            y: Y coordinate in map frame  
            yaw: Orientation in radians

        Returns:
            True if navigation succeeded, False otherwise
        """
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = euler_to_quaternion(yaw)

        self.get_logger().info(
            f'Navigating to: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')

        send_goal_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        self._current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            return True
        else:
            self.get_logger().warn(
                f'Navigation failed with status: {result.status}')
            return False

    def navigate_to_waypoint(self, waypoint_name: str) -> bool:
        """
        Navigate to a named waypoint.

        Args:
            waypoint_name: Name of the waypoint (e.g., 'loading_dock')

        Returns:
            True if navigation succeeded, False otherwise
        """
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'Unknown waypoint: {waypoint_name}')
            self.get_logger().info(
                f'Available waypoints: {list(self.waypoints.keys())}')
            return False

        x, y, yaw = self.waypoints[waypoint_name]
        self.get_logger().info(f'Going to waypoint: {waypoint_name}')
        return self.navigate_to_pose(x, y, yaw)

    def add_waypoint(self, name: str, x: float, y: float, yaw: float):
        """Add or update a waypoint."""
        self.waypoints[name] = (x, y, yaw)
        self.get_logger().info(
            f'Added waypoint: {name} at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')


def main(args=None):
    rclpy.init(args=args)

    node = WarehouseNavServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
