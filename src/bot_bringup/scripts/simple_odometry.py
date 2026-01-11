#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class SimpleOdometry(Node):
    def __init__(self):
        super().__init__('simple_odometry')

        # Robot Parameters (TurtleBot3 Waffle Pi)
        self.wheel_radius = 0.033  # Meters
        self.wheel_separation = 0.287  # Meters

        # State Variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.initialized = False

        # Subscribers
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.odom_publisher = self.create_publisher(
            Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Simple Odometry Node Started')

    def joint_state_callback(self, msg):
        # Identify wheel indices
        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
        except ValueError:
            return  # Joints not found

        current_left_pos = msg.position[left_idx]
        current_right_pos = msg.position[right_idx]

        if not self.initialized:
            self.prev_left_pos = current_left_pos
            self.prev_right_pos = current_right_pos
            self.initialized = True
            return

        # Calculate changes in wheel positions
        delta_left = current_left_pos - self.prev_left_pos
        delta_right = current_right_pos - self.prev_right_pos

        self.prev_left_pos = current_left_pos
        self.prev_right_pos = current_right_pos

        # Calculate displacements
        d_left = delta_left * self.wheel_radius
        d_right = delta_right * self.wheel_radius

        # Calculate robot displacement
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # Update Pose (using simplified integration)
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Use timestamp from JointState message for perfect synchronization
        current_time = msg.header.stamp

        # Publish Odometry
        self.publish_odometry(current_time)

    def publish_odometry(self, timestamp):
        # Quaternion from yaw
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        # Create Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Positions
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q

        self.odom_publisher.publish(odom_msg)

        # Broadcast Transform (Optional, usually handled by robot_state_publisher or plugin)
        # un-comment if you want this node to be the source of the odom->base_footprint tf
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
