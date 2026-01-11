#!/usr/bin/env python3
"""
Ground Truth Odometry Publisher
Converts Gazebo ground truth pose to odometry for perfect localization testing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__('ground_truth_odom')

        # Subscribe to Gazebo ground truth pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/model/simple_bot/pose',
            self.pose_callback,
            10
        )

        # Publish odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)

        # TF broadcaster for odom->base_footprint
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Ground truth odometry node started')

    def pose_callback(self, msg: PoseStamped):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Copy pose
        odom.pose.pose = msg.pose

        # Publish odometry
        self.odom_pub.publish(odom)

        # Publish TF (odom->base_footprint)
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
