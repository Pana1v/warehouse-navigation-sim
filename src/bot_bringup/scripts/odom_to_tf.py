#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    """
    Simple node that subscribes to /odom and publishes the odom->base_footprint TF transform.
    This is needed because Gazebo's DiffDrive plugin doesn't publish TF transforms by default.
    """

    def __init__(self):
        super().__init__('odom_to_tf')

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Odom to TF Node Started - Publishing odom->base_footprint transform')

    def odom_callback(self, msg):
        """
        Convert Odometry message to TF transform and broadcast it.
        Uses current time to avoid TF synchronization issues.
        """
        t = TransformStamped()

        # Header
        # Use the exact timestamp from the odometry message
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # Should be 'odom'
        t.child_frame_id = msg.child_frame_id    # Should be 'base_footprint'

        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
