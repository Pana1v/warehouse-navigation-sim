#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class PatrolController(Node):
    def __init__(self):
        super().__init__('patrol_controller')
        self.publisher_ = self.create_publisher(Float64, '/box_cmd_vel', 10)

        # internal state
        self.direction = 1.0
        self.speed = 0.1
        self.time_since_toggle = 0.0
        self.toggle_interval = 15.0
        self.timer_period = 0.1  # 10 Hz

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Patrol Controller Started')

    def timer_callback(self):
        try:
            # Update time
            self.time_since_toggle += self.timer_period

            # Check if it's time to switch
            if self.time_since_toggle >= self.toggle_interval:
                self.direction *= -1.0
                self.time_since_toggle = 0.0
                self.get_logger().info(
                    f'Switching direction: {self.direction}')

            # Always publish
            msg = Float64()
            msg.data = float(self.direction * self.speed)
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PatrolController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in spin: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
