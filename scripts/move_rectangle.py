#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import sys

class MoveRectangle(Node):
    def __init__(self):
        super().__init__('move_rectangle_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('linear_speed', 0.05)
        self.declare_parameter('angular_speed', 0.2)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

    def move_straight(self, distance):
        duration = distance / self.linear_speed
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.get_logger().info(f'Moving straight for {distance} m...')
        start = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start) < (duration * 1e9):
            self.publisher.publish(twist)
            time.sleep(0.1)
        self.publisher.publish(Twist())

    def rotate_in_place(self, angle):
        duration = abs(angle) / self.angular_speed
        twist = Twist()
        twist.angular.z = self.angular_speed if angle > 0 else -self.angular_speed
        self.get_logger().info(f'Rotating {math.degrees(angle)} deg...')
        start = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start) < (duration * 1e9):
            self.publisher.publish(twist)
            time.sleep(0.1)
        self.publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = MoveRectangle()

    side_lengths = [1.35, 1.35, 1.35, 1.35] # [1.35, 1.35, 1.35, 1.35]
    turn_angle = -math.pi / 2

    node.get_logger().info("Starting rectangle routine...")
    for i in range(4):
        node.move_straight(side_lengths[i])
        time.sleep(1.0)
        node.rotate_in_place(turn_angle)
        time.sleep(1.0)

    node.get_logger().info("Rectangle path complete. Exiting.")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()

