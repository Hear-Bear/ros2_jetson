#!/usr/bin/env python3
import os
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class PixelToNavGoal(Node):
    def __init__(self):
        super().__init__('pixel_to_nav_goal')

        # map.yaml 읽어서 해상도·원점·이미지 높이 로드
        self.map_yaml_path = os.path.expanduser('~/turtlebot3_ws/map/map.yaml')
        with open(self.map_yaml_path) as f:
            map_data = yaml.safe_load(f)
        self.resolution =map_data['resolution']
        self.origin_x, self.origin_y = map_data['origin'][0], map_data['origin'][1]
        self.image_path = os.path.join(os.path.dirname(self.map_yaml_path), map_data['image'])
        self.image_height = Image.open(self.image_path).height

        # 구독자: 'pixel_goal'
        self.sub = self.create_subscription(
            String,
            'pixel_goal',
            self.on_pixel_goal,
            10
        )
        # 퍼블리셔: 'goal_pose'
        self.pub = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )
        self.get_logger().info('PixelToNaviGoal ready.')

    def on_pixel_goal(self, msg):
        try:
            px, py = map(int, msg.data.split(','))
            self.send_goal(px, py)
        except Exception as e:
            self.get_logger().error(f"Invalid pixel goal: {msg.data} ({e})")

    def send_goal(self, px, py, orientation=(0.0, 0.0, 0.0, 1.0)):
        mx, my = self.pixel_to_real_world(px, py)

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = mx
        goal_msg.pose.position.y = my
        goal_msg.pose.position.z = 0.0

        goal_msg.pose.orientation.x = orientation[0]
        goal_msg.pose.orientation.y = orientation[1]
        goal_msg.pose.orientation.z = orientation[2]
        goal_msg.pose.orientation.w = orientation[3]

        self.pub.publish(goal_msg)
        self.get_logger().info(f'Navigation goal sent: ({mx:.2f}, {my:.2f})')

    def pixel_to_real_world(self, px, py):
        mx = self.origin_x + (px * self.resolution)
        my = self.origin_y + ((self.image_height - py) * self.resolution)
        return mx, my

def main(args=None):
    rclpy.init(args=args)
    node = PixelToNavGoal()

    # node.send_goal(pixel_x, pixel_y)
    # rclpy.spin_once(node, timeout_sec=2)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()