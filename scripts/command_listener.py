#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import command_activate as ca

class CommandListener(Node):
    def __init__(self):
        super().__init__('command_listener')
        # FastAPI와 동일한 토픽 이름을 지정하세요
        self.subscription = self.create_subscription(
            String,
            'command',        # 혹은 'robot_command' 등 퍼블리셔와 맞춰주세요
            self.callback,
            10
        )
        self.controller = ca.TurtleBotController()
        self.command_dict = {
            "안녕": 1,
            "놀아줘": 1,
            "앞으로 가": 1,
            "뒤로 가": 1,
            "너 별로야": 1,
            "잘자": 1,
            "돌아봐": 1,
            "춤춰줘": 1
        }

    def callback(self, msg: String):
        cmd = msg.data.strip()
        if cmd in self.command_dict:
            self.get_logger().info(f'Received command: "{cmd}"')
            ca.execute_command(cmd, self.controller)
        else:
            self.get_logger().warn(f'Unknown command: "{cmd}"')

def main(args=None):
    rclpy.init(args=args)
    node = CommandListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
