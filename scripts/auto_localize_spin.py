#!/usr/bin/env python3
"""
한-번만:
1) /reinitialize_global_localization (std_srvs/Empty) 서비스 요청
2) cmd_vel로 CCW 360° 회전 → AMCL 수렴 → 정지 후 종료
"""
import math, time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist


class AutoLocalizeSpin(Node):
    def __init__(self):
        super().__init__('auto_localize_spin')

        # 런치 파일에서 파라미터화 가능
        self.declare_parameter('service_name', '/reinitialize_global_localization')
        self.declare_parameter('angular_speed', 0.3)          # rad/s (+CCW)
        self.declare_parameter('spin_angle', 2 * math.pi)     # rad (360°)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.spin_angle   = self.get_parameter('spin_angle').get_parameter_value().double_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.cli  = self.create_client(Empty, self.service_name)
        self.pub  = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.twist = Twist(); self.twist.angular.z = self.angular_speed

        self._state = 'WAIT_SERVICE'
        self._t_start = None
        self._duration = abs(self.spin_angle / self.angular_speed)

        # 10 Hz 주기
        self.create_timer(0.1, self._step)
        self.get_logger().info('auto_localize_spin node started')

    #──────────────────────────────────────────────────────
    def _step(self):
        if self._state == 'WAIT_SERVICE':
            if self.cli.wait_for_service(timeout_sec=0.0):
                self.get_logger().info(f'service {self.service_name} available → call')
                self._call_future = self.cli.call_async(Empty.Request())
                self._state = 'CALLING'

        elif self._state == 'CALLING':
            if self._call_future.done():
                self._state = 'SPIN'
                self._t_start = self.get_clock().now()
                self.get_logger().info('Global localization done → start spin')

        elif self._state == 'SPIN':
            self.pub.publish(self.twist)
            if (self.get_clock().now() - self._t_start).nanoseconds * 1e-9 >= self._duration:
                self.pub.publish(Twist())          # 정지
                self.get_logger().info('Spin finished → node exits')
                rclpy.shutdown()

#────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = AutoLocalizeSpin()
    try:
        rclpy.spin(node)        # ← node 인자를 넘겨야 함
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()