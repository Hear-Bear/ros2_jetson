#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.status_pub = self.create_publisher(String, '/command_status', 10)
        self.orientation = 0  # 현재 방향 (도 단위)

    def publish_twist(self, linear=0.0, angular=0.0, duration=1.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.1)
        # 동작 후 정지
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def move(self, distance):
        """
        distance: 양수이면 전진, 음수이면 후진.
        단위: cm (cm를 m 단위로 변환)
        """
        speed = 0.2  # m/s
        direction = 1 if distance >= 0 else -1
        duration = abs(distance) / 100.0 / speed  # (cm -> m) / 속도
        self.get_logger().info(f"전진 {abs(distance)}cm (속도: {speed * direction}m/s, 시간: {duration:.2f}s)")
        self.publish_twist(linear=speed * direction, angular=0.0, duration=duration)
        

    def rotate(self, angle, speed="normal"):
        """
        angle: 양수이면 오른쪽(시계 방향) 회전, 음수이면 왼쪽(반시계방향) 회전
        speed: "quick", "normal", "slow"에 따라 각속도 조절
        """
        # 각속도 (rad/s) 기본값 설정
        angular_speed = 0.5
        if speed == "quick":
            angular_speed = 1.0
        elif speed == "slow":
            angular_speed = 0.3
        

        # 도 -> radian 변환
        angle_rad = angle * 3.14159265 / 180.0  
        duration = abs(angle_rad) / angular_speed
        self.get_logger().info(f"{speed.upper()} 속도로 {angle}° 회전 (각속도: {angular_speed} rad/s, 시간: {duration:.2f}s)")
        # 오른쪽 회전은 음수 각속도, 왼쪽은 양수 각속도
        direction = 1 if angle < 0 else -1
        self.publish_twist(linear=0.0, angular=angular_speed * direction, duration=duration)
        self.orientation = (self.orientation + angle) % 360
        self.get_logger().info(f"현재 방향: {self.orientation}°")

    def shake(self, angle, speed="normal"):
        """
        '안녕' 동작: 왼쪽으로 angle° 회전 -> 오른쪽으로 2*angle° 회전 -> 왼쪽으로 angle° 회전
        (이미 원위치로 복귀)
        """
        self.get_logger().info("좌우 흔들기 시작")
        self.rotate(-angle, speed)   # 왼쪽 회전
        self.rotate(2 * angle, speed)  # 오른쪽 회전
        self.rotate(-angle, speed)     # 왼쪽 회전하여 원위치 복귀
        self.get_logger().info("좌우 흔들기 종료")

    def oscillate(self, angle, speed="normal", times=2):
        """
        '잘자' 동작: 좌우로 천천히 흔들기. 동작 후 원래 상태 유지.
        """
        self.get_logger().info("천천히 좌우 흔들기 시작")
        for _ in range(times):
            self.rotate(-angle, speed)
            self.rotate(angle, speed)
        self.get_logger().info("천천히 좌우 흔들기 종료")

    def continuous_oscillate(self, angle, speed="normal", duration=3):
        """
        '춤춰줘' 동작: 일정 시간 동안 좌우로 반복 흔들기.
        동작 전의 방향으로 복귀하도록 보정.
        """
        self.get_logger().info("계속 좌우 흔들기 시작 (춤 동작)")
        start_orientation = self.orientation
        end_time = time.time() + duration
        direction = 1
        while time.time() < end_time:
            self.rotate(direction * angle, speed)
            direction *= -1
        # 원래 방향으로 복귀
        delta = (self.orientation - start_orientation) % 360
        if delta != 0:
            # 가장 짧은 회전 경로 선택 (예: 270° 대신 -90°)
            if delta > 180:
                delta = delta - 360
            self.get_logger().info(f"원위치 복귀: {delta}° 조정")
            self.rotate(-delta, speed)
        self.get_logger().info("계속 좌우 흔들기 종료")

def execute_command(cmd, robot):
    """
    명령어(cmd)를 해석하여 로봇(robot)의 해당 동작을 수행하고,
    동작 후 원위치 복귀 동작을 추가합니다.
    """
    cmd = cmd.strip()
    
    if cmd == "안녕":
        # shake 동작은 자체적으로 원위치 복귀
        robot.shake(30, speed="quick")
        
    elif cmd in ("놀아줘", "같이 놀자"):
        robot.get_logger().info("빠른 앞뒤 이동으로 놀기 시작")
        # 놀기 동작은 이미 앞뒤 이동으로 원위치 복귀
        for _ in range(3):
            robot.move(10)
            robot.move(-10)
        robot.get_logger().info("빠른 앞뒤 이동 종료")
        
    elif cmd == "앞으로 가":
        # 앞으로 30cm 이동 후 반대 이동하여 원위치 복귀
        robot.move(30)
        robot.move(-30)
        
    elif cmd == "뒤로 가":
        # 뒤로 30cm 이동 후 반대 이동하여 원위치 복귀
        robot.move(-30)
        robot.move(30)
        
    elif cmd == "너 별로야":
        # 90° 회전 후 반대 회전으로 원위치 복귀
        robot.rotate(90, speed="quick")
        robot.rotate(-90, speed="quick")
        
    elif cmd == "잘자":
        # oscillate 동작은 자체적으로 원위치 복귀
        robot.oscillate(30, speed="slow", times=2)
        
    elif cmd == "돌아봐":
        # 360° 회전은 원래 방향으로 복귀
        robot.rotate(360, speed="normal")
        
    elif cmd == "춤춰줘":
        # continuous_oscillate 내부에서 원위치 보정 실시
        robot.continuous_oscillate(30, speed="normal", duration=3)
        
    else:
        robot.get_logger().info("알 수 없는 명령어입니다.")
    
    status_msg = String()
    status_msg.data = 'done'
    robot.status_pub.publish(status_msg)
