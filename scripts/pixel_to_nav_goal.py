#!/usr/bin/env python3
import sys
import argparse
import math
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft
from PIL import Image


class PixelToNavGoal(Node):
    def __init__(self, map_yaml_path: Path):
        super().__init__('pixel_to_nav_goal')

        # ── 지도 정보 로드 ──
        with open(map_yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        self.resolution = data['resolution']
        self.origin_x, self.origin_y = data['origin'][:2]  # yaw 무시
        img_path = map_yaml_path.parent / data['image']
        self.image_h = Image.open(img_path).height

        # ── Nav2 액션 클라이언트 ──
        self.nav_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # TF 버퍼 & 리스너 (yaw 유지용)
        self.tf_buf = Buffer()
        self.tfl = TransformListener(self.tf_buf, self)

        # 액션 핸들 저장용
        self._goal_handle = None
        self._result_future = None

    # ── 픽셀 → 실좌표 ──
    def pixel_to_real_world(self, px: int, py: int) -> tuple[float, float]:
        mx = self.origin_x + (px * self.resolution)
        my = self.origin_y + ((self.image_h - py) * self.resolution)
        return mx, my

    # ── Nav2 액션 전송 & 완료까지 대기 ──
    def send_nav_goal_and_wait(self, px: int, py: int) -> bool:
        # 1) 목표 좌표 변환
        mx, my = self.pixel_to_real_world(px, py)

        # 2) current yaw 추출 (map→base_link)
        try:
            tr = self.tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), Duration(seconds=0.5)
            )
            q = tr.transform.rotation
            _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            half = yaw / 2.0
            qx, qy, qz, qw = 0.0, 0.0, float(math.sin(half)), float(math.cos(half))
        except Exception:
            # TF를 못 가져오면 yaw=0 (정면)
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        # 3) 액션 서버 대기
        self.get_logger().info(f'→ Sending nav goal: ({mx:.2f}, {my:.2f})')
        if not self.nav_cli.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose 서버를 찾을 수 없습니다')
            return False

        # 4) Goal 메시지 생성
        goal_msg = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = mx
        ps.pose.position.y = my
        ps.pose.position.z = 0.0
        ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        goal_msg.pose = ps

        # 5) Goal 전송 및 콜백 설정
        send_goal_future = self.nav_cli.send_goal_async(
            goal_msg, feedback_callback=self._on_feedback
        )
        send_goal_future.add_done_callback(self._on_goal_response)

        while rclpy.ok() and self._goal_handle is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        #    만약 Goal이 거부(rejected)되었다면, 바로 리턴
        if self._goal_handle is None:
            self.get_logger().warn('Goal이 애초에 수락되지 않았습니다. (rejected)')
            return False

        # 7) 이제 GoalHandle이 수락(accepted)되었으니, “결과(Result)” future를 얻어서 기다린다.
        self._result_future = self._goal_handle.get_result_async()

        # 8) “네비실행 완료” 메시지가 올 때까지 spin
        while rclpy.ok() and not self._result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        # 7) 결과 확인
        result = self._result_future.result().result
        status = self._result_future.result().status
        # Nav2 STATUS_SUCCEEDED == 4
        if status == 4:
            self.get_logger().info('navigation succeeded')
            return True
        else:
            self.get_logger().warn(f'navigation failed (status: {status})')
            return False

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2가 목표를 거부했습니다')
            # 그래도 handle.done()을 True로 만들기 위해 설정
            self._goal_handle = goal_handle
            return
        self.get_logger().info('Goal accepted by Nav2')
        self._goal_handle = goal_handle

    def _on_feedback(self, feedback_msg):
        rem = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'… remaining: {rem:.2f} m')


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=int, required=True, help='pixel X')
    parser.add_argument('--y', type=int, required=True, help='pixel Y')
    parser.add_argument(
        '--map',
        type=Path,
        default=Path.home() / 'turtlebot3_ws/map/map.yaml',
        help='map.yaml 경로 (기본값: ~/turtlebot3_ws/map/map.yaml)',
    )
    args = parser.parse_args()

    if not args.map.exists():
        sys.stderr.write(f'[ERROR] map.yaml not found: {args.map}\n')
        sys.exit(1)

    node = PixelToNavGoal(args.map)

    # 목표 전송 및 완료까지 대기
    success = node.send_nav_goal_and_wait(args.x, args.y)

    # 결과 로그 출력 후 종료
    node.get_logger().info(f'PixelToNavGoal 프로세스 종료 (success={success})')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
