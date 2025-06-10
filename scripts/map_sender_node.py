#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import base64
import json
from pathlib import Path
import sys

class MapSenderNode(Node):
    def __init__(self):
        super().__init__('map_sender_node')
        self.pub = self.create_publisher(String, '/map_data_json', 10)
        # 1초 후에 publish_map()을 한 번 호출
        self.timer = self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        map_dir   = Path("~/turtlebot3_ws/map").expanduser()
        yaml_path = map_dir / "map.yaml"
        png_path  = map_dir / "map.png"

        if not yaml_path.exists() or not png_path.exists():
            self.get_logger().warn("Map 파일이 존재하지 않습니다.")
            return

        # 1) YAML 메타데이터 읽기
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)

        # 2) PNG 이미지 base64 인코딩
        with open(png_path, 'rb') as f:
            img_b64 = base64.b64encode(f.read()).decode('utf-8')

        # 3) JSON 페이로드 구성 (image_type 필드 추가)
        payload = {
            "map_name":   "map",
            "resolution": yaml_data["resolution"],
            "origin":     yaml_data["origin"],
            "image_type": "png",
            "image_data": img_b64,
        }

        # 4) Publish
        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)
        self.get_logger().info("지도 데이터를 /map_data_json 토픽으로 전송했습니다.")

        # 5) 한 번만 실행하고 종료하도록 타이머 취소 + shutdown
        self.timer.cancel()
        self.destroy_node()
        
        

def main():
    rclpy.init()
    node = MapSenderNode()
    rclpy.spin(node)
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()
