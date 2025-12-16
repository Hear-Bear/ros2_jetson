#!/usr/bin/env python3
"""
audio_doa_rotate_node.py

소리 감지 → 백그라운드 녹음·분류·회전·이동 처리 → Nav2 결과 콜백으로 busy 해제
"""

import math
import time
import threading
import sys
from pathlib import Path
import os
import pycuda.autoinit 
from pycuda.autoinit import context   # 생성된 인스턴스를 가져옴
_CUDA_CTX = context     

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import String

import numpy as np
import sounddevice as sd
import soundfile as sf

import cv2

sys.path.append(os.path.expanduser("~/turtlebot3_ws"))
from usb_4_mic_array.tuning import Tuning
import usb.core, usb.util

sys.path.append(os.path.expanduser("~/turtlebot3_ws/audio_project"))
from trt_execute import classify_noise

from sound_goal_utils import select_px_py
from pixel_to_nav_goal import PixelToNavGoal
from nav2_msgs.action import NavigateToPose
import tf_transformations as tft
from action_msgs.msg import GoalStatus

from pathlib import Path
from datetime import datetime
import json, os


class AudioDoaRotateNode(Node):
    def __init__(self):
        super().__init__('audio_doa_rotate')

        # ── 파라미터 ────────────────────────────
        self.threshold_db    = -20.0           # dBFS 임계값
        self.record_sec      = 5.0             # 녹음 길이(s)
        self.sample_rate     = 16000           # Hz
        self.ang_vel         = 0.5             # rad/s
        self.angle_tolerance = math.radians(5) # 회전 스킵 허용 오차(rad)
        self.busy            = False           # 처리 중 플래그
        
        # ── 카메라 설정 파라미터 추가 ─────────────────── ← 추가된 부분
        self.camera_index = 0  # 사용할 카메라 장치 번호 (보통 0)
        self.image_save_path = Path('~/turtlebot3_ws/captured_images').expanduser()
        
        # 마지막으로 녹음된 파일 경로 저장용
        self.last_wav = None

        # ── ReSpeaker 설정 ─────────────────────
        idx = next((i for i,d in enumerate(sd.query_devices())
                    if 'ReSpeaker' in d['name']), None)
        if idx is None:
            raise RuntimeError('ReSpeaker device not found')
        sd.default.device = (idx, idx)
        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if dev is None:
            raise RuntimeError('ReSpeaker USB not found')
        self.tuning = Tuning(dev)

        # ── ROS 인터페이스 ─────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer   = self.create_timer(0.1, self.detect_loop)
        
        # ── Audio 상태 발행 토픽 추가 ────────────────────
        # "idle" / "navigating" 문자열을 main_monitor로 보냅니다.
        self.audio_status_pub = self.create_publisher(String, '/audio_status', 10)
        
        # 처음에는 듣기 상태(idle)이므로 한 번 발행
        self.audio_status_pub.publish(String(data="idle"))
        
        self.noise_pub = self.create_publisher(String, '/noise', 10)


        # TF 버퍼 & 리스너
        self.tf_buf = Buffer()
        self.tfl    = TransformListener(self.tf_buf, self)

        # map→base_link TF 준비 대기
        self.get_logger().info('Waiting for map→base_link TF...')
        self.tf_buf.can_transform(
            'map', 'base_link',
            rclpy.time.Time(),
            timeout=Duration(seconds=5.0)
        )
        self.get_logger().info('map→base_link TF is ready')
        
        map_yaml = Path.home() / "turtlebot3_ws/map/map.yaml"

        # 픽셀→맵 변환기
        self.nav_converter = PixelToNavGoal(map_yaml)

        # Nav2 액션 클라이언트
        self.nav_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 로그용 JSON
        self.log_path       = Path('~/turtlebot3_ws/noise_data/noise.json').expanduser()
        self.noise_category = None
        self.dest_category  = None

        self.get_logger().info(f'Audio-DOA monitor ready (device {idx})')

    def detect_loop(self):
        if self.busy:
            return

        # RMS 측정
        frame = sd.rec(int(0.1*self.sample_rate),
                       samplerate=self.sample_rate,
                       channels=1, dtype='int16', blocking=True)
        rms = math.sqrt((frame.astype(float)**2).mean())
        db  = 20*math.log10(rms/32768.0 + 1e-12)
        self.get_logger().debug(f'RMS: {db:.1f} dB')

        if db >= self.threshold_db:
            self.busy = True
            self.audio_status_pub.publish(String(data="detecting"))
            self.get_logger().info(f'▶ {db:.1f} dB detected – starting thread')
            threading.Thread(target=self.handle_event, daemon=True).start()

    def handle_event(self):
        # 1) 녹음 & 분류
        wav_file = self._record()
        
        # ── 녹음 파일 경로를 인스턴스 변수에 저장 ← 추가
        self.current_wav_file = wav_file
        
        try:
            _CUDA_CTX.push()     
            tidx, cat = classify_noise(str(wav_file))
            _CUDA_CTX.pop() 
            self.noise_category = cat
            self.get_logger().info(f'  · classified: {cat} (class {tidx})')
        except Exception as e:
            _CUDA_CTX.pop()
            self.noise_category = -1
            self.get_logger().warn(f'  · classify failed: {e}')

        # 2) DOA 회전
        doa = self._get_doa()
        if doa is None:
            self.get_logger().warn('DOA failed')
            self._finish_event()
            return

        self._rotate(doa)
        time.sleep(0.2)

        # 3) 픽셀 후보 선택
        sel = select_px_py(self.tf_buf)
        if not sel:
            self.get_logger().info('no candidate in tolerance')
            self._finish_event()
            return

        dest_cat, px, py = sel
        self.dest_category = dest_cat
        self.get_logger().info(f'selected pixel: ({px},{py}), category={dest_cat}')
        
        # Nav2에 목표를 보내기 바로 직전에 calling
        self.audio_status_pub.publish(String(data="navigating"))

        # 4) Nav2 액션 전송
        self._send_nav(px, py)

    def _record(self) -> Path:
        frames = sd.rec(int(self.record_sec*self.sample_rate),
                        samplerate=self.sample_rate,
                        channels=1, dtype='int16')
        sd.wait()
        save_dir = Path('~/turtlebot3_ws/recordings').expanduser()
        save_dir.mkdir(parents=True, exist_ok=True)
        out = save_dir / f'noise_{int(time.time())}.wav'
        sf.write(out, frames, self.sample_rate)
        self.get_logger().info(f'  · recorded to {out}')
        return out

    def _get_doa(self) -> float|None:
        ang = self.tuning.direction
        if ang is None:
            return None
        self.get_logger().info(f'  · DOA = {ang:.1f}°')
        return float(ang)

    def _rotate(self, doa_deg: float):
        # 최소 회전 방향·크기
        if doa_deg <= 180:
            diff, dir_ = math.radians(doa_deg), +1
        else:
            diff, dir_ = math.radians(360-doa_deg), -1

        if diff < self.angle_tolerance:
            self.get_logger().info('rotation skip (within tol)')
            return

        duration = diff/self.ang_vel
        twist = Twist(); twist.angular.z = dir_*self.ang_vel
        self.cmd_pub.publish(twist)

        # duration 동안 spin_once 로 콜백 처리
        time.sleep(duration)

        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info('rotation complete')

    def _send_nav(self, px: int, py: int):
        mx,my = self.nav_converter.pixel_to_real_world(px, py)
        if not self.nav_cli.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose server unavailable')
            self._finish_event()
            return

        # 현재 yaw 유지
        tr = self.tf_buf.lookup_transform(
            'map','base_link',
            rclpy.time.Time(),
            Duration(seconds=0.2)
        )
        q = tr.transform.rotation
        _,_,yaw = tft.euler_from_quaternion([q.x,q.y,q.z,q.w])
        half = yaw/2
        qx,qy,qz,qw = 0.0,0.0,math.sin(half),math.cos(half)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = mx
        goal.pose.pose.position.y = my
        goal.pose.pose.orientation = Quaternion(x=qx,y=qy,z=qz,w=qw)

        self.get_logger().info(f'→ Sending nav goal: ({mx:.2f},{my:.2f})')
        fut = self.nav_cli.send_goal_async(goal, feedback_callback=self._nav_fb)
        fut.add_done_callback(self._nav_done)

    def _nav_fb(self, feedback):
        rem = feedback.feedback.distance_remaining
        self.get_logger().debug(f'… remaining {rem:.2f}m')

    def _nav_done(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('nav rejected')
            self._finish_event()
            return
        handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('navigation succeeded')
            self._capture_and_save_image()
        else:
            self.get_logger().warn(f'navigation failed ({status})')
    
        # ───────────── 로그 기록 ─────────────
        try:
            # 1) 로그 파일/폴더 보장
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
    
            # 2) 안전하게 JSON 읽기
            try:
                raw = self.log_path.read_text() if self.log_path.exists() else ""
                logs = json.loads(raw) if raw.strip() else []      # ← 빈 파일이면 []
            except json.JSONDecodeError as e:
                # 손상된 파일은 백업 후 새로 시작
                corrupt = self.log_path.with_suffix('.corrupt')
                self.log_path.rename(corrupt)
                self.get_logger().warn(f'log corrupted → {corrupt.name}: {e}')
                logs = []
    
            # 3) 새 항목 추가
            new_entry = {
                'dest_category':  self.dest_category,
                'date':           datetime.now().strftime('%Y-%m-%d'),
                'time':           datetime.now().strftime('%H:%M:%S'),
                'noise_category': self.noise_category,
            }
    
            # 4) 임시 파일에 먼저 쓰고 교체(중간 0 바이트 방지)
            logs.append(new_entry)
            tmp = self.log_path.with_suffix('.tmp')
            tmp.write_text(json.dumps(logs, ensure_ascii=False, indent=2))
            os.replace(tmp, self.log_path)
            
            msg = String()
            msg.data = json.dumps(new_entry, ensure_ascii=False)
            self.noise_pub.publish(msg)
            self.get_logger().info(f'Published to /noise: {msg.data}')
    
        except Exception as e:
            self.get_logger().error(f'log write failed: {e}')
            
        # ───────────── “idle” 상태 발행 ─────────────
        self.audio_status_pub.publish(String(data="idle"))
    
        # ───────────── 후처리 ─────────────
        self._finish_event()

    def _finish_event(self):
    
        if self.current_wav_file and self.current_wav_file.exists():
            try:
                os.remove(self.current_wav_file)
                self.get_logger().info(f'Deleted recording: {self.current_wav_file.name}')
            except Exception as e:
                self.get_logger().warn(f'Failed to delete {self.current_wav_file}: {e}')
        self.current_wav_file = None
    
        # 처리 끝나면 busy 해제
        self.busy = False
        self.get_logger().info('back to listening')
        
        
    # ── 사진 촬영 및 저장 함수 ─────────────────── ← 추가된 부분
    def _capture_and_save_image(self):
        """카메라에서 이미지를 캡처하고 지정된 경로에 저장합니다."""
        self.get_logger().info('Capturing image upon arrival...')
        try:
            # VideoCapture 객체 생성
            cap = cv2.VideoCapture(self.camera_index)

            # 카메라가 정상적으로 열렸는지 확인
            if not cap.isOpened():
                self.get_logger().error(f"Cannot open camera index {self.camera_index}")
                return

            # 프레임 한 장 읽기
            ret, frame = cap.read()
            
            # 카메라 장치 해제
            cap.release()

            if ret:
                # 저장 경로가 없으면 생성
                self.image_save_path.mkdir(parents=True, exist_ok=True)
                
                file_path = self.image_save_path / f'enviroment.jpg'
                
                # 이미지 저장
                cv2.imwrite(str(file_path), frame)
                self.get_logger().info(f'Successfully saved image to: {file_path}')
            else:
                self.get_logger().warn("Failed to capture frame from camera.")

        except Exception as e:
            self.get_logger().error(f'An error occurred during image capture: {e}')

def main():
    rclpy.init()
    node = AudioDoaRotateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
