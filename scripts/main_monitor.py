#!/usr/bin/env python3
"""
MainMonitor
===========

‣ bringup + nav만 살아 있으면
    • audio_doa_rotate_launch.py 자동 ON  
    • /command, /pixel_goal 허용

‣ /command 수신 시
    • 허용된 명령어인지 확인
    • Audio 노드가 켜져 있으면 즉시 OFF
    • ca.execute_command(...) 실행
    • 이후 5 초 주기 타이머가 조건을 확인해 Audio 자동 ON

‣ pixel_to_goal·mapping 등 다른 작업이 돌면 Audio OFF (+ command 거부)
"""

import json, signal, subprocess
import threading
from pathlib import Path
from threading import Lock


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import command_activate as ca           # ← 로봇 모션 실행 모듈


class MainMonitor(Node):
    # ───────────────────── 초기화 ─────────────────────
    def __init__(self):
        super().__init__('map_monitor')

        # ── 지도 경로 ──
        mdir = Path.home() / 'turtlebot3_ws/map'
        self.map_yaml = mdir / 'map.yaml'
        self.map_png  = mdir / 'map.png'
        self.map_pgm  = mdir / 'map.pgm'

        # ── 프로세스 관리 ──
        self.proc_table: dict[str, subprocess.Popen] = {}
        self.proc_lock = Lock()
        self.mapping_proc = None
        self.mapping_active = False

        # ── 로봇 컨트롤러 ──
        self.controller = ca.TurtleBotController()
        
        # ── 전역 busy 플래그 (command or pixel_to_nav 수행 중 여부) ──
        self.busy_general = False

        # 허용 명령 사전
        self.command_dict = {
            "안녕": 1, "놀아줘": 1, "앞으로 가": 1, "뒤로 가": 1,
            "너 별로야": 1, "잘자": 1, "돌아봐": 1, "춤춰줘": 1
        }

        # 상태 알림 토픽
        self.status_pub = self.create_publisher(String, '/monitor_status', 10)

        # 주기 타이머
        self.create_timer(2.0, self.check_mapping_done)
        self.create_timer(5.0, self.cleanup_proc_table)
        self.create_timer(5.0, self.ensure_audio_state)

        # 토픽 구독
        self.create_subscription(String, '/reexplore', self.reexplore_cb, 10)
        self.create_subscription(String, '/pixel_goal',    self.pixel_goal_cb, 10)
        self.create_subscription(String, '/command',       self.command_cb,    10)
        self.create_subscription(String, '/command_status', self.command_status_cb, 10)

        self.nav_init_wait = 60.0
        self.report("bringup 완료, 초기 지도 상태 확인")
        self.initial_check()
        
        # Audio 내비게이션 중인지 감지하는 플래그
        self.audio_busy = False
        
        self.create_subscription(
            String,
            '/audio_status',
            self.audio_status_cb,
            10,
        )

    # ───────────────────── 유틸 ─────────────────────
    def report(self, msg: str, lvl: str = "info"):
        getattr(self.get_logger(), lvl)(msg)
        self.status_pub.publish(String(data=msg))

    def run_unique(self, key: str, cmd: list[str]):
        with self.proc_lock:
            if key in self.proc_table and self.proc_table[key].poll() is None:
                self.report(f"{key} 이미 실행 중 – 새로 실행 안 함", "info")
                return None
            try:
                p = subprocess.Popen(cmd)
                self.proc_table[key] = p
                self.report(f"{key} 실행 시작 (PID {p.pid})")
                return p
            except Exception as e:
                self.report(f"{key} 실행 실패: {e}", "info")
                return None

    def terminate_process(self, proc: subprocess.Popen):
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=10)
            self.report(f"프로세스 종료 완료: PID={proc.pid}")
        except subprocess.TimeoutExpired:
            self.report(f"프로세스 강제 종료: PID={proc.pid}", "info")
            proc.kill()

    def cleanup_proc_table(self):
        with self.proc_lock:
            for k, p in list(self.proc_table.items()):
                if p.poll() is not None:
                    del self.proc_table[k]

    # ───────────────────── 초기 동작 ─────────────────────
    def initial_check(self):
        if self.map_yaml.exists():
            self.report("지도 있음 → Navigation 실행")
            self.launch_navigation()
        else:
            self.report("지도 없음 → Mapping 실행")
            self.launch_mapping()

    # ───────────────────── 런처 ─────────────────────
    def launch_navigation(self):
        nav = self.run_unique("nav",
            ['ros2','launch','hearbear','nav_with_auto_init.launch.py'])
        if not nav: return
        self.report(f"Navigation 초기화 대기... ({self.nav_init_wait}초)")
        self.get_clock().sleep_for(
            rclpy.duration.Duration(seconds=self.nav_init_wait))

    def launch_audio(self):
        self.run_unique("audio",
            ['ros2','launch','hearbear','audio_doa_rotate_launch.py'])

    def launch_mapping(self):
        for k in ("nav", "audio"):
            with self.proc_lock:
                if k in self.proc_table and self.proc_table[k].poll() is None:
                    self.report(f"Mapping 시작 전 {k} 중단")
                    self.terminate_process(self.proc_table[k])
        proc = self.run_unique("mapping",
            ['ros2','launch','hearbear','cartographer_mapping_launch.py'])
        if proc:
            self.mapping_proc, self.mapping_active = proc, True

    # ───────────────────── 타이머 콜백 ─────────────────────
    def check_mapping_done(self):
        if self.mapping_active and self.mapping_proc.poll() is not None:
            self.report("Mapping 종료 감지 → Navigation 재시작")
            self.mapping_active = False
            self.mapping_proc = None
            self.launch_navigation()

    def ensure_audio_state(self):
        with self.proc_lock:
            audio_alive = ('audio' in self.proc_table and
                           self.proc_table['audio'].poll() is None)
            nav_alive   = ('nav' in self.proc_table and
                           self.proc_table['nav'].poll() is None)
            busy_else = [k for k,p in self.proc_table.items()
                         if k not in ('nav','audio','bringup')
                         and p.poll() is None]

        if nav_alive and not busy_else:
            if not audio_alive:
                self.report("Audio 조건 만족 → 실행")
                self.launch_audio()
        else:
            if audio_alive:
                self.report(f"Audio 중단 – 조건 불충족 (busy={busy_else or 'nav 없음'})")
                self.terminate_process(self.proc_table['audio'])

    # ───────────────────── 토픽 콜백 ─────────────────────
    def audio_status_cb(self, msg: String) -> None:
        """
        Audio 노드가 publish하는 상태 문자열("idle" / "navigating")을 받아서
        self.audio_busy 플래그를 갱신합니다.
        """
        status = msg.data.strip()
        if status in ("detecting", "navigating"):
            self.audio_busy = True
        else:
            # "idle", "listening" 등 그 외 모든 상태를 False로 간주
            self.audio_busy = False
    
    def reexplore_cb(self, _msg: String) -> None:
    
        if self.busy_general:
            self.report("reexplore_map 거부 – 다른 작업 수행 중", "info")
            return
            
        # 0) Audio 내비게이션 중이면 거부
        if self.audio_busy:
            self.report("reexplore_map 거부 – Audio 노드 내비게이션 중", "info")
            return

        # 1) 다른 busy 작업 있는지 확인
        with self.proc_lock:
            busy_else = [
                k
                for k, p in self.proc_table.items()
                if k not in ("nav", "audio", "bringup") and p.poll() is None
            ]
        if busy_else:
            self.report(f"reexplore_map 거부 – 다른 작업 진행 중: {busy_else}", "info")
            return

        # 2) 모든 프로세스 종료 + 지도 삭제 + Mapping 재시작
        self.report("재탐색 요청 → 모든 프로세스 종료 + 지도 삭제 + Mapping 재시작")
        for k in list(self.proc_table):
            self.terminate_process(self.proc_table[k])
        for f in (self.map_yaml, self.map_png, self.map_pgm):
            f.unlink(missing_ok=True)
        self.launch_mapping()

    def pixel_goal_cb(self, msg: String) -> None:
    
        if self.busy_general:
            self.report("pixel_goal 거부 – 다른 작업 수행 중", "info")
            return
            
        # 0) Audio 내비게이션 중이면 거부
        if self.audio_busy:
            self.report("pixel_goal 거부 – Audio 노드 내비게이션 중", "info")
            return
            
        # 2) Audio가 켜져 있으면 즉시 OFF (듣기 상태였더라도 kill)
        with self.proc_lock:
            if "audio" in self.proc_table and self.proc_table["audio"].poll() is None:
                self.report("pixel_goal 수신 → Audio 중단")
                self.terminate_process(self.proc_table["audio"])

        try:
            px, py = (
                (lambda d: (int(d["px"]), int(d["py"])))(json.loads(msg.data))
                if "{" in msg.data
                else map(int, msg.data.replace(",", " ").split()[:2])
            )
        except Exception:
            self.report("pixel_goal: 잘못된 형식", "info")
            return
            

        # 1) nav alive 체크 + 다른 busy 작업 있는지 확인
        with self.proc_lock:
            nav_alive = (
                "nav" in self.proc_table and self.proc_table["nav"].poll() is None
            )
            busy_else = [
                k
                for k, p in self.proc_table.items()
                if k not in ("nav", "audio", "bringup") and p.poll() is None
            ]
        if not nav_alive:
            self.report("pixel_goal 거부 – nav 없음", "info")
            return
        if busy_else:
            self.report(f"pixel_goal 거부 – 다른 작업 진행 중: {busy_else}", "info")
            return


        self.busy_general = True
        
        # 3) pixel_to_nav_goal 실행
        proc = self.run_unique(
            "pixel_to_goal",
            [
                "ros2",
                "run",
                "hearbear",
                "pixel_to_nav_goal.py",
                "--x",
                str(px),
                "--y",
                str(py),
            ],
        )
        self.report(f"픽셀 목표 ({px},{py}) 실행 요청 완료")
        
        if proc is not None:
            def _on_proc_exit():
                proc.wait()            # 프로세스가 완전히 종료될 때까지 대기
                self.busy_general = False

            import threading
            threading.Thread(target=_on_proc_exit, daemon=True).start()
        else:
            # 만약 run_unique가 None을 리턴했다면, 바로 busy_general 해제
            self.busy_general = False
            
    def command_status_cb(self, msg: String):
        if msg.data.strip() == 'done' and self.busy_general:
            self.busy_general = False
            self.report("Command 완료 신호 수신 → busy_general=False", "info")
            

    def command_cb(self, msg: String) -> None:
        cmd = msg.data.strip()
        
        if self.busy_general:
            self.report("command 거부 – 다른 작업 수행 중", "info")
            return

        # 0) Audio 내비게이션 수행 중이면 거부
        if self.audio_busy:
            self.report("command 거부 – Audio 노드 내비게이션 중", "info")
            return
            
          # 3) Audio가 켜져 있으면 즉시 OFF (듣기 상태였더라도 kill)
        with self.proc_lock:
            if "audio" in self.proc_table and self.proc_table["audio"].poll() is None:
                self.report("command 수신 → Audio 중단")
                self.terminate_process(self.proc_table["audio"])

        # 1) 허용 여부
        if cmd not in self.command_dict:
            self.report(f'Unknown command: "{cmd}"', "info")
            return

        # 2) nav alive 체크 + 다른 busy 작업 있는지 확인
        with self.proc_lock:
            nav_alive = (
                "nav" in self.proc_table and self.proc_table["nav"].poll() is None
            )
            busy_else = [
                k
                for k, p in self.proc_table.items()
                if k not in ("nav", "audio", "bringup") and p.poll() is None
            ]
        if not nav_alive:
            self.report("command 거부 – nav 없음", "warning")
            return
        if busy_else:
            self.report(f"command 거부 – 다른 작업 진행 중: {busy_else}", "info")
            return
            
        

        self.busy_general = True
        self.report(f'Command 수행 시작: "{cmd}"', "info")
        
        # 4) 실제 명령 실행
        
        threading.Thread(
            target=lambda: ca.execute_command(cmd, self.controller),
            daemon=True
        ).start()
        # ⑤ 이후 ensure_audio_state() 주기 타이머가 Audio 자동 재ON
        

# ───────────────────── main ─────────────────────
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(4)
    node = MainMonitor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
