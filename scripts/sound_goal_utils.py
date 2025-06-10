#!/usr/bin/env python3
"""
sound_goal_utils.py

TF 버퍼에서 map→base_link 변환을 조회해
yaw ± tol 범위 내 픽셀 후보를 선택합니다.
"""

import math
import json
import yaml
import os
from pathlib import Path
from PIL import Image
from tf2_ros import Buffer
import tf_transformations as tft
import rclpy
from rclpy.logging import get_logger

# 설정
MAP_YAML      = Path("~/turtlebot3_ws/map/map.yaml").expanduser()
SOURCES_JSON  = Path("~/turtlebot3_ws/dest_list/destination.json").expanduser()
ANGLE_TOL_DEG = 12.0
TF_TIMEOUT    = 0.3

# 지도 메타 로드
_data      = yaml.safe_load(open(MAP_YAML, "r"))
_RES       = _data["resolution"]
_ORX, _ORY = _data["origin"][:2]
_IMG_H     = Image.open(os.path.join(MAP_YAML.parent, _data["image"])).height

# 후보 목록 로드
with open(SOURCES_JSON, "r") as f:
    _TARGETS = json.load(f)  # [{"id":.., "category":.., "px":.., "py":..}, ...]

def _px_to_map(px: int, py: int):
    """픽셀(px,py) → map 좌표(m) 변환"""
    mx = _ORX + px * _RES
    my = _ORY + (_IMG_H - py) * _RES
    return mx, my

def _load_targets():
    """JSON에서 타겟 목록 로드"""
    try:
        with open(SOURCES_JSON, "r") as f:
            targets = json.load(f)
        return targets
    except Exception as e:
        get_logger("sound_goal_utils").error(f"Failed to load targets JSON: {e}")
        return []


def select_px_py(tf_buf: Buffer,
                 angle_tol_deg: float = ANGLE_TOL_DEG,
                 tf_timeout: float = TF_TIMEOUT):
    """
    :param tf_buf: tf2_ros.Buffer
    :param angle_tol_deg: 허용 오차(°)
    :param tf_timeout: TF 조회 타임아웃(s)
    :return: (category, px, py) 또는 None
    """
    logger = get_logger("sound_goal_utils")
    
    #후보 목록을 매 호출마다 로드
    _TARGETS = _load_targets()
    logger.info(f"[select_px_py] called (tol={angle_tol_deg}°), loaded {len(_TARGETS)} targets")
    
    if not _TARGETS:
        logger.error("[select_px_py] No targets loaded. Aborting selection.")
        return None

    # TF 조회 (예외 시 None 반환)
    try:
        tr = tf_buf.lookup_transform(
            "map", "base_link",
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=tf_timeout)
        )
    except Exception as e:
        logger.error(f"[select_px_py] TF lookup failed: {e}")
        # TF가 준비되지 않았으면 None
        return None

    # 로봇 위치·yaw 계산
    rx, ry = tr.transform.translation.x, tr.transform.translation.y
    qx, qy, qz, qw = (
        tr.transform.rotation.x,
        tr.transform.rotation.y,
        tr.transform.rotation.z,
        tr.transform.rotation.w,
    )
    _, _, yaw = tft.euler_from_quaternion([qx, qy, qz, qw])
    yaw_deg = math.degrees(yaw)
    tol_rad = math.radians(angle_tol_deg)
    logger.info(f"[select_px_py] robot yaw = {yaw_deg:.1f}°")

    # 후보 탐색
    best = None
    for idx, target in enumerate(_TARGETS):
        px, py = target['px'], target['py']
        cat     = target.get('category')
        mx, my  = _px_to_map(px, py)

        bearing    = math.atan2(my - ry, mx - rx)
        diff       = abs(((bearing - yaw + math.pi) % (2*math.pi)) - math.pi)
        diff_deg   = math.degrees(diff)
        logger.info(
            f"[select_px_py] [{idx}] px,py=({px},{py}) → mx,my=({mx:.1f},{my:.1f}), "
            f"bearing={math.degrees(bearing):.1f}°, Δ={diff_deg:.1f}°"
        )

        if diff <= tol_rad:
            dist = math.hypot(mx - rx, my - ry)
            key  = (diff, dist)
            if best is None or key < best['key']:
                best = {'category': cat, 'px': px, 'py': py, 'key': key}

    # 4) 결과
    if best:
        logger.info(f"[select_px_py] selected → "
                    f"cat={best['category']}, px,py=({best['px']},{best['py']})")
        return best["category"], best["px"], best["py"]

    fallback = min(
        _TARGETS,
        key=lambda t: abs(((math.atan2(
            _px_to_map(t["px"], t["py"])[1] - ry,
            _px_to_map(t["px"], t["py"])[0] - rx
        ) - yaw + math.pi) % (2*math.pi)) - math.pi)
    )
    return fallback["category"], fallback["px"], fallback["py"]
