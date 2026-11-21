"""[공통] 설정(config/scan.yaml), 표식 판, 로봇 주소."""
import json
import subprocess
from pathlib import Path

import yaml

from .robot import tagpose

ROOT = Path(__file__).resolve().parents[1]
ORDER = ("L", "H")  # 로봇 역할. L 카메라 낮게, H 높게


def load_config(path=None):
    path = Path(path or ROOT / "config" / "scan.yaml")
    return path, yaml.safe_load(path.read_text(encoding="utf-8"))


def load_board(cfg):
    board_path = ROOT / cfg["board"]["layout"]
    board_json = json.loads(board_path.read_text(encoding="utf-8"))
    return board_path, tagpose.load_board(board_json, cfg["board"]["tag_mm"])


def load_env(path=None):
    """저장소 .env(KEY=VALUE 줄)를 dict로. 없으면 빈 dict."""
    path = Path(path or ROOT / ".env")
    env = {}
    if path.is_file():
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                key, _, value = line.partition("=")
                env[key.strip()] = value.strip().strip('"').strip("'")
    return env


ENV_HOST = {"pinky1": "PINKY1_HOST", "pinky2": "PINKY2_HOST"}


def resolve_host(host, env=None):
    """로봇 주소를 푼다. `~/.ssh/config` 별칭이면 `ssh -G`의 hostname, 별칭이 없고 .env에
    PINKY1_HOST·PINKY2_HOST가 있으면 그 값, 둘 다 아니면 그대로."""
    via_ssh = _ssh_hostname(host)
    if via_ssh != host:
        return via_ssh
    key = ENV_HOST.get(host)
    if key:
        value = (load_env() if env is None else env).get(key)
        if value:
            return value
    return host


def _ssh_hostname(host):
    try:
        out = subprocess.run(["ssh", "-G", host], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return host
    for line in out.splitlines():
        key, _, value = line.partition(" ")
        if key == "hostname" and value:
            return value.strip()
    return host


def robot_addr(cfg, role):
    """config의 역할 → (host, 제어 포트, 스트림 포트)."""
    rc = cfg["robots"][role]
    return resolve_host(rc["host"]), rc["port"], rc["stream_port"]
