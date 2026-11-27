"""알람: 코드표, 발생·해제 이력, 확인(docs/specs/webui.md)."""
import itertools
import socket
import threading
import time

CODES = {
    "E01": "표식 미검출",
    "E02": "주행 경로 막힘",
    "E03": "로봇 통신 끊김",
    "E04": "두 로봇 각도 차 벗어남",
    "E05": "재구성 실패",
    "W06": "장애물 감지",
    "E09": "기타 오류",
}
MAX_KEEP = 200


def classify(e):
    """작업 실패 예외 → 알람 코드."""
    from ..robot import drive
    from ..robot.link import RemoteError
    from ..robot.unit import StartError

    msg = str(e)
    if isinstance(e, drive.Blocked):
        return "E02"
    if isinstance(e, (ConnectionError, TimeoutError, RemoteError, socket.timeout)):
        return "E03"
    if isinstance(e, StartError):
        if "표식" in msg:
            return "E01"
        if "각도 차" in msg:
            return "E04"
        return "E09"
    if "융합" in msg or "재구성" in msg:
        return "E05"
    return "E09"


class Alarms:
    """알람 목록. key가 같은 활성 알람은 하나만(통신 끊김·장애물처럼 상태로 켜고 끄는 것)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self._items = []

    def raise_(self, code, detail="", role=None, key=None):
        with self._lock:
            if key is not None:
                for a in self._items:
                    if a["key"] == key and a["active"]:
                        if a["detail"] != detail:      # 내용이 바뀌면 다시 확인받는다
                            a["detail"], a["acked"] = detail, False
                        return a["id"]
            a = {"id": next(self._ids), "code": code, "level": "warn" if code.startswith("W") else "alarm",
                 "text": CODES.get(code, code), "detail": detail, "role": role, "key": key,
                 "t": time.strftime("%H:%M:%S"), "cleared_t": None, "active": True, "acked": False}
            self._items.append(a)
            del self._items[:-MAX_KEEP]
            return a["id"]

    def clear(self, key):
        """상태로 켠 알람(key)을 해제한다(이력에는 남는다)."""
        with self._lock:
            for a in self._items:
                if a["key"] == key and a["active"]:
                    a["active"], a["cleared_t"] = False, time.strftime("%H:%M:%S")

    def ack(self, alarm_id=None):
        """확인. id가 없으면 전부. key 없는 알람(작업 실패)은 확인하면 해제."""
        with self._lock:
            for a in self._items:
                if alarm_id is None or a["id"] == alarm_id:
                    a["acked"] = True
                    if a["key"] is None and a["active"]:
                        a["active"], a["cleared_t"] = False, time.strftime("%H:%M:%S")

    def active(self, key):
        with self._lock:
            return any(a["key"] == key and a["active"] for a in self._items)

    def snapshot(self):
        with self._lock:
            items = [dict(a) for a in reversed(self._items)]
        unacked = [a for a in items if not a["acked"] and (a["active"] or a["level"] == "alarm")]
        return {"items": items[:50], "unacked": len(unacked),
                "active_alarm": any(a["active"] and a["level"] == "alarm" for a in items),
                "active_warn": any(a["active"] and a["level"] == "warn" for a in items), "codes": CODES}
