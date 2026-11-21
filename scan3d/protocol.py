"""스캔 PC ↔ 로봇 TCP 메시지(PLAN 4.4, docs/specs/protocol.md).

메시지는 UTF-8 JSON 한 줄(`\\n`)이고, 머리말의 `color`, `depth`, `ir` 항목 중 `bytes`가 있는 것의
길이만큼 원본 바이트가 이 순서로 뒤따른다. 로봇에서도 쓰므로 표준 라이브러리만 쓴다.
"""
import json
import math
import select
import socket
import threading
import time
import zlib

CTRL_PORT = 7010
STREAM_PORT = 7011
PAYLOAD_KEYS = ("color", "depth", "ir")
MAX_LINE = 16 * 1024 * 1024
# 보내기 한도. 스냅샷(약 6 MB)을 Wi-Fi로 보낼 때 버퍼가 잠깐 막혀도 끊기지 않게 넉넉히 둔다.
SEND_TIMEOUT = 30.0


def _payload_items(obj):
    """머리말에서 바이트가 뒤따르는 항목 (키, dict)를 순서대로."""
    for key in PAYLOAD_KEYS:
        item = obj.get(key)
        if isinstance(item, dict) and "bytes" in item:
            yield key, item


def encode(obj, payloads=()):
    """한 메시지의 전송 바이트."""
    payloads = list(payloads)
    sizes = [item["bytes"] for _, item in _payload_items(obj)]
    if sizes != [len(p) for p in payloads]:
        raise ValueError(f"payload sizes {[len(p) for p in payloads]} != header {sizes}")
    line = json.dumps(obj, ensure_ascii=False, separators=(",", ":")).encode("utf-8") + b"\n"
    return b"".join([line, *payloads])


def pack_snapshot(header, streams, compress):
    """머리말에 스트림별 `bytes`를 채우고(압축하면 `zlib`, `raw_bytes`도) (머리말, 바이트 목록)을 돌려준다."""
    hdr = json.loads(json.dumps(header))
    payloads = []
    for key in PAYLOAD_KEYS:
        if key not in streams:
            continue
        data = bytes(streams[key])
        item = hdr.setdefault(key, {})
        item.pop("zlib", None)
        item.pop("raw_bytes", None)
        if compress:
            packed = zlib.compress(data, 1)
            item["zlib"] = True
            item["raw_bytes"] = len(data)
            data = packed
        item["bytes"] = len(data)
        payloads.append(data)
    return hdr, payloads


class Channel:
    """소켓 하나 위의 메시지 주고받기. `send`는 여러 스레드에서 불러도 된다."""

    def __init__(self, sock):
        self.sock = sock
        # 소켓 한도는 보내기용으로 고정한다. 받기 한도는 _fill이 select로 따로 기다린다.
        # (받기 한도를 settimeout으로 주면 짧은 recv 한도가 큰 sendall에도 걸려 연결이 끊긴다.)
        self.sock.settimeout(SEND_TIMEOUT)
        self._buf = bytearray()
        self._send_lock = threading.Lock()
        self._recv_lock = threading.Lock()

    def send(self, obj, payloads=()):
        data = encode(obj, payloads)
        with self._send_lock:
            self.sock.sendall(data)

    def _fill(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("recv timeout")
        try:
            ready, _, _ = select.select([self.sock], [], [], None if math.isinf(remaining) else remaining)
        except (OSError, ValueError) as e:  # 닫힌 소켓(fd -1) 등
            raise ConnectionError(str(e)) from e
        if not ready:
            raise TimeoutError("recv timeout")
        try:
            chunk = self.sock.recv(1 << 20)
        except socket.timeout:
            raise TimeoutError("recv timeout") from None
        except OSError as e:
            raise ConnectionError(str(e)) from e
        if not chunk:
            raise ConnectionError("connection closed")
        self._buf += chunk

    def recv(self, timeout=None):
        """다음 메시지 (obj, payloads). 끊기면 ConnectionError, 시간이 지나면 TimeoutError.

        시간이 지나도 받은 바이트는 버리지 않으므로 다음 recv가 이어서 읽는다.
        """
        deadline = math.inf if timeout is None else time.monotonic() + timeout
        with self._recv_lock:
            while True:
                end = self._buf.find(b"\n")
                if end >= 0:
                    break
                if len(self._buf) > MAX_LINE:
                    raise ConnectionError("line too long")
                self._fill(deadline)
            obj = json.loads(self._buf[:end].decode("utf-8"))
            items = list(_payload_items(obj)) if isinstance(obj, dict) else []
            total = end + 1 + sum(int(item["bytes"]) for _, item in items)
            while len(self._buf) < total:
                self._fill(deadline)
            payloads = []
            pos = end + 1
            for _, item in items:
                n = int(item["bytes"])
                data = bytes(self._buf[pos:pos + n])
                pos += n
                if item.pop("zlib", False):
                    data = zlib.decompress(data)
                    raw = item.pop("raw_bytes", len(data))
                    if raw != len(data):
                        raise ValueError(f"zlib size {len(data)} != raw_bytes {raw}")
                    item["bytes"] = len(data)
                payloads.append(data)
            del self._buf[:total]
            return obj, payloads

    def close(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


def connect(host, port, timeout=5.0):
    """연결 시간 한도만 정한다. 요청별 한도는 recv(timeout)로 준다."""
    sock = socket.create_connection((host, port), timeout=timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return Channel(sock)


def encode_scan(t, angle_min, angle_inc, range_min, range_max, ranges_m):
    """LaserScan 값 → `scan` 메시지. 무효값(유한하지 않음, 범위 밖)은 0 mm."""
    mm = []
    for r in ranges_m:
        r = float(r)
        if math.isfinite(r) and range_min <= r <= range_max:
            mm.append(int(round(r * 1000.0)))
        else:
            mm.append(0)
    return {"type": "scan", "t": float(t), "angle_min": float(angle_min), "angle_inc": float(angle_inc),
            "range_min": float(range_min), "range_max": float(range_max), "ranges_mm": mm}


def decode_scan(msg):
    """`scan` 메시지 → `ranges`(m, 무효값 inf)."""
    return {"t": msg["t"], "angle_min": msg["angle_min"], "angle_inc": msg["angle_inc"],
            "range_min": msg["range_min"], "range_max": msg["range_max"],
            "ranges": [v / 1000.0 if v > 0 else math.inf for v in msg["ranges_mm"]]}
