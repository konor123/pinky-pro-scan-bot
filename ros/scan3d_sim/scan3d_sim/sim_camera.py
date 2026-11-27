"""ROS 없이 동작하는 Gazebo RGB-D 카메라 래퍼와 메시지 변환(PLAN 4.2)."""
import array
import threading
import time

_IDENTITY_ROT = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def calib_from_camera_info(width: int, height: int, k) -> dict:
    """`CameraInfo`의 해상도와 3×3 행렬 `k`로 protocol `calib` dict를 만든다."""
    w, h = int(width), int(height)
    if w <= 0 or h <= 0:
        raise ValueError(f"invalid image size: {w}x{h}")
    k_vals = [float(v) for v in k]
    if len(k_vals) != 9:
        raise ValueError(f"camera_info k must have 9 elements, got {len(k_vals)}")
    fx, ppx, fy, ppy = k_vals[0], k_vals[2], k_vals[4], k_vals[5]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError(f"invalid focal lengths: fx={fx}, fy={fy}")
    intr = {
        "w": w,
        "h": h,
        "fx": fx,
        "fy": fy,
        "ppx": ppx,
        "ppy": ppy,
        "model": "none",
        "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    }
    return {
        "color": dict(intr),
        "depth": dict(intr),
        "depth_to_color": {
            "rotation": list(_IDENTITY_ROT),
            "translation": [0.0, 0.0, 0.0],
        },
        "depth_scale": 0.001,
    }


def calib_from_info_msg(msg) -> dict:
    """ROS `sensor_msgs/CameraInfo` 메시지에서 `calib` dict를 만든다."""
    k = getattr(msg, "k", None)
    if k is None:
        k = getattr(msg, "K")
    return calib_from_camera_info(msg.width, msg.height, k)


def _strip_row_padding(data: bytes, height: int, row_bytes: int, step: int | None) -> bytes:
    buf = bytes(data)
    stride = row_bytes if step is None or int(step) == 0 else int(step)
    if stride < row_bytes:
        raise ValueError(f"step {stride} < row bytes {row_bytes}")
    need = stride * height
    if len(buf) < need:
        raise ValueError(f"image buffer too short: {len(buf)} < {need}")
    if stride == row_bytes and len(buf) == need:
        return buf
    return b"".join(buf[r * stride : r * stride + row_bytes] for r in range(height))


def pack_color_frame(
    width: int,
    height: int,
    encoding: str,
    data: bytes,
    step: int | None = None,
) -> tuple[dict, bytes]:
    """`rgb8` 컬러 영상을 protocol 머리말과 조밀한 바이트로 묶는다."""
    if encoding != "rgb8":
        raise ValueError(f"unsupported color encoding: {encoding!r}")
    w, h = int(width), int(height)
    if w <= 0 or h <= 0:
        raise ValueError(f"invalid color size: {w}x{h}")
    raw = _strip_row_padding(data, h, w * 3, step)
    return {"w": w, "h": h, "fmt": "rgb8"}, raw


def pack_depth_frame(
    width: int,
    height: int,
    encoding: str,
    data: bytes,
    step: int | None = None,
    is_bigendian: bool = False,
) -> tuple[int, int, bytes]:
    """`32FC1` 깊이 영상 한 장을 리틀 엔디언 바이트로 정규화한다."""
    if encoding != "32FC1":
        raise ValueError(f"unsupported depth encoding: {encoding!r}")
    w, h = int(width), int(height)
    if w <= 0 or h <= 0:
        raise ValueError(f"invalid depth size: {w}x{h}")
    raw = _strip_row_padding(data, h, w * 4, step)
    if is_bigendian:
        # big-endian 4바이트 float은 호스트 엔디언과 무관하게 바이트 순서를 뒤집으면 리틀 엔디언이 된다.
        arr = array.array("f")
        arr.frombytes(raw)
        arr.byteswap()
        raw = arr.tobytes()
    return w, h, raw


def pack_depth_frames(frames: list[tuple[int, int, bytes]]) -> tuple[dict, bytes]:
    """정규화된 깊이 프레임 목록을 protocol `f32` 머리말과 바이트로 묶는다."""
    if not frames:
        raise ValueError("frames must be non-empty")
    w, h, _ = frames[0]
    chunks = []
    for fw, fh, raw in frames:
        if fw != w or fh != h:
            raise ValueError(f"depth frame size mismatch: {fw}x{fh} != {w}x{h}")
        if len(raw) != w * h * 4:
            raise ValueError(f"depth frame bytes {len(raw)} != {w * h * 4}")
        chunks.append(raw)
    return {"w": w, "h": h, "fmt": "f32", "n": len(frames)}, b"".join(chunks)


class GazeboCamera:
    """`scan3d_robot.agent.Agent`가 쓰는 Gazebo 카메라 어댑터."""

    def __init__(self, serial: str = "SIM-D455", clock=time.monotonic, timeout_s: float = 20.0):
        self._serial = str(serial)
        self._clock = clock
        self._timeout_s = float(timeout_s)
        self._cond = threading.Condition()
        self._calib: dict | None = None
        self._color: tuple[dict, bytes] | None = None
        self._color_seq = 0
        self._color_t = 0.0
        self._depth: tuple[int, int, bytes] | None = None
        self._depth_seq = 0

    def serial(self) -> str:
        return self._serial

    def on_camera_info(self, width: int, height: int, k) -> None:
        calib = calib_from_camera_info(width, height, k)
        with self._cond:
            self._calib = calib
            self._cond.notify_all()

    def on_camera_info_msg(self, msg) -> None:
        calib = calib_from_info_msg(msg)
        with self._cond:
            self._calib = calib
            self._cond.notify_all()

    def on_color(self, width: int, height: int, encoding: str, data: bytes, step: int | None = None) -> None:
        packed = pack_color_frame(width, height, encoding, data, step=step)
        now = float(self._clock())
        with self._cond:
            self._color = packed
            self._color_t = now
            self._color_seq += 1
            self._cond.notify_all()

    def on_color_msg(self, msg) -> None:
        self.on_color(msg.width, msg.height, msg.encoding, bytes(msg.data), step=msg.step)

    def on_depth(
        self,
        width: int,
        height: int,
        encoding: str,
        data: bytes,
        step: int | None = None,
        is_bigendian: bool = False,
    ) -> None:
        packed = pack_depth_frame(width, height, encoding, data, step=step, is_bigendian=is_bigendian)
        with self._cond:
            self._depth = packed
            self._depth_seq += 1
            self._cond.notify_all()

    def on_depth_msg(self, msg) -> None:
        self.on_depth(
            msg.width,
            msg.height,
            msg.encoding,
            bytes(msg.data),
            step=msg.step,
            is_bigendian=bool(msg.is_bigendian),
        )

    def calibration(self, color_profile, depth_profile) -> dict:
        del color_profile, depth_profile
        deadline = time.monotonic() + self._timeout_s
        with self._cond:
            while self._calib is None:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    raise TimeoutError("timed out waiting for Gazebo camera_info")
                self._cond.wait(rem)
            c = self._calib
            return {
                "color": dict(c["color"]),
                "depth": dict(c["depth"]),
                "depth_to_color": {
                    "rotation": list(c["depth_to_color"]["rotation"]),
                    "translation": list(c["depth_to_color"]["translation"]),
                },
                "depth_scale": float(c["depth_scale"]),
            }

    def snapshot(self, color_profile, depth_profile, n_depth: int, n_skip: int = 0):
        del color_profile, depth_profile, n_skip
        n = int(n_depth)
        if n < 1:
            raise ValueError(f"n_depth must be >= 1, got {n}")
        deadline = time.monotonic() + self._timeout_s
        with self._cond:
            c0 = self._color_seq
            d0 = self._depth_seq
            got_color = None
            t = 0.0
            got_depths: list[tuple[int, int, bytes]] = []
            while got_color is None or len(got_depths) < n:
                if got_color is None and self._color_seq > c0 and self._color is not None:
                    got_color = self._color
                    t = self._color_t
                while len(got_depths) < n and self._depth_seq > d0 and self._depth is not None:
                    d0 = self._depth_seq
                    got_depths.append(self._depth)
                if got_color is not None and len(got_depths) >= n:
                    break
                rem = deadline - time.monotonic()
                if rem <= 0:
                    raise TimeoutError("timed out waiting for Gazebo camera frames")
                self._cond.wait(rem)
            c_hdr, c_bytes = got_color
            d_hdr, d_bytes = pack_depth_frames(got_depths)
            return t, dict(c_hdr), c_bytes, d_hdr, d_bytes

    def snapshot_ir(self, depth_profile, n_skip: int = 0):
        raise NotImplementedError("IR snapshot is not supported in simulation")
