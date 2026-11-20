"""RealSense D455 파이프라인 래퍼(PLAN 4.2, docs/specs/robot_agent.md)."""
import time


def _frame_bytes(frame) -> bytes:
    return memoryview(frame.get_data()).tobytes()


def _intrinsics_dict(intr) -> dict:
    return {
        "w": int(intr.width),
        "h": int(intr.height),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "model": intr.model.name,
        "coeffs": [float(c) for c in intr.coeffs],
    }


class RealSenseCamera:
    """pyrealsense2 기반 스냅샷·보정 카메라. 테스트에서는 `rs_module`에 가짜 객체를 주입한다."""

    def __init__(self, rs_module=None, clock=time.monotonic):
        self._rs_mod = rs_module
        self._clock = clock
        self._serial: str | None = None

    def _rs(self):
        if self._rs_mod is None:
            import pyrealsense2 as rs
            self._rs_mod = rs
        return self._rs_mod

    def serial(self) -> str:
        if self._serial is not None:
            return self._serial
        rs = self._rs()
        devices = rs.context().query_devices()
        if not devices:
            raise RuntimeError("RealSense camera not found")
        self._serial = str(devices[0].get_info(rs.camera_info.serial_number))
        return self._serial

    def calibration(self, color_profile, depth_profile) -> dict:
        rs = self._rs()
        cw, ch, cfps = [int(v) for v in color_profile]
        dw, dh, dfps = [int(v) for v in depth_profile]

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, cw, ch, rs.format.yuyv, cfps)
        cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)

        profile = pipe.start(cfg)
        try:
            dev = profile.get_device()
            self._serial = str(dev.get_info(rs.camera_info.serial_number))
            depth_scale = float(dev.first_depth_sensor().get_depth_scale())

            c_sp = profile.get_stream(rs.stream.color).as_video_stream_profile()
            d_sp = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            c_intr = c_sp.get_intrinsics()
            d_intr = d_sp.get_intrinsics()
            ext = d_sp.get_extrinsics_to(c_sp)
            return {
                "color": _intrinsics_dict(c_intr),
                "depth": _intrinsics_dict(d_intr),
                "depth_to_color": {
                    "rotation": [float(v) for v in ext.rotation],
                    "translation": [float(v) for v in ext.translation],
                },
                "depth_scale": depth_scale,
            }
        finally:
            pipe.stop()

    def snapshot(self, color_profile, depth_profile, n_depth: int, n_skip: int):
        rs = self._rs()
        cw, ch, cfps = [int(v) for v in color_profile]
        dw, dh, dfps = [int(v) for v in depth_profile]
        if n_depth < 1:
            raise ValueError(f"n_depth must be >= 1, got {n_depth}")

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, cw, ch, rs.format.yuyv, cfps)
        cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)

        pipe.start(cfg)
        try:
            for _ in range(int(n_skip)):
                pipe.wait_for_frames()

            color_bytes = None
            depth_chunks = []
            t = 0.0
            retries = 0
            while len(depth_chunks) < n_depth:
                frames = pipe.wait_for_frames()
                now = float(self._clock())
                df = frames.get_depth_frame()
                if color_bytes is None:
                    cf = frames.get_color_frame()
                    if not cf or not df:
                        retries += 1
                        if retries > 10:
                            raise RuntimeError("incomplete color/depth frameset")
                        continue
                    t = now
                    color_bytes = _frame_bytes(cf)
                else:
                    if not df:
                        retries += 1
                        if retries > 10:
                            raise RuntimeError("missing depth frame")
                        continue
                depth_chunks.append(_frame_bytes(df))

            return (
                t,
                {"w": cw, "h": ch, "fmt": "yuyv"},
                color_bytes,
                {"w": dw, "h": dh, "fmt": "z16", "n": int(n_depth)},
                b"".join(depth_chunks),
            )
        finally:
            pipe.stop()

    def snapshot_ir(self, depth_profile, n_skip: int):
        rs = self._rs()
        dw, dh, dfps = [int(v) for v in depth_profile]

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.infrared, 1, dw, dh, rs.format.y8, dfps)

        profile = pipe.start(cfg)
        try:
            sensor = profile.get_device().first_depth_sensor()
            prev_emitter = None
            if sensor.supports(rs.option.emitter_enabled):
                prev_emitter = sensor.get_option(rs.option.emitter_enabled)
                sensor.set_option(rs.option.emitter_enabled, 0.0)
            try:
                for _ in range(int(n_skip)):
                    pipe.wait_for_frames()
                frames = pipe.wait_for_frames()
                t = float(self._clock())
                irf = frames.get_infrared_frame(1)
                if not irf:
                    irf = frames.get_infrared_frame()
                if not irf:
                    raise RuntimeError("missing infrared frame")
                return (t, {"w": dw, "h": dh, "fmt": "y8"}, _frame_bytes(irf))
            finally:
                if prev_emitter is not None:
                    sensor.set_option(rs.option.emitter_enabled, prev_emitter)
        finally:
            pipe.stop()
