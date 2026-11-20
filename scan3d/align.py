"""스냅샷 원본 → 컬러 카메라 기준의 왜곡 없는 컬러·깊이와 K(docs/specs/align.md).

정렬은 librealsense v2.59.2 `wrappers/python/examples/align-with-software-device.py`대로
software device + `rs.align` 하나로 한다.
"""
from collections import namedtuple

import cv2
import numpy as np
import pyrealsense2 as rs

Frame = namedtuple("Frame", "color depth K")

MAX_TRIES = 10
FPS = 15
MODELS = {
    "none": rs.distortion.none,
    "brown_conrady": rs.distortion.brown_conrady,
    "inverse_brown_conrady": rs.distortion.inverse_brown_conrady,
    "modified_brown_conrady": rs.distortion.modified_brown_conrady,
}


def decode_color(hdr, data):
    """컬러 바이트 → BGR uint8. `hdr`는 머리말의 `color` 항목."""
    w, h, fmt = int(hdr["w"]), int(hdr["h"]), hdr["fmt"]
    buf = np.frombuffer(data, np.uint8)
    if fmt == "yuyv":
        return cv2.cvtColor(buf.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUYV)
    if fmt == "rgb8":
        return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    raise ValueError(f"color fmt {fmt}")


def decode_depth(hdr, data, depth_scale):
    """깊이 바이트 → uint16 mm 영상 n장. `hdr`는 머리말의 `depth` 항목."""
    w, h, fmt, n = int(hdr["w"]), int(hdr["h"]), hdr["fmt"], int(hdr.get("n", 1))
    if fmt == "z16":
        m = np.frombuffer(data, "<u2").reshape(n, h, w).astype(np.float64) * (depth_scale * 1000.0)
    elif fmt == "f32":
        m = np.frombuffer(data, "<f4").reshape(n, h, w).astype(np.float64) * 1000.0
    else:
        raise ValueError(f"depth fmt {fmt}")
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(m) & (m > 0)
    mm = np.where(ok, np.clip(np.round(np.where(ok, m, 0.0)), 0, 65535), 0).astype(np.uint16)
    return list(mm)


def median_depth(frames):
    """픽셀마다 0이 아닌 값의 중앙값(짝수 개면 가운데 두 값 평균을 반올림). 0이 아닌 값이 ceil(n/2)개보다 적으면 0."""
    st = np.stack([np.asarray(f, np.uint16) for f in frames]).astype(np.int64)
    n = st.shape[0]
    cnt = (st > 0).sum(0)
    s = np.sort(np.where(st > 0, st, 1 << 20), axis=0)
    lo = np.clip((cnt - 1) // 2, 0, n - 1)
    hi = np.clip(cnt // 2, 0, n - 1)
    a = np.take_along_axis(s, lo[None], 0)[0]
    b = np.take_along_axis(s, hi[None], 0)[0]
    med = (a + b + 1) // 2
    return np.where(cnt >= -(-n // 2), med, 0).astype(np.uint16)


def _model(c):
    if c["model"] not in MODELS:
        raise ValueError(f"distortion model {c['model']}")
    return MODELS[c["model"]]


def _rs_intr(c):
    i = rs.intrinsics()
    i.width, i.height = int(c["w"]), int(c["h"])
    i.fx, i.fy, i.ppx, i.ppy = float(c["fx"]), float(c["fy"]), float(c["ppx"]), float(c["ppy"])
    i.model = _model(c)
    i.coeffs = [float(v) for v in c["coeffs"]]
    return i


def K_of(c):
    """보정값 항목의 핀홀 내부 행렬."""
    return np.array([[c["fx"], 0.0, c["ppx"]], [0.0, c["fy"], c["ppy"]], [0.0, 0.0, 1.0]])


def _distorted(c):
    return _model(c) != rs.distortion.none and any(float(v) != 0.0 for v in c["coeffs"])


class _SoftwarePair:
    """깊이·컬러 software sensor 한 쌍과 syncer(호출마다 새로 만든다)."""

    def __init__(self, calib):
        self.dc, self.cc = calib["depth"], calib["color"]
        self.dev = rs.software_device()
        self.ds = self.dev.add_sensor("Depth")
        self.cs = self.dev.add_sensor("Color")
        st = rs.video_stream()
        st.type, st.index, st.uid = rs.stream.depth, 0, 1
        st.width, st.height, st.fps, st.bpp, st.fmt = int(self.dc["w"]), int(self.dc["h"]), FPS, 2, rs.format.z16
        st.intrinsics = _rs_intr(self.dc)
        self.dp = self.ds.add_video_stream(st)
        st = rs.video_stream()
        st.type, st.index, st.uid = rs.stream.color, 0, 2
        st.width, st.height, st.fps, st.bpp, st.fmt = int(self.cc["w"]), int(self.cc["h"]), FPS, 2, rs.format.yuyv
        st.intrinsics = _rs_intr(self.cc)
        self.cp = self.cs.add_video_stream(st)
        self.ds.add_read_only_option(rs.option.depth_units, 0.001)
        ext = rs.extrinsics()
        ext.rotation = [float(v) for v in calib["depth_to_color"]["rotation"]]
        ext.translation = [float(v) for v in calib["depth_to_color"]["translation"]]
        self.dp.register_extrinsics_to(self.cp, ext)
        self.ds.open(self.dp)
        self.cs.open(self.cp)
        self.syncer = rs.syncer()
        self.ds.start(self.syncer)
        self.cs.start(self.syncer)
        self.color_px = np.zeros((int(self.cc["h"]), int(self.cc["w"]) * 2), np.uint8)
        self.depth_px = None

    def push(self, depth_mm, i):
        """같은 픽셀 쌍을 시도 번호 i의 timestamp·frame_number로 넣는다."""
        self.depth_px = np.ascontiguousarray(depth_mm, np.uint16)
        for sensor, prof, px, bpp in ((self.ds, self.dp, self.depth_px, 2), (self.cs, self.cp, self.color_px, 2)):
            f = rs.software_video_frame()
            f.stride = px.strides[0]
            f.bpp = bpp
            f.timestamp = i * 1000.0 / FPS
            f.domain = rs.timestamp_domain.hardware_clock
            f.frame_number = i
            f.profile = prof.as_video_stream_profile()
            f.pixels = px
            if sensor is self.ds:
                f.depth_units = 0.001
            sensor.on_video_frame(f)

    def wait(self):
        ok, fs = self.syncer.try_wait_for_frames(500)
        return fs if ok else None

    def close(self):
        for s in (self.ds, self.cs):
            try:
                s.stop()
                s.close()
            except RuntimeError:
                pass


def _complete(fs):
    return fs is not None and bool(fs.get_depth_frame()) and bool(fs.get_color_frame())


def _sync(pair, depth_mm, tries=MAX_TRIES):
    """깊이·컬러가 모두 든 세트가 나올 때까지 쌍을 다시 넣는다. (세트, 시도 횟수)"""
    for i in range(tries):
        pair.push(depth_mm, i)
        fs = pair.wait()
        if _complete(fs):
            return fs, i + 1
    raise RuntimeError(f"no depth+color frameset after {tries} tries")


def _rs_align(depth_mm, calib):
    """`rs.align` 결과(깊이 카메라 z, mm, 왜곡 있는 컬러 격자)와 시도 횟수."""
    pair = _SoftwarePair(calib)
    try:
        fs, tries = _sync(pair, depth_mm)
        out = rs.align(rs.stream.color).process(fs)
        d = np.asanyarray(out.get_depth_frame().get_data()).copy()
    finally:
        pair.close()
    return d, tries


def color_rays(cc):
    """컬러 픽셀 격자(왜곡 있음)마다 왜곡을 편 광선 (x, y, 1)의 x, y."""
    w, h = int(cc["w"]), int(cc["h"])
    if _distorted(cc):
        u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
        pts = np.stack([u.ravel(), v.ravel()], -1).reshape(-1, 1, 2)
        n = cv2.undistortPoints(pts, K_of(cc), np.asarray(cc["coeffs"], np.float64)).reshape(h, w, 2)
        return n[..., 0], n[..., 1]
    x = (np.arange(w, dtype=np.float64) - cc["ppx"]) / cc["fx"]
    y = (np.arange(h, dtype=np.float64) - cc["ppy"]) / cc["fy"]
    return np.broadcast_to(x[None, :], (h, w)), np.broadcast_to(y[:, None], (h, w))


def _to_color_z(z_d_mm, calib):
    """깊이 카메라 z → 컬러 카메라 z: z_c = (z_d + r3·t) / (r3·ray)."""
    rot = np.asarray(calib["depth_to_color"]["rotation"], np.float64)
    t = np.asarray(calib["depth_to_color"]["translation"], np.float64)
    r3 = rot[6:9]  # 열 우선이라 R의 셋째 열
    x, y = color_rays(calib["color"])
    denom = r3[0] * x + r3[1] * y + r3[2]
    zc = (z_d_mm / 1000.0 + float(r3 @ t)) / denom
    mm = np.where(z_d_mm > 0, np.clip(np.round(zc * 1000.0), 0, 65535), 0)
    return mm.astype(np.uint16)


def align_to_color(depth_mm, calib):
    """깊이(uint16 mm, 깊이 해상도) → 컬러 해상도·왜곡 있는 컬러 격자의 컬러 카메라 z(uint16 mm)."""
    z_d, _ = _rs_align(depth_mm, calib)
    return _to_color_z(z_d, calib)


def undistort(color, depth, calib):
    """컬러 왜곡 계수로 컬러(INTER_LINEAR)와 정렬된 깊이(INTER_NEAREST)를 같은 지도로 편다. (color, depth, K)"""
    cc = calib["color"]
    K = K_of(cc)
    if not _distorted(cc):
        return color.copy(), depth.copy(), K
    w, h = int(cc["w"]), int(cc["h"])
    mx, my = cv2.initUndistortRectifyMap(K, np.asarray(cc["coeffs"], np.float64), None, K, (w, h), cv2.CV_32FC1)
    c = cv2.remap(color, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    d = cv2.remap(depth, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
    return c, d, K


def prepare(header, payloads, calib):
    """스냅샷(머리말, [컬러 바이트, 깊이 바이트]) → Frame(color BGR, depth uint16 mm, K)."""
    _model(calib["color"])
    color = decode_color(header["color"], payloads[0])
    frames = decode_depth(header["depth"], payloads[1], float(calib.get("depth_scale", 0.001)))
    depth = align_to_color(median_depth(frames), calib)
    return Frame(*undistort(color, depth, calib))
