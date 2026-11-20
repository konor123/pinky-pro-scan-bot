"""시뮬레이션 카메라 변환(ROS 없이): CameraInfo → calib, 프레임 바이트."""
import types

import numpy as np
import pytest

from scan3d_sim.sim_camera import (calib_from_camera_info, calib_from_info_msg, pack_color_frame,
                                   pack_depth_frame, pack_depth_frames)


def test_camera_info_to_calib():
    k = [337.3, 0.0, 319.5, 0.0, 337.4, 179.5, 0.0, 0.0, 1.0]
    calib = calib_from_camera_info(640, 360, k)
    assert set(calib.keys()) == {"color", "depth", "depth_to_color", "depth_scale"}
    assert calib["color"] == calib["depth"]
    assert calib["color"] == {
        "w": 640,
        "h": 360,
        "fx": 337.3,
        "fy": 337.4,
        "ppx": 319.5,
        "ppy": 179.5,
        "model": "none",
        "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    }
    assert calib["depth_to_color"] == {
        "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "translation": [0.0, 0.0, 0.0],
    }
    assert calib["depth_scale"] == pytest.approx(0.001)

    msg_lower = types.SimpleNamespace(width=640, height=360, k=k)
    msg_upper = types.SimpleNamespace(width=640, height=360, K=k)
    assert calib_from_info_msg(msg_lower) == calib
    assert calib_from_info_msg(msg_upper) == calib

    with pytest.raises(ValueError):
        calib_from_camera_info(0, 360, k)
    with pytest.raises(ValueError):
        calib_from_camera_info(640, 360, k[:8])
    with pytest.raises(ValueError):
        calib_from_camera_info(640, 360, [0.0, 0.0, 319.5, 0.0, 337.4, 179.5, 0.0, 0.0, 1.0])


def test_frame_packing_sizes_and_formats():
    rng = np.random.default_rng(0)
    w, h = 640, 360
    rgb = rng.integers(0, 256, (h, w, 3), dtype=np.uint8)
    c_hdr, c_bytes = pack_color_frame(w, h, "rgb8", rgb.tobytes())
    assert c_hdr == {"w": w, "h": h, "fmt": "rgb8"}
    assert len(c_bytes) == w * h * 3
    assert c_bytes == rgb.tobytes()

    padded_rgb = np.zeros((h, w * 3 + 16), dtype=np.uint8)
    padded_rgb[:, : w * 3] = rgb.reshape(h, w * 3)
    c_hdr_pad, c_bytes_pad = pack_color_frame(w, h, "rgb8", padded_rgb.tobytes(), step=w * 3 + 16)
    assert c_hdr_pad == c_hdr
    assert c_bytes_pad == c_bytes

    with pytest.raises(ValueError):
        pack_color_frame(w, h, "bgr8", rgb.tobytes())

    depth_f32 = rng.uniform(0.4, 1.5, (h, w)).astype("<f4")
    f1 = pack_depth_frame(w, h, "32FC1", depth_f32.tobytes())
    d_hdr1, d_bytes1 = pack_depth_frames([f1])
    assert d_hdr1 == {"w": w, "h": h, "fmt": "f32", "n": 1}
    assert len(d_bytes1) == w * h * 4
    assert d_bytes1 == depth_f32.tobytes()

    frames5 = [pack_depth_frame(w, h, "32FC1", (depth_f32 + i * 0.01).astype("<f4").tobytes()) for i in range(5)]
    d_hdr5, d_bytes5 = pack_depth_frames(frames5)
    assert d_hdr5 == {"w": w, "h": h, "fmt": "f32", "n": 5}
    assert len(d_bytes5) == w * h * 4 * 5

    padded_d = np.zeros((h, w * 4 + 12), dtype=np.uint8)
    padded_d[:, : w * 4] = np.frombuffer(depth_f32.tobytes(), dtype=np.uint8).reshape(h, w * 4)
    _, d_bytes_pad = pack_depth_frames([pack_depth_frame(w, h, "32FC1", padded_d.tobytes(), step=w * 4 + 12)])
    assert d_bytes_pad == depth_f32.tobytes()

    depth_be = depth_f32.astype(">f4")
    _, d_bytes_be = pack_depth_frames([pack_depth_frame(w, h, "32FC1", depth_be.tobytes(), is_bigendian=True)])
    assert d_bytes_be == depth_f32.tobytes()

    with pytest.raises(ValueError):
        pack_depth_frame(w, h, "16UC1", depth_f32.tobytes())
    with pytest.raises(ValueError):
        pack_depth_frames([f1, pack_depth_frame(320, 180, "32FC1", np.zeros((180, 320), "<f4").tobytes())])
