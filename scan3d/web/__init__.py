"""웹 화면 하나로 연결 확인, 맞추기, 스캔, 보강 촬영(docs/specs/webui.md).

  python -m scan3d.web [--fake] [--host 0.0.0.0] [--port 8080]

server·results는 표준 라이브러리만(tools/view_web.py가 호스트 python3로 쓴다).
session·preview·focus·holes는 스캔 PC 패키지(cv2, open3d, pyrealsense2)가 필요하다.
"""
