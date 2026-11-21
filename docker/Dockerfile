# 스캔 PC 환경(Ubuntu 24.04 + Python 3.12 + pyproject.toml 고정 버전 패키지).
# 코드는 넣지 않고 실행 때 저장소를 /work에 마운트한다(PYTHONPATH=/work). 사용법은 docker/README.md.
# amd64·arm64 둘 다 된다(Apple Silicon Mac은 에뮬레이션 없이 arm64로 빌드).
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-venv python3-pip \
        # open3d·opencv 런타임
        libgl1 libegl1 libgomp1 libgfortran5 libx11-6 libxext6 libxrender1 libglib2.0-0 \
        # pyrealsense2(libusb), 로봇 원격 작업(ssh·rsync), 다운로드(curl)
        libusb-1.0-0 openssh-client rsync curl ca-certificates git \
    && rm -rf /var/lib/apt/lists/*

# 의존성만 먼저 설치(pyproject.toml이 바뀔 때만 다시 받는다)
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH=$VIRTUAL_ENV/bin:$PATH
COPY pyproject.toml /tmp/scan3d/pyproject.toml
RUN python - <<'PY' > /tmp/scan3d/requirements.txt
import tomllib
p = tomllib.load(open("/tmp/scan3d/pyproject.toml", "rb"))["project"]
print("\n".join(p["dependencies"] + p.get("optional-dependencies", {}).get("test", [])))
PY
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir -r /tmp/scan3d/requirements.txt

# 저장소 스크립트(tools/scan.sh, scripts/*.sh)가 .venv 대신 이 파이썬을 쓰게 한다
ENV SCAN3D_PY=$VIRTUAL_ENV/bin/python \
    PYTHONPATH=/work \
    PYTHONDONTWRITEBYTECODE=1 \
    OPEN3D_CPU_RENDERING=true

COPY docker/entrypoint.sh /usr/local/bin/scan3d-entrypoint
RUN chmod +x /usr/local/bin/scan3d-entrypoint
WORKDIR /work
ENTRYPOINT ["scan3d-entrypoint"]
CMD ["bash"]
