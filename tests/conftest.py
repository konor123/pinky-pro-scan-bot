from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return ROOT


@pytest.fixture
def cfg() -> dict:
    """config/scan.yaml을 테스트마다 새로 읽은 dict. 테스트 안에서 값을 바꿔 써도 된다."""
    with open(ROOT / "config" / "scan.yaml", encoding="utf-8") as f:
        return yaml.safe_load(f)


@pytest.fixture
def rng() -> np.random.Generator:
    return np.random.default_rng(0)
