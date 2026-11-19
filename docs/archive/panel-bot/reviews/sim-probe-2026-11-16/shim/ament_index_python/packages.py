import os
from pathlib import Path


class PackageNotFoundError(KeyError):
    pass


def get_package_share_directory(name):
    root = Path(os.environ['PINKY_UPSTREAM'])
    path = root / name
    if not path.is_dir():
        raise PackageNotFoundError(name)
    return str(path)
