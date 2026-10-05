#!/usr/bin/env python3
"""
Download the production detector weights and verify their SHA-256.

    python3 scripts/fetch_models.py

Files land in laptop_ws/src/perception_pkg/perception_pkg/ (ignored by git);
rebuild the workspace afterwards so they are installed. A file already present
with the right hash is left alone.
"""

import argparse
import hashlib
from pathlib import Path
import sys
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'laptop_ws' / 'src' / 'perception_pkg' / 'perception_pkg'
RELEASE_URL = 'https://github.com/DangTuyen0604/robot_systems/releases/download/models-v1/'

# name: (sha256, purpose)
MODELS = {
    'yolov8n.onnx': (
        '6bd6b0deb7671c10754cd7df8b85fb43689f9830435a2c7d8cc9353980b2b026',
        'sign detector, ONNX 320 px (simulation)'),
    'viet.pt': (
        'eeab84b4ef1c5818417bb34abaac237c95857449b57de66cbf432267caaef8f7',
        'sign detector, PyTorch checkpoint (real robot)'),
}


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def from_release(name, target):
    with urllib.request.urlopen(RELEASE_URL + name, timeout=60) as response:
        target.write_bytes(response.read())


def main():
    argparse.ArgumentParser(description=__doc__.splitlines()[1]).parse_args()

    failed = False
    for name, (expected, purpose) in MODELS.items():
        target = DEST / name
        if target.exists() and sha256(target) == expected:
            print(f'ok       {name} ({purpose})')
            continue
        with tempfile.NamedTemporaryFile(dir=DEST, delete=False) as tmp:
            tmp_path = Path(tmp.name)
        try:
            from_release(name, tmp_path)
            actual = sha256(tmp_path)
            if actual != expected:
                raise RuntimeError(f'SHA-256 mismatch: got {actual}, expected {expected}')
            tmp_path.replace(target)
            print(f'fetched  {name} ({purpose})')
        except Exception as exc:  # report every model, then fail
            print(f'FAILED   {name}: {exc}', file=sys.stderr)
            failed = True
        finally:
            tmp_path.unlink(missing_ok=True)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
