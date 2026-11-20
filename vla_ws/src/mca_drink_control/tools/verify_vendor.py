#!/usr/bin/env python3
"""Verify vendored MCA files against pinned upstream Git blob object IDs."""
from __future__ import annotations

import hashlib
import pathlib
import re
import sys

LOCK = pathlib.Path(__file__).resolve().parents[1] / "MCA_UPSTREAM.lock"
VENDOR = pathlib.Path(__file__).resolve().parents[1] / "third_party" / "MotionControlAlgorithms"


def main() -> int:
    lock_text = LOCK.read_text(encoding="utf-8")
    pairs = re.findall(
        r"^  - path: (.+)\n    git_blob_sha1: ([0-9a-f]{40})$",
        lock_text,
        flags=re.MULTILINE,
    )
    if not pairs:
        print("MCA lock has no file integrity entries", file=sys.stderr)
        return 2
    failures = []
    for relative, expected in pairs:
        path = VENDOR / relative
        if not path.is_file():
            failures.append(f"missing: {relative}")
            continue
        data = path.read_bytes()
        header = f"blob {len(data)}\0".encode("ascii")
        actual = hashlib.sha1(header + data).hexdigest()
        if actual != expected:
            failures.append(f"hash mismatch: {relative}: expected {expected}, got {actual}")
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print(f"verified {len(pairs)} pinned MCA files byte-for-byte (Git blob SHA-1)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
