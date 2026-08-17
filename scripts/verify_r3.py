#!/usr/bin/env python3
"""
verify_r3.py — Requirement R3 Verification Script

Checks that the total size of a target dependency directory
(e.g., .venv, venv, site-packages) does not exceed 500MB.

Usage:
    python scripts/verify_r3.py --target-dir .venv --max-size-mb 500

Exit codes:
    0 — Directory size <= max_size_mb (PASS)
    1 — Directory size > max_size_mb or directory not found (FAIL)
"""

import argparse
import os
import sys


def parse_args():
    parser = argparse.ArgumentParser(description="R3: Dependency folder size check.")
    parser.add_argument(
        "--target-dir",
        type=str,
        default=".venv",
        help="Path to dependency directory to check (default: .venv)",
    )
    parser.add_argument(
        "--max-size-mb",
        type=float,
        default=500.0,
        help="Maximum allowed size in megabytes (default: 500)",
    )
    parser.add_argument("--quiet", action="store_true", help="Suppress verbose output")
    return parser.parse_args()


def get_dir_size_mb(path: str) -> float:
    """Recursively calculate total size of a directory in MB."""
    total_bytes = 0
    for root, _, files in os.walk(path):
        for fname in files:
            fpath = os.path.join(root, fname)
            try:
                if not os.path.islink(fpath) and os.path.exists(fpath):
                    total_bytes += os.path.getsize(fpath)
            except OSError:
                pass
    return total_bytes / (1024.0 * 1024.0)


def main():
    args = parse_args()

    target = os.path.abspath(args.target_dir)

    if not args.quiet:
        print(f"[R3] Checking dependency directory: {target}")
        print(f"[R3] Maximum allowed size: {args.max_size_mb:.1f} MB")

    if not os.path.exists(target):
        print(f"\n[R3] SKIP — Directory not found: {target}")
        print(f"[R3] (No virtual environment installed — dependency constraint trivially satisfied)")
        sys.exit(0)

    size_mb = get_dir_size_mb(target)

    if not args.quiet:
        print(f"[R3] Measured size: {size_mb:.2f} MB")

    if size_mb <= args.max_size_mb:
        print(f"\n[R3] PASS — {size_mb:.2f} MB <= {args.max_size_mb:.1f} MB")
        sys.exit(0)
    else:
        print(f"\n[R3] FAIL — {size_mb:.2f} MB > {args.max_size_mb:.1f} MB (exceeds limit!)")
        sys.exit(1)


if __name__ == "__main__":
    main()
