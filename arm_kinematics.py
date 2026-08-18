#!/usr/bin/env python3
"""Compatibility CLI entry point."""

from pathlib import Path
import sys

SRC = Path(__file__).resolve().parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from arm3dof.cli import run


if __name__ == "__main__":
    run()
