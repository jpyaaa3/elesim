#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`tools.experiments.walking_baseline`."""

from tools.experiments.walking_baseline import *  # noqa: F401,F403
from tools.experiments.walking_baseline import _parse_gaze, _validate_gaze_config, main


if __name__ == "__main__":
    main()
