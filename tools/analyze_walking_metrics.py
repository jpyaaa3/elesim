#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`tools.analysis.analyze_walking_metrics`."""

from tools.analysis.analyze_walking_metrics import *  # noqa: F401,F403
from tools.analysis.analyze_walking_metrics import _nearest_merge, _visibility_lost_counts, main


if __name__ == "__main__":
    main()
