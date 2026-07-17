#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`tools.experiments.run_walking_baseline_batch`."""

from tools.experiments.run_walking_baseline_batch import *  # noqa: F401,F403
from tools.experiments.run_walking_baseline_batch import main


if __name__ == "__main__":
    main()
