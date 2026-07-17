#!/usr/bin/env python3
"""Compatibility module and launcher for the host process.

The implementation lives in :mod:`apps.host.main`.  Re-export its public
surface so existing operational scripts and external integrations can keep
using ``import host`` while the composition root has a stable home.
"""

from apps.host.main import *  # noqa: F401,F403
from apps.host.main import _DirectEmbeddedControlClient, main


if __name__ == "__main__":
    main()
