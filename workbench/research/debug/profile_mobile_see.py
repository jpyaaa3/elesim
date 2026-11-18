"""Run the published Pilot with bounded SEE-stage timing and Python stacks.

Use only in an owned diagnostic replacement of a stopped Pilot endpoint. This
does not change planner results, actuator commands, authority or time limits.
It is diagnostic evidence, not ordinary installed-runtime acceptance.
"""
from __future__ import annotations

import argparse
import faulthandler
import functools
import sys
import time

# Setuptools' production console script imports Pilot.main even when spawn
# executes it as __mp_main__. Preserve that import cost/side effects in workers.
if __name__ == '__mp_main__':
    import elesim_pilot.main


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stack-period', type=float, default=1.)
    args, pilot_args = parser.parse_known_args()
    if not .25 <= args.stack_period <= 10.:
        parser.error('stack period must be .25..10 seconds')
    from elesim_pilot.mobile_manipulation.see_planner import MobileSeePlanner
    from elesim_pilot.mobile_manipulation.scan_motion import ScanMotion
    from elesim_pilot.vision.see import SeeBackend
    from elesim_pilot.main import main as pilot_main
    stack_depth = 0

    def instrument(cls, name, *, stacks=False):
        original = getattr(cls, name)

        @functools.wraps(original)
        def traced(*values, **options):
            nonlocal stack_depth
            started = time.monotonic()
            label = cls.__name__ + '.' + name
            print('MOBILE_SEE_PROFILE begin', label, 'wall', time.time(), flush=True)
            if stacks:
                if stack_depth == 0:
                    faulthandler.dump_traceback_later(args.stack_period, repeat=True)
                stack_depth += 1
            try:
                return original(*values, **options)
            finally:
                if stacks:
                    stack_depth -= 1
                    if stack_depth == 0:
                        faulthandler.cancel_dump_traceback_later()
                print('MOBILE_SEE_PROFILE end', label, 'elapsed_s', time.monotonic()-started, flush=True)
        setattr(cls, name, traced)

    instrument(MobileSeePlanner, 'choose', stacks=True)
    for name in ('frontiers', '_choose_mobile', '_information_view'):
        instrument(MobileSeePlanner, name)
    instrument(MobileSeePlanner, 'motion', stacks=True)
    instrument(ScanMotion, 'observe_floor')
    instrument(SeeBackend, 'propose')
    print('MOBILE_SEE_PROFILE diagnostic installed Pilot; planner/commands/limits unchanged', flush=True)
    sys.argv[1:] = pilot_args
    try:
        pilot_main()
    finally:
        faulthandler.cancel_dump_traceback_later()


if __name__ == '__main__':
    main()
