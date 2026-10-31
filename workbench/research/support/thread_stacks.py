"""Periodic Python stacks with owned frame references and normal GIL handling.

CPython's native faulthandler watchdog walks concurrently changing frames
without the GIL. Keep fatal-signal handling separate from periodic sampling.
"""
from __future__ import annotations

import math
import sys
import threading
import time
import traceback


class ThreadStackSampler:
    def __init__(self,output,*,interval_s=1.,max_threads=32,max_frames=50):
        if not math.isfinite(interval_s) or interval_s <= 0:
            raise ValueError('stack interval must be finite and positive')
        if not 1 <= max_threads <= 100 or not 1 <= max_frames <= 100:
            raise ValueError('stack sampling bounds must be 1..100')
        self.output,self.interval_s = output,float(interval_s)
        self.max_threads,self.max_frames = max_threads,max_frames
        self._stop = threading.Event()
        self._thread = None
        self.samples = 0
        self.error = ''

    def start(self):
        if self._thread is not None:
            raise RuntimeError('stack sampler has already started')
        self._thread = threading.Thread(target=self._run,name='python-stack-sampler',daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()

    def close(self,timeout_s=2.):
        self.stop()
        if self._thread is not None:
            self._thread.join(timeout_s)
            if self._thread.is_alive():
                raise TimeoutError('stack sampler has not stopped')

    def _run(self):
        try:
            while not self._stop.wait(self.interval_s):
                # sys._current_frames returns owning references under the GIL.
                # Keep them alive while extracting stacks, even when file I/O
                # temporarily releases the GIL. Never capture frame locals.
                frames = sys._current_frames()
                try:
                    names = {thread.ident:thread.name for thread in threading.enumerate()}
                    text = [f'Stack sample wall={time.time():.6f}\n']
                    for ident,frame in list(frames.items())[:self.max_threads]:
                        text.append(f'Thread {ident} ({names.get(ident,"unknown")}):\n')
                        text.extend(traceback.format_list(traceback.extract_stack(frame,limit=self.max_frames)))
                    self.output.write(''.join(text))
                    self.output.flush()
                    self.samples += 1
                finally:
                    frames.clear()
                    frame = None
        except Exception as error:
            self.error = f'{type(error).__name__}: {error}'
