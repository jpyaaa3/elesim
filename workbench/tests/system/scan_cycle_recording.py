"""Record the real UI-owned WebRTC decoders without opening a native window."""
from __future__ import annotations

from fractions import Fraction
import json
from pathlib import Path
import threading
import time


class FrameRecorder:
    def __init__(self, session, service, output, stage, *, fps=10):
        self.session, self.service, self.output, self.stage = session,service,Path(output),stage
        self.fps = fps
        self.stop = threading.Event()
        self.thread = None
        self.rows, self.error = [], ''
        self.streams = ('hand_eye_preview','observer')
        self.files = {name:name+'.mp4' for name in self.streams}
        self.started_wall = self.started_mono = 0.
        self.duration_s = 0.
        self.counts = {name:0 for name in self.streams}

    def start(self):
        self.output.mkdir(exist_ok=True)
        self.started_wall, self.started_mono = time.time(), time.monotonic()
        self.thread = threading.Thread(target=self._run,name='scan-cycle-video',daemon=True)
        self.thread.start()

    def _run(self):
        import av
        import numpy as np

        encoders = {}
        try:
            for name in self.streams:
                frame = self.session.frame(name)
                if frame is None:
                    raise RuntimeError('live decoded frame required before recording: '+name)
                height,width = frame.shape[:2]
                container = av.open(str(self.output/self.files[name]),mode='w')
                stream = container.add_stream('libx264',rate=self.fps)
                stream.width, stream.height, stream.pix_fmt = width,height,'yuv420p'
                stream.codec_context.thread_count = 1
                stream.codec_context.time_base = Fraction(1,1000)
                stream.codec_context.gop_size = self.fps*2
                stream.options = {'preset':'veryfast','crf':'25','tune':'zerolatency'}
                encoders[name] = container,stream,(height,width,3)
            deadline = time.monotonic()
            while not self.stop.is_set():
                now = time.monotonic()
                elapsed = now-self.started_mono
                if elapsed > 900:
                    raise TimeoutError('recording exceeded 900 seconds')
                scan = self.service.object_scan
                row = dict(t_s=elapsed,stage=self.stage(),phase=scan.get('phase','idle'),
                    view=scan.get('view'),views=scan.get('views',3),
                    accepted_views=scan.get('accepted_views',0),observed_points=scan.get('observed_points',0),
                    frames={})
                for name,(container,stream,shape) in encoders.items():
                    source = self.session.frame(name)
                    frame = np.zeros(shape,np.uint8) if source is None else source.copy()
                    if frame.shape != shape:
                        raise ValueError('decoder dimensions changed during recording')
                    row['frames'][name] = dict(available=source is not None,
                        version=self.session.frame_version(name))
                    video = av.VideoFrame.from_ndarray(frame,format='bgr24')
                    video.pts, video.time_base = round(elapsed*1000),Fraction(1,1000)
                    for packet in stream.encode(video):
                        container.mux(packet)
                    self.counts[name] += 1
                self.rows.append(row)
                deadline += 1./self.fps
                if deadline < time.monotonic():
                    deadline = time.monotonic()
                self.stop.wait(max(0.,deadline-time.monotonic()))
        except Exception as exc:
            self.error = type(exc).__name__+': '+str(exc)
        finally:
            for container,stream,_shape in encoders.values():
                try:
                    for packet in stream.encode():
                        container.mux(packet)
                    container.close()
                except Exception as exc:
                    self.error = self.error or type(exc).__name__+': '+str(exc)
            self.duration_s = time.monotonic()-self.started_mono

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(10.)
            if self.thread.is_alive():
                self.error = 'video encoder did not stop within 10 seconds'
        metadata = dict(complete=not self.error and all(self.counts.values()),error=self.error,
            fps=self.fps,files=self.files,frames=self.counts,started=self.started_wall,
            duration_s=self.duration_s,clock='actual wall time; original decoder frames, no speed change')
        (self.output/'video-timeline.json').write_text(json.dumps(dict(metadata=metadata,samples=self.rows),
            indent=2,allow_nan=False)+'\n')
        return metadata
