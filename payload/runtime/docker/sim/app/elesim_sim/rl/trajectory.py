"""Timed command artifacts shared by external collectors and replay tools.

Channels and frame/unit metadata are explicit. This format has no fixed task
observations, object shapes, rewards or success thresholds.
"""
from dataclasses import dataclass, field
import json
from pathlib import Path

import numpy as np


@dataclass
class Trajectory:
    times: np.ndarray
    commands: np.ndarray
    channels: tuple[str, ...]
    metadata: dict = field(default_factory=dict)
    diagnostics: dict = field(default_factory=dict)

    def validate(self):
        self.times = np.asarray(self.times, dtype=float)
        self.commands = np.asarray(self.commands, dtype=float)
        self.channels = tuple(self.channels)
        if (self.times.ndim != 1 or len(self.times) < 2
                or self.commands.shape != (len(self.times), len(self.channels))):
            raise ValueError('Require >=2 times and one command per named channel')
        if not np.isfinite(self.times).all() or not np.isfinite(self.commands).all():
            raise ValueError('Nonfinite trajectory')
        if self.times[0] != 0 or np.any(np.diff(self.times) <= 0):
            raise ValueError('Times must start at zero and strictly increase')
        if (not self.channels or any(not isinstance(c, str) or not c for c in self.channels)
                or len(set(self.channels)) != len(self.channels)):
            raise ValueError('Channels must be nonempty unique strings')
        if not isinstance(self.metadata, dict) or not isinstance(self.diagnostics, dict):
            raise ValueError('Metadata and diagnostics must be mappings')
        return self

    def sample(self, times):
        self.validate()
        times = np.asarray(times, dtype=float)
        if (not np.isfinite(times).all() or np.any(times < 0)
                or np.any(times > self.times[-1] + 1e-9)):
            raise ValueError('Replay query outside trajectory')
        return np.stack([
            np.interp(times, self.times, self.commands[:, i])
            for i in range(len(self.channels))
        ], axis=-1)

    def save(self, path):
        self.validate()
        # Serialize before creating an artifact so bad metadata leaves no file.
        body = json.dumps({
            'schema': 'elesim-command-trajectory', 'schema_version': 1,
            'times_s': self.times.tolist(), 'commands': self.commands.tolist(),
            'channels': list(self.channels), 'metadata': self.metadata,
            'diagnostics': self.diagnostics,
        }, allow_nan=False, indent=2)
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('x', encoding='utf-8') as stream:
            stream.write(body + '\n')

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if (not isinstance(data, dict) or data.get('schema') != 'elesim-command-trajectory'
                or data.get('schema_version') != 1):
            raise ValueError('Unsupported trajectory')
        return cls(data['times_s'], data['commands'], tuple(data['channels']),
                   data.get('metadata', {}), data.get('diagnostics', {})).validate()
