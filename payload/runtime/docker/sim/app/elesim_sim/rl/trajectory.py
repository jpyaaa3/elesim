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
        self.commands = np.asarray(self.commands)
        if self.commands.dtype.kind not in 'fiu':
            raise ValueError('Commands must be real numeric arrays')
        if self.commands.dtype.kind != 'f':
            self.commands = self.commands.astype(float)
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
        path = Path(path)
        if path.suffix.lower() != '.npz':
            raise ValueError('Trajectory output must end in .npz')
        metadata = json.dumps(self.metadata, allow_nan=False)
        diagnostics = json.dumps(self.diagnostics, allow_nan=False)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('xb') as stream:
            np.savez_compressed(
                stream, schema=np.asarray('elesim-command-trajectory'),
                schema_version=np.asarray(2), times_s=self.times, commands=self.commands,
                channels=np.asarray(self.channels, dtype=str),
                metadata_json=np.asarray(metadata), diagnostics_json=np.asarray(diagnostics),
            )

    @classmethod
    def load(cls, path):
        path = Path(path)
        if path.suffix.lower() == '.json':
            # Read-only migration path for previously collected trajectories.
            data = json.loads(path.read_text(encoding='utf-8'))
            if data.get('schema') != 'elesim-command-trajectory' or data.get('schema_version') != 1:
                raise ValueError('Unsupported legacy trajectory')
            return cls(data['times_s'], data['commands'], tuple(data['channels']),
                       data.get('metadata', {}), data.get('diagnostics', {})).validate()
        with np.load(path, allow_pickle=False) as data:
            if (str(data['schema'].item()) != 'elesim-command-trajectory'
                    or int(data['schema_version'].item()) != 2):
                raise ValueError('Unsupported trajectory NPZ')
            return cls(data['times_s'].copy(), data['commands'].copy(),
                       tuple(data['channels'].tolist()),
                       json.loads(str(data['metadata_json'].item())),
                       json.loads(str(data['diagnostics_json'].item()))).validate()
