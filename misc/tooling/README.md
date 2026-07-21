# Developer Tools

`misc/tooling/` is the repository's operator workspace, not runtime code. It contains
commands used to inspect, reproduce, run, and evaluate the system.  Reusable
behavior belongs in its owning deployment package.

| Directory | Purpose | Start here |
| --- | --- | --- |
| `analysis/` | Read logs, compare trials, generate figures | `analyze_walking_metrics.py` |
| `debug/` | Targeted manual diagnostics and runtime previews | `run_grasp_guided_mock.py` |
| `experiments/` | Repeatable walking/gaze experiment runners and batch scripts | `walking_baseline.py` |
| `quality/` | Manual test and quality interfaces | `test_gui.py` |

```bash
python3 misc/tooling/quality/test_gui.py
python3 misc/tooling/analysis/analyze_walking_metrics.py --help
python3 misc/tooling/experiments/walking_baseline.py --help
```

Do not import a tool from runtime code.  Tests may import a tool's pure helper
functions when that helper is specifically the subject under test.
