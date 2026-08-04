# Developer Tools

`tools/` is the repository's developer and CI workspace, not runtime code. It
contains commands used to inspect, reproduce, build, release and evaluate the
system. Reusable behavior belongs in its owning deployment package.

| Directory | Purpose | Start here |
| --- | --- | --- |
| `quality/` | Manual test and quality interfaces | `test_gui.py` |
| `release/` | Build and verify isolated role release contexts | `build.py` |
| `pilot_runtime.py` | In-process Pilot helper for research/debug tools | `start_tool_pilot` |

```bash
python3 tools/quality/test_gui.py
PYTHONPATH=packages/protocol/src:installer/package/src python3 -m elesim_setup.cli --help
python3 research/analysis/analyze_walking_metrics.py --help
python3 research/experiments/walking_baseline.py --help
```

Do not import a tool from runtime code.  Tests may import a tool's pure helper
functions when that helper is specifically the subject under test.
