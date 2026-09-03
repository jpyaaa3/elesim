# Developer Tools

`misc/tools/` is the repository's developer and CI workspace, not runtime code. It
contains commands used to inspect, reproduce, build, release and evaluate the
system. Reusable behavior belongs in its owning deployment package.

| Directory | Purpose | Start here |
| --- | --- | --- |
| `quality/` | Manual test and quality interfaces | `test_gui.py` |
| `release/` | Build and verify isolated role release contexts | `build.py` |
| `pilot_runtime.py` | In-process Pilot helper for misc/research/debug tools | `start_tool_pilot` |
| `setup_preview.py` | Side-effect-free browser preview of the setup wizard | `python3 misc/tools/setup_preview.py` |

```bash
python3 misc/tools/quality/test_gui.py
PYTHONPATH=payload/runtime/common/protocol:payload/runtime/docker/tools/app python3 -m elesim_setup.cli --help
python3 misc/research/analysis/analyze_walking_metrics.py --help
python3 misc/research/experiments/walking_baseline.py --help
python3 misc/tools/setup_preview.py
```

Do not import a tool from runtime code.  Tests may import a tool's pure helper
functions when that helper is specifically the subject under test.
