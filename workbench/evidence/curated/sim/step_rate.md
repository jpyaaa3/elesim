# T1 - wrap-scene step rate

Throughput of the contact-enabled wrap scene (arm + cylinder + floor + GO2 trunk) against the parallel-environment count.

## Measurement

- `20` warm-up steps, then `100` timed `scene.step()` calls.
- Each `N` runs in its own child process, so a point that runs out of memory is recorded as a failure instead of aborting the sweep.
- Physics `dt = 0.01` s, solver substeps `1`, `max_collision_pairs = 512`.
- Memory is peak process RSS. On unified-memory hosts (Apple silicon) that is the honest figure: there is no separate VRAM pool to read, and GPU-side allocations may not appear in RSS at all, so treat it as a lower bound.
- `with_contact = True`. When true the arm is ramped into a wrapping pose first, so the timed loop carries live contacts. A run with `live contacts = 0` is measuring a scene where contact is enabled but never happens, and its rate does not describe wrap grasping.

## Environment

| item | value |
|---|---|
| `platform` | macOS-26.3.1-arm64-arm-64bit |
| `machine` | arm64 |
| `processor` | arm |
| `python` | 3.11.16 |
| `torch` | 2.12.1 |
| `torch_cuda` | None |
| `cuda_available` | False |
| `mps_available` | True |
| `genesis` | 1.2.0 |
| `nvidia_smi` | not present |

Genesis backend requested: `gpu`. `torch_cuda` is the CUDA the torch wheel was built against, not `nvcc`.

## Results

| N envs | total steps/s | per-env steps/s | build s | peak RSS | device alloc | contact buffer | live contacts |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 256 | 18,558 | 72.5 | 9.0 | 0.94 GiB | 0.00 GiB | 10 | 0 |
| 1024 | 51,285 | 50.1 | 8.0 | 0.96 GiB | 0.00 GiB | 11 | 0 |
| 2048 | 48,696 | 23.8 | 17.7 | 0.90 GiB | 0.01 GiB | 10 | 0 |
| 4096 | 517,462 | 126.3 | 17.5 | 0.94 GiB | 0.01 GiB | 0 | 0 |

