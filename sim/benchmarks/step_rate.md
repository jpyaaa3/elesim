# T1 - wrap-scene step rate

Throughput of the contact-enabled wrap scene (arm + cylinder + floor + GO2 trunk) against the parallel-environment count.

## Measurement

- `20` warm-up steps, then `100` timed `scene.step()` calls.
- Each `N` runs in its own child process, so a point that runs out of memory is recorded as a failure instead of aborting the sweep.
- Physics `dt = 0.01` s, solver substeps `1`, `max_collision_pairs = 512`.
- Memory is peak process RSS. On unified-memory hosts (Apple silicon) that is the honest figure: there is no separate VRAM pool to read.

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

| N envs | total steps/s | per-env steps/s | build s | peak RSS | device alloc | contact buffer |
|---:|---:|---:|---:|---:|---:|---:|
| 64 | 16,001 | 250.0 | 9.7 | 0.88 GiB | 0.00 GiB | 5 |
| 256 | 56,751 | 221.7 | 12.1 | 0.92 GiB | 0.00 GiB | 5 |
| 1024 | 137,180 | 134.0 | 13.2 | 0.89 GiB | 0.00 GiB | 5 |
| 4096 | 543,339 | 132.7 | 29.5 | 0.84 GiB | 0.01 GiB | 5 |

