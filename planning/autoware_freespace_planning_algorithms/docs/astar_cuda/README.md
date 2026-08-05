# CUDA Hybrid A* for Autoware Freespace Planning

Optional GPU backend for Autoware freespace/parking planning. It accelerates the
core **Hybrid A\*** logic used by `autoware_freespace_planner` while keeping the
same ROS interfaces, vehicle model, and parameter set as the CPU planner.

| Layer | Package | Role |
|-------|---------|------|
| Node | `autoware_freespace_planner` | Scenario/route/odometry → trajectory; selects algorithm |
| Algorithms | `autoware_freespace_planning_algorithms` | CPU `AstarSearch` / RRT* / CUDA `AstarSearchCuda` |

Select the backend with:

```yaml
planning_algorithm: "astar_cuda"   # options: astar, astar_cuda, rrtstar
```

Default remains `"astar"` (CPU). If CUDA was not built or no GPU is available at
runtime, the node logs a warning and falls back to CPU Hybrid A\*.

## Architecture

- **Host open-list** preserves classic A\* node ordering.
- **CUDA expand kernel** evaluates all steering × forward/back successors in
  parallel (kinematics, footprint collision, EDT costs, **device Reeds–Shepp**
  heuristic).
- **GPU EDT** recomputes the obstacle distance field on `setMap`.
- Map buffers (obstacles, EDT, collision-index tables) stay resident on device
  across replans of the same map size.

```
FreespacePlannerNode
        │
        ├── planning_algorithm: astar       → AstarSearch (CPU)
        ├── planning_algorithm: astar_cuda  → AstarSearchCuda
        └── planning_algorithm: rrtstar     → RRTStar (CPU)
```

### Source layout

```
include/.../astar_search_cuda.hpp
include/.../astar_cuda_kernels.hpp
include/.../astar_cuda_types.hpp
include/.../reeds_shepp_device.cuh
src/cuda/
  astar_search_cuda.cpp
  astar_cuda_kernels.cu
test/src/
  test_astar_cuda_correctness.cpp
  bench_astar_cpu_vs_cuda.cpp
docs/astar_cuda/README.md
```

## Build

Requires CUDA toolkit + NVIDIA driver. If CUDA is not found, the package builds
without `astar_cuda`.

```bash
colcon build --packages-up-to autoware_freespace_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Expected performance

Targets assume Ampere-or-newer GPU, parking maps (~150×150, `theta_size` ≈ 144).

| Workload | Expected GPU vs CPU |
|----------|---------------------|
| EDT in `setMap` | ~5–20× on large maps |
| Expand (collision + RS per successor batch) | meaningful when many steers / back gear |
| Easy `makePlan` | ~0.8–1.5× (may be flat; launch overhead) |
| Hard `makePlan` | ~2–5× median (scene / GPU dependent) |

## Tests

```bash
# Correctness (RS parity, plan succeeds, collision-free)
colcon test --packages-select autoware_freespace_planning_algorithms \
  --ctest-args -R test_astar_cuda_correctness

# CPU vs GPU bench (writes /tmp/fpalgos-astar-cuda-bench.csv)
ros2 run autoware_freespace_planning_algorithms astar_cpu_vs_cuda_bench
```

Scenarios: S1 easy, S2 hard parking, S3 denser map, S4 warm replans.

## References

- CPU Hybrid A\*: `src/astar_search.cpp`
- GPU patterns (not a dependency): [NVlabs/curobo](https://github.com/NVlabs/curobo)
- Device Reeds–Shepp ported from `src/reeds_shepp.cpp` (BSD-3 upstream)
