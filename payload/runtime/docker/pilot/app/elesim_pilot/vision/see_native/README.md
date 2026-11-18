# Official SEE core adapter

Upstream: https://github.com/robotic-esp/see-public
Pinned commit: b9f29b6daa4ed9322bcbdbef855a34ac048d3c11 (MIT).
`upstream/` preserves the selected original source files and LICENSE.

`prepare.py` creates a build-only overlay: ROS logging removal, PCL 1.12 std
shared-pointer compatibility, and removal of unused visualization/mesh headers.
Frontier classification and visibility remain upstream C++. The reactive
`ViewController` call is disabled in this adapter: EleSim may execute a different
reachable proposal than `GetNBV()`, so attributing its observations to the latter
frontier would be incorrect. Actual measured observations still reclassify points;
EleSim records attempted configurations to avoid retries. `bridge.cpp` exports
active frontier proposals, not stale entries in GetViewCloud. It runs one OpenMP
thread and rejects proposal overflow. Python isolates native execution in a
cancellable worker; it never issues robot commands.

Build inside the supported development attachment (PCL 1.12+, Eigen, Boost,
NLOPT C++ headers, Lemon, OpenMP and CMake required):

    cmake -S <this-directory> -B <build-directory> -DCMAKE_BUILD_TYPE=Release
    cmake --build <build-directory> -j2

Install `libelesim_see.so` at `/opt/elesim/native/libelesim_see.so`.
Pilot Docker release does this automatically. No ROS 1 installation is needed.
The source wheel includes the upstream source/notices and build files; the
platform-specific shared library is built inside the destination image.

SEE frontier exhaustion is not a complete-surface or safe-floor certificate.
The generic backend retains upstream radius/density defaults. The experimental
EleSim planner uses radius 0.01 m, density 5,000,000 points/m³ and view distance
0.35 m, calibrated on two recorded scans capped at 20,000 points per frame.
This is not a sensor-independent calibration or a reconstruction-quality guarantee.
Standalone Object Scan selects this adapter with `ELESIM_SCAN_VIEW_POLICY=see`.
Its default is `information`. The mobile workflow selects SEE through
`mobile_manipulation.yaml` and solves camera goals jointly over base X/Y/yaw and
four arm channels. Mobile return cost targets policy Home at the candidate base.
Standalone return cost targets the first scan posture. Missing native libraries
fail explicitly. The native worker returns geometry proposals to Pilot's planner.
