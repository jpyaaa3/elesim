# System tests

These tests are run by developers and CI, not by deployed EleSim processes.
They launch or construct multiple runtime participants and verify the system
boundaries between them.

- `smoke_topology.py`: four-process Router-free DDS topology smoke test
- `test_dds_rgbd.py`: latest-only typed RGBD DDS behavior
- `test_webrtc_media.py`: independent encoded WebRTC stream behavior

Application unit and scenario tests remain beside their owning deployment.
