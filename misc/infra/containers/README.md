# Container installation templates

These Dockerfiles are inputs to `elesim-setup`, not a sixth deployment. The
installer creates one isolated build context per selected role containing only
that role and `elesim_protocol`, then writes a Linux host-network Compose
project. Do not build these files directly from the repository root.

The generic Robot image is intentionally absent. Physical Robot deployment is
tied to JetPack/L4T, ROS2, `unitree_ros2`, camera devices and serial permissions;
use the Robot native release on the target Jetson.

Simulator uses the Ubuntu 22.04 ROS base and Robotpkg Pinocchio and is currently
limited to `linux/amd64`. Other roles use the smaller Python base. Compute roles
preinstall the locked Torch build from the official CPU wheel index in CPU mode
or PyPI in GPU/inherited mode, preventing optional packages from silently
selecting a newer CUDA stack.

Robotpkg does not expose HTTPS. Its public signing key is therefore pinned as
`robotpkg.asc` in this directory and copied into the image; never replace that
with a build-time HTTP key download. Review the pinned key fingerprint when
rotating it: `F6F9 3D4D 4258 60C0 B0FB E848 ADD5 35E0 5E56 C3FD`.
