# Jetson GO2 + arm installation, Mac UI/Sim

## Jetson 부팅 자동 실행

Robot 서비스는 이미 부팅 자동 실행이 활성화되어 있다. Pilot 서비스 파일과
DDS 네트워크 준비 대기 도구는 Jetson `~/elesim/local/systemd/`에 준비했다.
서비스 파일·Python 진입점·현재 Wi-Fi IPv4 준비 검증을 완료했다. sudo는
비밀번호를 요구하여 아직 시스템 서비스로 등록하거나 실행하지 않았다.

Jetson에서 한 번 실행한다. `--now`를 사용하지 않으므로 현재 실행 중인
수동 Pilot에는 영향을 주지 않고, 다음 부팅부터 서비스로 실행된다.

```bash
sudo install -m 644 ~/elesim/local/systemd/elesim-pilot.service /etc/systemd/system/elesim-pilot.service
sudo systemctl daemon-reload
sudo systemctl enable elesim-robot.service elesim-pilot.service
```

Pilot은 DDS에 설정된 Wi-Fi 인터페이스의 IPv4를 최대 120초 기다리고,
실패하면 5초 뒤 재시도한다. Robot보다 뒤에 시작하도록 순서를 지정하되
Robot이 아직 발견되지 않아도 Pilot은 실행되어 재연결을 기다린다.
현재 GO2 startup의 Stand/Gait는 `none`이고 Pilot/Robot 시작 코드에서
Torque ON·Home·계획 경로 실행을 자동 요청하지 않는다. 모터 전원을 유지한
채 Jetson만 재부팅하는 경우 모터 자체의 기존 토크 상태까지 OFF라고
가정하지 않는다. `use_go2: true`인 현재 Robot은 GO2 링크/bridge 준비도 필요하다.

자동 실행 등록 뒤에는 Pilot을 별도 Python 터미널로 중복 실행하지 않는다.
재부팅 없이 지금 서비스 방식으로 전환하려면 먼저 기존 수동 Pilot을
Ctrl+C로 종료한 뒤 실행한다.

```bash
sudo systemctl start elesim-pilot.service
```

이후 유지보수용 명령:

```bash
systemctl status elesim-robot.service elesim-pilot.service --no-pager
sudo journalctl -u elesim-robot.service -u elesim-pilot.service -f
sudo systemctl restart elesim-pilot.service
```

## 최신 실행 순서: H3C_D41BD2 + Ubuntu UI

Ubuntu PC와 Jetson을 `H3C_D41BD2`에 연결한 다음 실행한다. 기존 Pilot·UI와
Mac Sim이 실행 중이면 각 터미널에서 Ctrl+C로 종료한다. 멀티캐스트 자동
발견을 사용하므로 AP 이동 후 새 IP를 설정 파일에 입력할 필요는 없다.

**1. Jetson: Robot 재시작**

```bash
sudo systemctl restart elesim-robot.service
```

**2. Jetson: Pilot 실행**

위 자동 실행 등록 후에는 `sudo systemctl restart elesim-pilot.service`를
사용한다. 아래 Python 명령은 Pilot 서비스를 사용하지 않을 때의 수동 실행이다.

```bash
source /opt/ros/humble/setup.bash
source ~/elesim/ros/install/setup.bash
~/elesim/roles/pilot/venv/bin/python -u -m elesim_pilot.main \
  --config ~/elesim/roles/pilot/config/installed.yaml \
  --runtime-config ~/elesim/roles/pilot/config/runtime.installed.yaml
```

**3. Ubuntu PC: 데스크톱 터미널에서 UI 실행**

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate elesim-ui
cd ~/elesim
source local/ubuntu-ui/ros/install/setup.bash
python -m elesim_ui.main --config local/ubuntu-ui/ui-runtime.yaml
```

UI에서 실물 대상 `robot-go2`를 선택한다. Ubuntu에도 이 실행 순서와 Wi-Fi
연결 명령을 `~/elesim/local/ubuntu-ui/README.md`에 저장했다.

**4. 선택: Mac에서 Sim 화면도 사용할 때**

Mac도 `H3C_D41BD2`와 같은 LAN에 연결하고 실행한다. Mac UI는 종료한 상태로 둔다.

```bash
cd ~/Workspace/elesim
conda activate elesim-ros
python -m elesim_sim.main \
  --config local/jetson-go2/sim-config.yaml \
  --runtime-config local/jetson-go2/sim-runtime.yaml
```

현재 AP에서 자동 발견 DDS 왕복은 확인했지만 H3C AP 자체는 아직 미검증이다.
연결되지 않으면 AP의 단말 간 통신 및 멀티캐스트 허용 여부를 확인한다.

## 이전 배포·검증 기록

아래 실행 상태·PID·IP는 각 작업 당시의 기록이다. 현재 실행은 위 순서를 따른다.

AP 이동 준비 (2026-10-21): Ubuntu UI·Jetson Robot/Pilot·Mac Sim 설정은
멀티캐스트 자동 발견으로 변경했다. `H3C_D41BD2`에 연결한 뒤 앱을 재시작하면
새 DHCP IP를 `static_peers`에 다시 적지 않아도 된다. Wi-Fi 연결과 앱 실행은
변경하지 않았다. 아래 IP는 기존 `IDIM_Robot_5G`에서의 설치 당시 주소다.
Mac Sim도 사용하려면 같은 LAN에 연결한다. H3C AP 자체의 단말 간 통신과
멀티캐스트 허용 여부는 해당 AP에서 확인해야 한다. 이전 고정 peer 설정 백업은
Jetson `~/elesim/updates/20261021-h3c-multicast-194410/`, Ubuntu
`~/elesim/local/ubuntu-ui/before-multicast-20261021-194409/`, Mac
`local/jetson-go2/before-multicast-20261021-194410/`다.

Current network update (2026-10-21): Ubuntu UI is installed at
`idim@192.168.0.62:2222`, checkout `~/elesim`, Conda environment `elesim-ui`.
The current Jetson and Mac profiles use direct LAN, without Tailscale:
Jetson `192.168.0.112` / `wlP1p1s0`, Mac Sim `192.168.0.29` / `en0`, Ubuntu UI
`192.168.0.62` / `wlp0s20f3`. Domain 68 and system ID are unchanged.
Jetson Robot/Pilot and Mac Sim need restarting to activate the new files; no
process was restarted during installation. Close the Mac UI when moving to
Ubuntu. See [Ubuntu UI setup and launch sequence](../ubuntu-ui/README.md).
The prior VPN configs are backed up under
`~/elesim/updates/20261021-ui-ubuntu-lan-193507/` on Jetson and
`local/jetson-go2/before-ubuntu-lan-20261021-193508/` on Mac.

Updated 2026-10-21. The corrected target-grasp Pilot wheel and 1005-sphere model
are installed in the Jetson role prefix. Target XYZ is the 20 mm object's center;
the planned open-claw grasp center now reaches it instead of stopping outside.
Both claws remain collision checked. Restart Pilot and the Mac UI to load this
change, then make a new plan. The running Pilot was left running during deployment;
no Robot/service restart or physical movement was requested. Robot is unchanged
by this update. Training continues independently on GPU 2 of the Linux server.
The existing Jetson Git checkout and the Mac simulation-only profiles are preserved.

Latest Jetson deployment (2026-10-20): the Robot and Pilot wheels now include
ZED Mini left/right image publication, the Pilot right-eye relay, and the
180-degree image/depth correction for the inverted mount. Robot config points
to `/elesim/robot_go2/rgbd/right` with `rotate_180: true`; Pilot config brokers
the right eye at `/elesim/pilot_main/rgbd/right`. Both prior packages and both
configs are backed up under
`/home/idim/elesim/updates/20261020-zed-stereo/`. Robot and Unitree bridge were
left running, so the existing Robot process (PID 2519713 at deployment) still
has the old code loaded. Pilot remains stopped. No service was restarted; the
installed changes take effect when you restart Robot and launch Pilot.

GO2's physical link is `eno1`, expected Jetson address `192.168.123.99/24`,
device `192.168.123.161`. GO2 telemetry was received on DDS domain **0**; the
old domain **1** received no telemetry. At the latest check `eno1` had no
carrier and no IP address, so that prior link configuration is not currently
active. The previous EleSim profile used `tailscale0`; current files use the
LAN interfaces listed above, domain **68**, system
`elesim-arm-20261016`. Automatic stand and gait changes are disabled.

Robot camera backend is `zed`, using ZED SDK 5.4.1 and `pyzed.sl`; the SDK and
RGB-D publisher run in a Robot-owned child process. The ZED Mini now enumerates
at USB 3 SuperSpeed, serial 11872518, and the SDK reports it AVAILABLE. A direct
Jetson probe successfully opened it and captured 640x480 color and depth frames.
An earlier UI-stream failure happened while Robot was down after it found two
active `sim-default` boot IDs. At the latest deployment check Robot and the
Unitree bridge were active, with Pilot stopped. The new stereo stream therefore
still needs Pilot launched and Robot restarted to load the stereo wheel; this is
separate from the successful direct ZED capture.
GO2 LowState joint angles now pass through Robot DDS telemetry as `go2.leg_q`,
which the existing Sim mirror applies to the twelve Genesis leg joints. This
update is already installed; the latest Robot restart loaded it.
The Unitree IPC heartbeat now runs independently of synchronous arm hardware
commands. This is also installed. The current stereo deployment did not restart
Robot or send Torque or movement commands.

## Jetson: activate Robot and bridge

Roll polarity rollback (2026-10-21): after the operator observed that -1 made
the physical and displayed Roll directions opposite, the installed Robot config
was restored to `arm.motor_direction: [1, 1, 1, -1]`. Wrap stays at
`roll_sign: 1.0`. The repository default alone does not establish this arm's
physical calibration. Stop Pilot before restarting Robot, then relaunch Pilot
with the command below and discard old paths before planning again. No process
was restarted during rollback. Backup and deployment record:
`~/elesim/updates/20261021-roll-motor-revert-192321/`.

Robot's installed `runtime.device` uses the U2D2 serial identity
`/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAAMNB6-if00-port0`.
It currently resolves to `/dev/ttyUSB1`; reconnecting the same U2D2 no longer
requires changing the config when its ttyUSB number changes. The 2026-10-21
config update was validated without opening the motor port or restarting Robot.

```bash
ssh -p 2227 idim@147.46.175.23
sudo systemctl restart elesim-robot.service
sudo journalctl -u elesim-robot.service -u elesim-unitree-bridge.service -f
```

This uses the installed `/home/idim/elesim` role environments and configs.
Robot pulls in the bridge and waits for its `Type=notify` readiness after
the IPC socket is listening. Before restarting, restore physical carrier on
`eno1` and make sure only one `sim-default` is running in this DDS system. The
bridge currently cannot start without `eno1`, and the last Robot run exited on
duplicate Sim boot IDs. Deployment did not restart either service.
Do not also start a separate Robot Python process while its service is running.

## Jetson: Pilot in another terminal

Wrap Roll update (2026-10-21): the installed Pilot default and adjacent
`wrap_grasp.yaml` now use `roll_sign: 1.0`, preserving the current policy's
Roll sense for observations, waypoints and lift. Backup, wheel and deployment
record are in `~/elesim/updates/20261021-wrap-roll/`. The running Pilot was
left unchanged; stop it with Ctrl+C and use the command below to activate.
Robot does not need restarting for this change.

The latest Pilot wheel and pre-install package/model backups are under
`/home/idim/elesim/updates/20261021-target-claw-center/`. The installed model is
`~/elesim/roles/pilot/payload/data/models/planning/zed-mini` (1005 spheres).
Wheel SHA-256: `f6cdb7a85b51b060ee9c353fbaca17a2593096cc1c3a6ea933358f0affb6a1b7`.
Stop the existing Pilot in its terminal with Ctrl+C before running the command
below. This activates the object-center goal and the finer claw collision model.
Restart the Mac UI for the planned claw-center label. Robot does not need a restart.

```bash
ssh -p 2227 idim@147.46.175.23
source /opt/ros/humble/setup.bash
source ~/elesim/ros/install/setup.bash
~/elesim/roles/pilot/venv/bin/python -u -m elesim_pilot.main \
  --config ~/elesim/roles/pilot/config/installed.yaml \
  --runtime-config ~/elesim/roles/pilot/config/runtime.installed.yaml
```

## Mac: optional real-robot mirror Sim

```bash
cd ~/Workspace/elesim
conda activate elesim-ros
python -m elesim_sim.main \
  --config local/jetson-go2/sim-config.yaml \
  --runtime-config local/jetson-go2/sim-runtime.yaml
```

This follows Robot's measured GO2 and arm state; GO2 locomotion is in mirror
mode. The current profile uses Mac LAN interface `en0` (192.168.0.29).
Keep one `sim-default` process per DDS system/domain.

## Mac: UI in another terminal

```bash
cd ~/Workspace/elesim
conda activate elesim-ros
python -m elesim_ui.main --config local/jetson-go2/ui-runtime.yaml
```

This UI enables hardware/GO2 controls in the Jetson system. The separate
`local/mac-sim` domain 69 remains simulation-only.
With `show_zed_preview: true`, a separate **Camera Views** window opens on
startup with **ZED View** selected. Choose `robot: robot-go2` in the UI Target
selector so Pilot streams the physical camera; if another RGB-D source is
selected, the view identifies that source instead. Use **Sim Cameras** in the
camera window to switch back to Genesis Observer and HAND-EYE streams. ZED
View shows the left and right stereo images side by side. The Robot publishes
both eyes upright by rotating the mounted-upside-down camera images 180°; the
registered left-eye depth and its intrinsics receive the same correction.

After updating UI source, restart the UI from the repository root with the
command above. In the expanded **Hardware** panel, a non-hardware safety fault
shows **Recheck hardware and clear safety fault**. It succeeds only when Robot
confirms Torque OFF, healthy telemetry and current back within its limit. It
does not re-enable torque; use the separate Torque control after the fault is
cleared. Restart the Jetson Pilot process with the command above before using
the new button. For the current `motor current limit exceeded: ID 1` latch, the
Robot service already has the necessary clear-check path.
The cuRobo panel displays the measured per-axis residual from the planned final
joint target when Pilot has restarted with the goal-settle update.

## Checks and recovery files

- Unitree IPC tests including startup readiness: 22 passed.
- Camera/lifecycle/config tests on Mac: 42 passed; camera/lifecycle/IPC on
  Jetson: 32 passed. Actual ZED SDK probe kept the parent responsive, with a
  maximum tick gap of approximately 21 ms, and captured 640x480 color/depth.
  The latest stereo/Pilot relay path awaits service activation and UI check.
- Read-only Mac UI to Jetson Pilot snapshot confirmed DDS and Pilot online.
- Robot DDS telemetry now includes the 12 LowState joint angles; verify the
  Sim `[sim_mirror] ... legs=12 joints` diagnostic after restarting Robot.
- Actual user-systemd test with a delayed fake backend: dependent service
  starts only after socket readiness; no ROS or hardware initialization.
- Native C++ motion/contact tests with the SDK test double: 5 passed.
- Actual ROS workspace sourcing, native ABI and Pilot generated model loading passed.
- Domain 0 read-only probe: 2400 Sport samples and approximately 4000 LowState
  samples in 8 seconds. Both reliable and best-effort subscriptions received data.
- No arm motor port was opened, no service was restarted, and no GO2 movement,
  standing or gait command was sent during preparation.
- Jetson backup and update evidence:
  `/home/idim/elesim/updates/20261020-go2/`.
  `before-install.tar.gz` preserves the original role Python packages, configs,
  console scripts and native library. The subsequent startup readiness fix
  is backed up under `readiness/` in the same update directory.
  `zed/before-zed.tar.gz` preserves the pre-ZED Robot package and config.

For a future installer run on this Jetson, specify
`ELESIM_UNITREE_INTERFACE=eno1`, `ELESIM_UNITREE_DOMAIN_ID=0` and
`ELESIM_UNITREE_ROS2_WS=/home/idim/ros2_ws`; the generic defaults are eth0/domain 1.
