# 코드 지도

EleSim의 canonical runtime role key와 source tree 이름은 같다. `controller`,
`simulator`, Router 같은 이름은 legacy state/topology를 읽거나 과거 cleanup을
확인할 때만 나타날 수 있다.

## 1. 배포 애플리케이션

| tree | package/entrypoint | 책임 |
| --- | --- | --- |
| `pilot/` | `elesim_pilot`, `elesim-pilot` | perception/workflow, target, motion intent |
| `sim/` | `elesim_sim`, `elesim-sim` | Genesis, virtual state, RGB-D, session, WebRTC |
| `ui/` | `elesim_ui`, `elesim-ui` | operator UI, DDS intent, WebRTC receive |
| `robot/` | `elesim_robot`, `elesim-robot` | physical I/O, deadman, local safety |

`robot/`의 `unitree_bridge_daemon.py`와 `unitree_ipc*.py`는 Robot-host-local
adapter다. 별도 deployable role이나 Router가 아니다.

## 2. 공유 경계

```text
packages/elesim_interfaces/       ROSIDL msg/service/action definitions
packages/protocol/                PeerEnvelope, discovery, lease/session, RGB-D
model/bundles/default/            self-contained runtime model bundle and builder input assets
model/builder/                    model generation tools
misc/tools/release/               release context build/verify
misc/system_tests/                cross-process acceptance probes
installer/package/                setup, state, topology, security, lifecycle
environment/containers/           generated image/build inputs
environment/development/          generated all-project dev inputs
```

Deployment tree끼리 서로 import하지 않는다. ROSIDL wire types와 protocol
transport primitive만 공유한다. 추가 typed ROS services/actions는 generated
artifact이지만 runtime-wired surface가 아니다.

## 3. 핵심 구현 지점

- `packages/protocol/src/elesim_protocol/dds_transport.py`: DDS peer node,
  descriptor/heartbeat, bounded source-boot queue, addressed carrier,
  target/session authority.
- `packages/protocol/src/elesim_protocol/rgbd.py`: typed latest-only RGB-D
  publisher/subscriber.
- `packages/protocol/src/elesim_protocol/contracts.py`: protocol-v6 registry.
- `sim/src/elesim_sim/media/`: bounded frame slots, WebRTC/AV/ICE dispatch,
  encoder selection.
- `sim/src/elesim_sim/turn.py`: managed/external TURN credential boundary.
- `robot/src/elesim_robot/go2/unitree_ipc*.py`: bounded UDS, peer credentials,
  replay fence, deadman.
- `robot/src/elesim_robot/go2/unitree_bridge_daemon.py`: private Unitree DDS
  owner and bridge-side stop.
- `installer/package/src/elesim_setup/`: state schema v9, bootstrap artifacts,
  generated Compose, GPU/display/network doctor, topology v4, security rollout,
  ownership/uninstall.
- `misc/system_tests/smoke_topology.py`: real-RMW multi-process smoke; NAT,
  GPU, physical safety proof가 아니다.

## 4. 산출물 경계

`misc/tools/release/build.py`는 `dist/releases/pilot`, `sim`, `ui`, `robot`와
별도 `infra`를 만든다. `infra`는 setup/bootstrap/connection-manager와
container inputs이며 runtime application이 아니다.

General 설치는 role별 image/context를 만들고 `elesim-runtime` project에
고정한다. Developer는 `elesim-runtime-dev`의 persistent `elesim-dev` 하나를
사용한다. 설치된 config/model/security/log는 source tree의 반대편인 prefix
ownership boundary에 있다.

## 5. 테스트 지도

```text
packages/elesim_interfaces      colcon/ROSIDL contract
pilot/tests, sim/tests, ui/tests, robot/tests  role unit/workflow
installer/package/tests         setup/topology/security/ownership
misc/system_tests               real DDS/RGB-D/WebRTC process checks
misc/tools/quality              required/extended gates
misc/tools/release              context isolation
```

호스트 Python에 scientific/ROS 의존성을 추가하지 말고 setup-generated
`elesim-dev`에서 canonical gate를 실행한다.
