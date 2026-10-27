# EleSim 원클릭 시작

이 PC의 운영체제는 재부팅하지 않는다. 시작 스크립트가 Jetson의 Pilot·Robot·Unitree bridge systemd 서비스를 정해진 순서로 재시작하고, 이 PC의 Sim·UI를 새 터미널에서 실행한다.

## 처음 한 번

Ubuntu 데스크톱에 로그인한 뒤 EleSim 저장소에서 실행한다.

```bash
bash ~/elesim/workbench/tools/start-elesim.sh
```

실행 파일: [`start-elesim.sh`](../workbench/tools/start-elesim.sh)

처음에는 Jetson의 SSH 주소, 포트, 사용자, EleSim 설치 폴더를 묻는다. 확인된 기본값이 맞으면 Enter를 누른다. SSH host key 확인과 sudo 암호 입력은 직접 진행한다. 접속 정보는 암호 없이 `~/elesim/local/one-click/connection.conf`에 저장된다.

## 매번 시작

팔이 움직이지 않도록 받치고, 하드웨어 전원과 케이블을 준비한 뒤 Ubuntu 데스크톱에서 다음 한 줄을 실행한다.

```bash
bash ~/elesim/workbench/tools/start-elesim.sh
```

스크립트는 먼저 이 Ubuntu 계정에서 실행 중인 모든 EleSim Sim 프로세스와 현재 저장소의 UI를 종료한다. 이어 Jetson 파일·서비스를 확인하고, Jetson에서 Pilot을 멈춘 다음 DDS 네트워크 준비를 기다린다. 이어 Robot 서비스를 재시작한다. Robot 서비스의 systemd 의존성이 Unitree bridge를 함께 관리하며, 둘 다 active인지 확인한 뒤 Pilot을 시작한다. 마지막으로 Ubuntu에서 새 Sim·UI 터미널 두 개를 연다. Mac 등 다른 컴퓨터의 Sim 프로세스는 해당 컴퓨터에 SSH로 접속하도록 설정해야 종료할 수 있다. 기존 Jetson SSH 연결 정보는 그대로 사용한다.

UI에서 Target `robot-go2`와 telemetry를 확인한다. 기동 스크립트는 모터 Torque를 켜거나 Safety fault를 지우지 않는다. Torque enable과 실제 동작은 상태를 확인한 뒤 UI에서 별도로 조작한다. `Safety fault`가 보이거나 telemetry가 최신이 아니면 로봇을 움직이지 않는다.

## 기동이 끝나지 않을 때

각 프로그램의 터미널은 프로세스가 종료되어도 오류와 로그 경로를 보여준다. 실행 로그는 `~/elesim/local/one-click/runs/` 아래에 저장된다. Jetson 서비스의 최근 로그도 시작 스크립트 출력에 포함된다.

| 보이는 증상 | 확인할 원인과 조치 |
| --- | --- |
| SSH 연결 실패 또는 host key 확인 | 저장된 Jetson 주소·포트·사용자와 SSH 연결을 확인한다. 주소를 바꿨다면 `bash ~/elesim/workbench/tools/start-elesim.sh --configure`로 저장값을 갱신한다. |
| Jetson 설치 경로/파일 확인 실패 | Jetson에 복사본이 아닌 실제 설치 폴더가 있고 `bin/elesim-robot`, `roles/robot/config/installed.yaml`, Pilot 설정 및 systemd 파일이 있는지 확인한다. |
| sudo 암호 또는 서비스 권한 오류 | SSH 사용자로 로그인할 수 있고 해당 계정이 sudo 권한을 갖는지 확인한다. 암호는 스크립트에 저장하지 않는다. |
| 네트워크 준비 timeout | Jetson에서 Pilot runtime 설정이 지정한 DDS 인터페이스가 연결됐는지 확인한다. Robot을 재시작하기 전에 네트워크가 준비되지 않으면 스크립트가 중단된다. |
| bridge 또는 Robot이 active가 아님 | Jetson에서 `sudo journalctl -b -u elesim-unitree-bridge -u elesim-robot --no-pager -n 80`을 확인한다. Robot의 U2D2 USB 경로, 모터 전원, 배선, baudrate와 ID 설정을 점검한다. |
| Pilot이 active가 아님 | Jetson에서 `sudo journalctl -b -u elesim-pilot --no-pager -n 80`을 확인하고 DDS 설정과 Pilot 시작 오류를 점검한다. |
| Sim 또는 UI 터미널이 열리지 않음 | 이 PC에 그래픽 데스크톱 세션, Miniforge의 `elesim-sim`·`elesim-ui` 환경과 각 ROS overlay가 있는지 확인하고 `bash ~/elesim/workbench/tools/start-elesim.sh --check`를 실행한다. |
| UI의 Dynamixel ID 254에 status packet 없음 | 254는 GroupSyncRead 브로드캐스트 요청의 오류 표기이며 모터 ID가 아니다. 설정상 모터 ID는 1–5다. U2D2 연결, 실제 Dynamixel 전원, 통신선, 장치 경로·baudrate·각 모터 ID를 점검한다. Torque disable 응답도 없었다면 Torque OFF를 확인했다고 간주하지 말고 팔을 받친다. |
| Safety fault가 남음 | Robot은 연속 telemetry 실패 뒤 안전 정지 상태를 유지한다. 통신과 telemetry를 복구하고 UI에서 Torque OFF 및 정상 telemetry를 확인한 뒤 절차에 따라 fault를 해제한다. 스크립트는 fault를 지우지 않는다. |

Jetson의 자동 부팅 등록이 필요한 경우에만 최초 1회 Jetson에서 Pilot 서비스를 등록한다. 평상시 시작은 원클릭 스크립트가 담당한다.

```bash
sudo install -m 0644 ~/elesim/local/systemd/elesim-pilot.service /etc/systemd/system/elesim-pilot.service
sudo systemctl daemon-reload
sudo systemctl enable elesim-robot.service elesim-pilot.service
```

실제 설치 위치가 `~/elesim`이 아니면 Jetson에서 해당 실제 경로를 사용한다. 이 등록은 OS를 재부팅하지 않는다.
