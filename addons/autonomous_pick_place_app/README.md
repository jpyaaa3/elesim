# autonomous_pick_place_app

RealSense(또는 mock)로 물체를 검출하고, **카메라 optical frame** 기준 3D 위치를 계산합니다.  
**world 변환은 elesim `host.py`**가 hand-eye 설정으로 처리하고, `sim.py`는 마커를 표시합니다.

## 동작 요약

1. 검출 (HSV / ROI / mock / YOLO) → mask  
2. depth → `p_camera_object` (optical: x=right, y=down, z=look)  
3. (선택) `host.py`로 **`object_camera`** 전송 (`--publish-host`)  
4. elesim host: FK + hand-eye → world debug marker 생성  
5. elesim sim: world 마커 표시  

물체 orientation / grasp / IK / 로봇 제어는 하지 않습니다.

## 설치

```bash
cd autonomous_pick_place_app
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

| 용도 | 추가 설치 |
|------|-----------|
| RealSense | `pip install pyrealsense2` |
| YOLO | `pip install ultralytics` |
| sim 마커 | `pip install pyzmq` |

```bash
cp configs/detector.yolo.example.json configs/detector.json
# model 경로, target_label 등 수정
```

## 실행

**Mock**

```bash
python main.py --detector-config configs/detector.example.json --mode mock
```

**Camera + sim 마커** (host·sim 먼저 실행)

```bash
# 터미널 1–2: elesim — python host.py / python sim.py

python main.py \
  --detector-config configs/detector.json \
  --detector yolo \
  --target-label cup \
  --mode camera \
  --publish-host \
  --host-endpoint tcp://127.0.0.1:5555
```

| 옵션 | 설명 |
|------|------|
| `--host-endpoint` | host ZMQ 주소 (기본 `tcp://127.0.0.1:5555`) |
| `--once` | 첫 프레임에 타깃 없으면 종료 |
| `--stop-on-detect` | 첫 탐지 후 종료 (기본: 계속 실행) |
| `--no-show` | OpenCV 미리보기 끔 |

Hand-eye 설정은 **elesim** `config.ini` → `hand_eye_config` 만 수정합니다.

## ZMQ (perception → host)

```json
{
  "t": "target",
  "source": "perception",
  "object_camera": [x, y, z],
  "object_label": "cup"
}
```

## 패키지 구조

```
main.py
observation.py          # CameraObservation
perception/             # camera, detector, depth_pose, preview
elesim_bridge/
  host_client.py        # ZMQ publish only
configs/
  detector*.json
tests/
```
