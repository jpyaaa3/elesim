# EleSim 문서 지도

각 사실은 아래 한 문서만 소유한다. 중복 설명을 추가하지 말고 소유 문서를
수정한다.

| 문서 | 소유 범위 |
| --- | --- |
| [`architecture.md`](architecture.md) | role/process 경계, authority, safety, runtime 불변식 |
| [`setup.md`](setup.md) | 한 host/prefix의 bootstrap, install, update, up/down, uninstall |
| [`deployment.md`](deployment.md) | release, multi-host topology, Connection Manager, SROS2 rollout |
| [`configuration.md`](configuration.md) | source/installed config 필드와 ownership |
| [`dds_contracts.md`](dds_contracts.md) | protocol v6 message registry, QoS, RGB-D wire, fencing |
| [`status.md`](status.md) | 완료 범위, 미해결 항목, manual acceptance gate |
| [`research.md`](research.md) | repository-only experiments와 진단 절차 |

판정 순서:

1. 현재 완료 여부와 남은 gate: `status.md`
2. 허용된 구조인지: `architecture.md`
3. wire 변경인지: `dds_contracts.md`
4. 설치 또는 단일 host 작업인지: `setup.md`
5. release 또는 multi-host 작업인지: `deployment.md`
6. 필드 의미인지: `configuration.md`

충돌 시 코드의 validated constant와 contract test를 확인한 뒤 해당 소유
문서 하나만 갱신한다. 자동 테스트 결과를 실제 네트워크, SROS2 enforce,
GPU, TURN relay, Jetson 또는 물리 안전의 증거로 승격하지 않는다.
