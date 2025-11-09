# 드론 BLE 조종 프로그램

안드로이드 드론 조종 앱을 기반으로 한 파이썬 데스크톱 버전입니다.

## 기능

- BLE (Bluetooth Low Energy)를 통한 드론 연결
- 실시간 조종 데이터 전송
- 대화형 명령어 인터페이스

## 사용 방법

### 1. 가상환경 활성화

```bash
# uv를 사용한 경우
source python-drone/.venv/bin/activate  # Linux/Mac
# 또는
python-drone\.venv\Scripts\activate  # Windows
```

### 2. 프로그램 실행

```bash
python python-drone/main.py
```

### 3. 조종 명령어

프로그램 실행 후 다음 명령어를 사용할 수 있습니다:

- `r <값>` - Roll 설정 (1-200, 기본 100)
- `p <값>` - Pitch 설정 (1-200, 기본 100)
- `y <값>` - Yaw 설정 (1-200, 기본 100)
- `t <값>` - Throttle 설정 (0-255)
- `reset` - 조종값을 중앙으로 리셋
- `status` - 현재 상태 출력
- `quit` - 종료

### 예제

```
명령어 입력: r 120
명령어 입력: p 80
명령어 입력: t 50
명령어 입력: status
명령어 입력: reset
```

## 기술 사양

### BLE UUID

- **SPP_WRITE**: `0000fff1-0000-1000-8000-00805f9b34fb` (데이터 전송)
- **SPP_NOTIFY**: `0000fff2-0000-1000-8000-00805f9b34fb` (데이터 수신)
- **INIT_SETTING**: `0000ffc1-0000-1000-8000-00805f9b34fb` (초기화)

### 데이터 패킷 형식

```
[0xF0, 0xA1, Roll, Pitch, Yaw, Throttle, 0x01, Checksum]
```

- **헤더**: 0xF0, 0xA1
- **Roll**: 1-200 (100 = 중앙)
- **Pitch**: 1-200 (100 = 중앙)
- **Yaw**: 1-200 (100 = 중앙)
- **Throttle**: 0-255
- **플래그**: 0x01
- **Checksum**: dataPackage[1]~[6]의 합

## 요구사항

- Python 3.13+
- bleak >= 1.1.1

## 참고

이 프로그램은 안드로이드 앱의 `BluetoothLeService.java`와 `DeviceControlActivity.java`를 기반으로 작성되었습니다.

