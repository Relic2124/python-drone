"""
드론 BLE 조종 프로그램
안드로이드 앱을 기반으로 한 파이썬 데스크톱 버전
"""

import asyncio
from calendar import prmonth
from typing import Optional

from bleak import BleakClient, BleakScanner

# BLE UUID 정의 (안드로이드 코드에서 추출)
UUID_FBL770_SPP_WRITE = "0000fff1-0000-1000-8000-00805f9b34fb"
UUID_FBL770_SPP_NOTIFY = "0000fff2-0000-1000-8000-00805f9b34fb"
UUID_FBL770_INIT_SETTING = "0000ffc1-0000-1000-8000-00805f9b34fb"
UUID_CLIENT_CHARACTERISTIC_CONFIG = "00002902-0000-1000-8000-00805f9b34fb"

speed = 10

# 미세 보정
rollDiff = 0 #(왼쪽-/오른쪽+)
pitchDiff = 0 #(뒤-/앞+)

async def main():
    # 1. 드론 장치 스캔
    print("드론 장치 스캔 중...")
    device = await BleakScanner.find_device_by_name("Firmtech(1F505F)")

    if device is None:
        print("드론을 찾을 수 없습니다.")
        return

    print(f"드론 발견: {device.name} - {device.address}")

    # 2. 드론에 연결
    async with BleakClient(device) as client:
        print("연결 중...")

        # 3. 서비스 발견 (자동으로 수행됨, services property로 접근)
        print("\n서비스 발견 중...")
        services = client.services

        # 안드로이드 앱과 동일한 방식으로 서비스/특성 리스트 구성
        # mGattCharacteristics = ArrayList<ArrayList<BluetoothGattCharacteristic>>
        gatt_characteristics = []

        # 서비스를 리스트로 변환 (len() 사용을 위해)
        services_list = list(services)
        print(f"\n총 {len(services_list)}개의 서비스 발견:")

        # 모든 서비스와 특성 탐색 및 인덱스 정보 출력
        for service_idx, service in enumerate(services_list):
            print(f"\n[서비스 인덱스 {service_idx}] UUID: {service.uuid}")
            characteristics = list(service.characteristics)
            gatt_characteristics.append(characteristics)

            for char_idx, char in enumerate(characteristics):
                # 특성의 속성 확인
                # Bleak의 properties는 CharacteristicProperties enum 또는 리스트일 수 있음
                props = char.properties
                props_str = ", ".join(props) if props else "N/A"
                print(f"  [특성 인덱스 {char_idx}] UUID: {char.uuid} [{props_str}]")

            # 서비스[2]의 특성[0]을 조종 특성으로 설정
        control_char = gatt_characteristics[2][0]

        print(f"   속성: {control_char.properties}")
        print(f"   UUID: {control_char.uuid}")

        # 5. Notification 활성화 (안드로이드 앱의 1초 후 활성화와 동일)
        print("\nNotification 활성화 중...")
        await asyncio.sleep(1.0)  # 1초 대기 (안드로이드 앱과 동일)

        def notification_handler(sender, data: bytearray):
            """Notification 수신 핸들러 (조용하게 처리)"""
            # 중요한 알림만 출력 (Init Data 등)
            # if len(data) >= 2 and data[0] == 0x55 and data[1] == 0x33:
            #     print("\n[알림] Init Setting Data 수신")
            # elif len(data) >= 2 and data[0] == 0x55 and data[1] == 0x03:
            # SPP READ는 조용하게 처리 (너무 자주 오므로)
            pass
            # 일반 notification은 출력하지 않음 (조종값 표시 방해 방지)
            # print(data)

        # await client.start_notify(control_char, notification_handler)
        print("Notification 활성화 완료")

        # 6. 드론 조종 구현
        print("\n" + "=" * 50)
        print("드론 조종 모드 시작")
        print("=" * 50)
        print("\n조종 방법:")
        print("  W/S: Pitch 증가/감소 (앞/뒤)")
        print("  A/D: Roll 감소/증가 (왼쪽/오른쪽)")
        print("  Q/E: Yaw 감소/증가 (왼쪽 회전/오른쪽 회전)")
        print("  Space: Throttle 증가")
        print("  X 또는 Backspace: Throttle 감소")
        print("  R: 리셋 (모든 값을 중앙으로)")
        print("  P: 현재 상태 출력")
        print("  Q 또는 Ctrl+C: 종료")
        print("\n조종값 범위:")
        print("  Roll/Pitch/Yaw: 1-200 (100 = 중앙)")
        print("  Throttle: 0-255")
        print("=" * 50 + "\n")

        mid = 100 # 중앙값
        rollMid = mid + rollDiff
        pitchMid =  mid + pitchDiff
        # 안드로이드 앱과 동일한 초기값
        roll = rollMid
        pitch = pitchMid
        yaw = mid
        throttle = 0

        # 조종값 변경 함수
        def clamp(value: int, min_val: int, max_val: int) -> int:
            """값을 범위 내로 제한"""
            return max(min_val, min(max_val, value))

        def send_control_data():
            """조종 데이터 전송 (안드로이드 앱의 sendData() 함수와 동일)"""
            data_package = bytearray(8)
            data_package[0] = 0xF0
            data_package[1] = 0xA1
            data_package[2] = roll
            data_package[3] = pitch
            data_package[4] = yaw
            data_package[5] = throttle
            data_package[6] = 0x01

            # Checksum 계산 (dataPackage[1]~[6]의 합)
            checksum = sum(data_package[1:7]) & 0xFF
            data_package[7] = checksum

            return data_package

        def print_status():
            """현재 조종 상태 출력"""
            print("\n현재 조종 상태:")
            print(f"  Roll:    {roll:3d} (1-200, 100=중앙)")
            print(f"  Pitch:   {pitch:3d} (1-200, 100=중앙)")
            print(f"  Yaw:     {yaw:3d} (1-200, 100=중앙)")
            print(f"  Throttle: {throttle:3d} (0-255)")

        # 주기적 데이터 전송 태스크 (안드로이드 앱: 800ms마다)
        send_task: Optional[asyncio.Task] = None
        stop_sending = False

        async def periodic_send():
            """주기적으로 조종 데이터 전송"""
            while not stop_sending:
                try:
                    data = send_control_data()
                    await client.write_gatt_char(control_char, data, response=False)
                    await asyncio.sleep(0.1)  # 100ms마다 전송 (안드로이드보다 빠르게)
                except Exception as e:
                    print(f"\n전송 오류: {e}")
                    break

        # 키보드 입력 처리 (Windows용)
        import msvcrt

        print_status()
        print("\n조종 시작! 키를 입력하세요...\n")

        # 주기적 전송 시작
        stop_sending = False
        send_task = asyncio.create_task(periodic_send())

        # 조종값을 한 줄로 표시하기 위한 변수
        last_status_line = ""


        is_resetting = False
        THROTTLE_DECAY_RATE = 1
        cnt = 0
        try:
            while True:
                # Windows에서 비동기 키 입력 확인
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode("utf-8").lower()

                    if key == "w": pitch = clamp(pitchMid + speed, 1, 200)
                    elif key == "s": pitch = clamp(pitchMid - speed, 1, 200)
                    else: pitch = pitchMid
                    if key == "a": roll = clamp(rollMid - speed, 1, 200)
                    elif key == "d": roll = clamp(rollMid + speed, 1, 200)
                    else: roll = rollMid
                    if key == "q": yaw = clamp(mid - speed, 1, 200)
                    elif key == "e": yaw = clamp(mid + speed, 1, 200)
                    else: yaw = mid

                    if key == " ":  # Space
                        throttle = clamp(throttle + 5, 0, 255)
                    elif (
                        key == "\x08" or key == "x"
                    ):  # Backspace 또는 X (Throttle 감소)
                        throttle = clamp(throttle - 5, 0, 255)
                    elif key == "r":
                        roll = rollMid
                        pitch = pitchMid
                        yaw = mid
                        is_resetting = True
                    elif key == "p":
                        print_status()
                        continue
                    elif key == "\x03" or key == "\x1b":  # Ctrl+C or ESC
                        print("\n종료 중...")
                        break
                    elif key not in 'wsadqe':
                        # 알 수 없는 키는 무시
                        continue

                    # 조종값 변경 시 한 줄로 표시 (이전 줄 덮어쓰기)
                    status_line = f"Roll:{roll:3d} Pitch:{pitch:3d} Yaw:{yaw:3d} Throttle:{throttle:3d}"
                    # 이전 줄을 지우고 새 줄 출력 (\r 사용)
                    print(f"\r{status_line}", end="", flush=True)
                    last_status_line = status_line

                if is_resetting:
                    if throttle > 0:
                        cnt += 1
                        if cnt&1: throttle = clamp(throttle - THROTTLE_DECAY_RATE, 0, 255)
                    else:
                        is_resetting = False
                        cnt = 0
                        print("리셋 완료")

                # 주기적으로 현재 상태 표시 (1초마다)
                await asyncio.sleep(0.01)  # CPU 사용량 줄이기

        except KeyboardInterrupt:
            print("\n종료 중...")
        finally:
            # 마지막 상태 줄 다음에 개행 추가
            if last_status_line:
                print()  # 개행 추가
            # 전송 중지
            stop_sending = True
            if send_task:
                send_task.cancel()
                try:
                    await send_task
                except asyncio.CancelledError:
                    pass

            # 마지막으로 throttle을 0으로 설정하여 안전하게 착륙
            throttle = 0
            for _ in range(3):  # 3번 전송하여 확실히
                data = send_control_data()
                await client.write_gatt_char(control_char, data, response=False)
                await asyncio.sleep(0.1)

            # Notification 비활성화
            await client.stop_notify(control_char)
            print("Notification 비활성화 완료")
            print("드론 연결 종료")


if __name__ == "__main__":
    asyncio.run(main())
