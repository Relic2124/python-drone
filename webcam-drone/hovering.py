import cv2
import numpy as np

# --- 전역 변수 ---
pre_target_x, pre_target_y = -1, -1
target_x, target_y = -1, -1

# --- 마우스 콜백 ---
def draw_point(event, x, y, flags, param):
    global target_x, target_y, drawing, points

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        if target_x==target_y==-1: pos.append((x,y))
        target_x, target_y = x, y

# 파라미터(필요시 조절)
THRESH     = 30             # 움직임 민감도(낮을수록 민감)
MIN_AREA   = 10            # 너무 작은 움직임 무시(픽셀 면적)
KERNEL     = np.ones((5,5), np.uint8)  # 모폴로지 커널

pos = [] # 드론 위치


import asyncio
from calendar import prmonth
from typing import Optional

from bleak import BleakClient, BleakScanner 

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
        print("Notification 활성화 완료")

        # 미세 보정
        rollDiff = 0 # (왼쪽-/오른쪽+)
        pitchDiff = 0 # (뒤-/앞+)

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

        # --- 튜닝 파라미터 (이 값을 조절해서 드론 반응을 바꿉니다) ---
        Kp = 0.05          # 비례 상수: 클수록 반응이 빠르지만 너무 크면 드론이 휘청거림
        Kd = 0.4          # 미분 상수 (브레이크/진동 억제)

        # DEADBAND = 20     # 데드존: 목표 반경 20픽셀 안에 들어오면 움직이지 않음 (진동 방지)
        MAX_SPEED = 10    # 안전을 위해 조종값의 최대 변화량을 제한 (100 ± 10 범위 내에서만 움직임)

        # 이전 오차를 기억하기 위한 변수 (초기화 필요)
        prev_error_x = 0
        prev_error_y = 0
        
        def get_data():
            if target_x == target_y == -1: return
            nonlocal throttle, roll, pitch, prev_error_x, prev_error_y
            global pre_target_x, pre_target_y
            throttle = 110  # 주의: 배터리 100%일 때, throttle 120 이상 설정 시 드론이 천장에 닿음
            if not pos: return
            x, y = pos[-1] # 드론 위치

            # 1. 오차(Error) 계산: 목표점 - 현재위치
            error_x = target_x - x
            error_y = target_y - y

            # 2. 데드존 체크: 오차가 너무 작으면(이미 도착했으면) 0으로 처리해서 멈춤
            # if abs(error_x) < DEADBAND: error_x = 0
            # if abs(error_y) < DEADBAND: error_y = 0

            # ---------------------------------------------------------
            # [PD 제어 핵심] P(비례) + D(미분) 계산
            # D항: (현재오차 - 이전오차) = 변화량(속도)
            # ---------------------------------------------------------

            # [Roll 제어]
            # P항: error_x * Kp
            # D항: (error_x - prev_error_x) * Kd
            if pre_target_x == target_x: control_roll = int((error_x * Kp) + ((error_x - prev_error_x) * Kd))
            else: control_roll = int(error_x * Kp)
            
            # [Pitch 제어] (화면 좌표계 Y축 반대 주의)
            # 전진하려면 Pitch 증가, 화면 위쪽(y감소)이 전진. 따라서 부호 반전(-)
            if pre_target_y == target_y: control_pitch = int((-error_y * Kp) + ((-error_y - prev_error_y) * Kd))
            else: control_pitch = int(-error_y * Kp)

            # 3. 현재 오차를 '이전 오차'로 저장 (다음 루프를 위해)
            prev_error_x = error_x
            prev_error_y = -error_y # Pitch는 부호 반전된 값을 저장

            # 4. 최대 속도 제한 (안전 장치)
            control_roll = clamp(control_roll, -MAX_SPEED, MAX_SPEED)
            control_pitch = clamp(control_pitch, -MAX_SPEED, MAX_SPEED)

            # 5. 값 적용
            roll = rollMid + control_roll
            pitch = pitchMid + control_pitch

            pre_target_x, pre_target_y = target_x, target_y

            #print(x, y, ix, iy, '\n')

        def send_control_data():
            get_data()
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

        # 주기적 데이터 전송 태스크 (안드로이드 앱: 800ms마다)
        send_task: Optional[asyncio.Task] = None
        stop_sending = False

        async def periodic_send():
            """주기적으로 조종 데이터 전송"""
            while not stop_sending:
                try:
                    data = send_control_data()
                    status_line = f"Roll:{roll:3d} Pitch:{pitch:3d} Yaw:{yaw:3d} Throttle:{throttle:3d}"
                    # 이전 줄을 지우고 새 줄 출력 (\r 사용)
                    print(f"\r{status_line}", end="", flush=True)
                    await client.write_gatt_char(control_char, data, response=False)
                    await asyncio.sleep(0.02)  # 20ms마다 전송 (안드로이드보다 빠르게)
                except Exception as e:
                    print(f"\n전송 오류: {e}")
                    break

        # 주기적 전송 시작
        stop_sending = False
        send_task = asyncio.create_task(periodic_send())

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')   # 코덱
        out = None
        recording = False

        # 배경 제거 객체 생성
        # history: 배경 학습 히스토리 길이, varThreshold: 감도 (낮을수록 민감)
        fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)

        cv2.namedWindow('Paint')
        cv2.setMouseCallback('Paint', draw_point)
        # 웹캠 열기
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            raise RuntimeError("웹캠을 열 수 없습니다.")

        try:
            print("[키] q: 종료, c: 기준 프레임 리셋")
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                mask = fgbg.apply(frame)

                # 모폴로지로 노이즈 제거
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  KERNEL, iterations=1)
                mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, KERNEL, iterations=2)

                # 윤곽선 → 모든 움직임에 사각형 박스
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                xs = []
                ys = []
                for c in contours:
                    if cv2.contourArea(c) < MIN_AREA:   # 작은 흔들림 무시
                        continue
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
                    
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                        #cv2.putText(frame, f"({cx},{cy})", (cx+8, cy-8),
                        #            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
                        if not target_x==target_y==-1:
                            xs.append(cx)
                            ys.append(cy)
                cv2.circle(frame, (target_x, target_y), 6, (0, 0, 0), -1)
                if pos:
                    cv2.circle(frame, pos[-1], 6, (255, 0, 255), -1)
                    cv2.putText(frame, f"({pos[-1][0]},{pos[-1][1]})", (pos[-1][0]+8, pos[-1][1]-8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2, cv2.LINE_AA)
                if xs:
                    nx = sum(xs)//len(xs)
                    ny = sum(ys)//len(ys)
                    if abs(pos[-1][0]-nx)<50 and abs(pos[-1][1]-ny)<50:
                        pos.append((nx, ny))
                    if len(pos)>10: pos.pop(0)

                # 표시
                cv2.imshow("Paint", frame)
                # (선택) 마스크도 보고 싶으면 아래 주석 해제
                cv2.imshow("Mask", mask)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): break # 종료
                elif key == ord('r'): # 녹화 시작/정지
                    if not recording:
                        h, w = frame.shape[:2]
                        out = cv2.VideoWriter('./result/record.mp4', fourcc, 20.0, (w, h))
                        recording = True
                        print('녹화 시작')
                    else:
                        recording = False
                        out.release()
                        print('녹화 종료')

                if recording:
                    out.write(frame)  # 프레임 저장

                # 주기적으로 현재 상태 표시 (1초마다)
                await asyncio.sleep(0.01)  # CPU 사용량 줄이기

        except KeyboardInterrupt:
            print("\n종료 중...")
        finally:
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

            cap.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"에러 발생: {e}")


