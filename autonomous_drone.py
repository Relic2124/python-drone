"""
드론 자율주행 통합 프로그램
비디오 수신 + YOLO 객체 감지 + 드론 조종을 통합합니다.
"""

import argparse
import asyncio
import queue
from typing import Optional

from bleak import BleakClient, BleakScanner

from autonomous_navigation import AutonomousNavigation, ControlCommand
from receive_video_pc_mqtt import VideoStreamReceiver

# BLE UUID 정의 (main.py에서 가져옴)
UUID_FBL770_SPP_WRITE = "0000fff1-0000-1000-8000-00805f9b34fb"
UUID_FBL770_SPP_NOTIFY = "0000fff2-0000-1000-8000-00805f9b34fb"
UUID_FBL770_INIT_SETTING = "0000ffc1-0000-1000-8000-00805f9b34fb"
UUID_CLIENT_CHARACTERISTIC_CONFIG = "00002902-0000-1000-8000-00805f9b34fb"


class AutonomousDrone:
    """자율주행 드론 클래스 - 비디오 수신과 드론 조종을 통합"""

    def __init__(
        self,
        mqtt_host: str,
        mqtt_port: int,
        uid: str,
        drone_name: str = "Firmtech(1F505F)",
        yolo_model_path: Optional[str] = None,
    ):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.uid = uid
        self.drone_name = drone_name

        # 비디오 수신기 초기화
        self.video_receiver = VideoStreamReceiver(mqtt_host, mqtt_port, uid)

        # 자율주행 모듈 초기화
        self.autonomous_nav = AutonomousNavigation(
            yolo_model_path=yolo_model_path,
            confidence_threshold=0.5,
            frame_width=1280,
            frame_height=720,
        )

        # 비디오 수신기에 자율주행 모듈 연결
        self.video_receiver.autonomous_nav = self.autonomous_nav

        # 조종 명령 큐
        self.control_command_queue = queue.Queue(maxsize=10)
        self.video_receiver.control_command_queue = self.control_command_queue

        # 드론 BLE 관련
        self.ble_client: Optional[BleakClient] = None
        self.control_char = None
        self.stop_control = False

        # 현재 조종값
        self.current_roll = 100
        self.current_pitch = 100
        self.current_yaw = 100
        self.current_throttle = 0

    async def connect_drone(self) -> bool:
        """드론에 BLE로 연결"""
        print("드론 장치 스캔 중...")
        device = await BleakScanner.find_device_by_name(self.drone_name)

        if device is None:
            print("드론을 찾을 수 없습니다.")
            return False

        print(f"드론 발견: {device.name} - {device.address}")

        try:
            self.ble_client = BleakClient(device)
            await self.ble_client.connect()
            print("드론 연결 완료")

            # 서비스 및 특성 찾기
            services = self.ble_client.services
            services_list = list(services)
            gatt_characteristics = []

            for service in services_list:
                characteristics = list(service.characteristics)
                gatt_characteristics.append(characteristics)

            # 조종 특성 찾기 (main.py와 동일한 방식)
            if len(gatt_characteristics) > 2 and len(gatt_characteristics[2]) > 0:
                self.control_char = gatt_characteristics[2][0]
                print(f"조종 특성 찾음: {self.control_char.uuid}")
                return True
            else:
                print("조종 특성을 찾을 수 없습니다.")
                return False

        except Exception as e:
            print(f"드론 연결 오류: {e}")
            return False

    def send_control_data(
        self, roll: int, pitch: int, yaw: int, throttle: int
    ) -> bytearray:
        """조종 데이터 생성 (main.py와 동일)"""
        data_package = bytearray(8)
        data_package[0] = 0xF0
        data_package[1] = 0xA1
        data_package[2] = roll
        data_package[3] = pitch
        data_package[4] = yaw
        data_package[5] = throttle
        data_package[6] = 0x01

        # Checksum 계산
        checksum = sum(data_package[1:7]) & 0xFF
        data_package[7] = checksum

        return data_package

    async def control_loop(self):
        """조종 명령을 주기적으로 전송하는 루프"""
        if not self.ble_client or not self.control_char:
            return

        print("조종 루프 시작...")

        while not self.stop_control:
            try:
                # 큐에서 최신 명령 가져오기 (블로킹 없이)
                try:
                    command: ControlCommand = self.control_command_queue.get_nowait()
                    self.current_roll = command.roll
                    self.current_pitch = command.pitch
                    self.current_yaw = command.yaw
                    self.current_throttle = command.throttle
                except queue.Empty:
                    # 명령이 없으면 현재 값 유지
                    pass

                # 조종 데이터 전송
                data = self.send_control_data(
                    self.current_roll,
                    self.current_pitch,
                    self.current_yaw,
                    self.current_throttle,
                )

                await self.ble_client.write_gatt_char(
                    self.control_char, data, response=False
                )

                # 상태 출력 (선택적)
                # print(f"Roll:{self.current_roll:3d} Pitch:{self.current_pitch:3d} "
                #       f"Yaw:{self.current_yaw:3d} Throttle:{self.current_throttle:3d}")

                await asyncio.sleep(0.1)  # 100ms마다 전송

            except Exception as e:
                print(f"조종 전송 오류: {e}")
                await asyncio.sleep(0.1)

    async def run(self):
        """메인 실행 함수"""
        # 1. 드론 연결
        if not await self.connect_drone():
            print("드론 연결 실패. 종료합니다.")
            return

        try:
            # 2. 조종 루프 시작
            control_task = asyncio.create_task(self.control_loop())

            # 3. 비디오 수신 및 자율주행 시작
            print("\n자율주행 모드 시작!")
            print("=" * 50)
            print("조종 방법:")
            print("  - 자동: YOLO가 객체를 감지하여 자동으로 회피")
            print("  - 수동: 키보드 입력으로 조종 가능 (추후 구현)")
            print("  - 'q' 키: 종료")
            print("=" * 50 + "\n")

            # 비디오 수신 시작 (자율주행 로직 포함)
            await self.video_receiver.run()

        except KeyboardInterrupt:
            print("\n종료 중...")
        finally:
            # 정리
            self.stop_control = True
            if control_task:
                control_task.cancel()
                try:
                    await control_task
                except asyncio.CancelledError:
                    pass

            # 안전 착륙
            print("안전 착륙 중...")
            self.current_throttle = 0
            for _ in range(3):
                data = self.send_control_data(
                    self.current_roll,
                    self.current_pitch,
                    self.current_yaw,
                    self.current_throttle,
                )
                if self.ble_client and self.control_char:
                    await self.ble_client.write_gatt_char(
                        self.control_char, data, response=False
                    )
                await asyncio.sleep(0.1)

            # 연결 종료
            if self.ble_client:
                await self.ble_client.disconnect()
            print("드론 연결 종료")


def main():
    parser = argparse.ArgumentParser(description="드론 자율주행 프로그램")
    parser.add_argument(
        "--mqtt-host",
        type=str,
        default="localhost",
        help="MQTT 브로커 주소",
    )
    parser.add_argument("--mqtt-port", type=int, default=1883, help="MQTT 브로커 포트")
    parser.add_argument(
        "--uid",
        type=str,
        default="drone-camera-001",
        help="라즈베리파이와 동일한 UID",
    )
    parser.add_argument(
        "--drone-name",
        type=str,
        default="Firmtech(1F505F)",
        help="드론 BLE 장치 이름",
    )
    parser.add_argument(
        "--yolo-model",
        type=str,
        default=None,
        help="YOLO 모델 파일 경로 (기본값: yolov8n.pt)",
    )

    args = parser.parse_args()

    drone = AutonomousDrone(
        mqtt_host=args.mqtt_host,
        mqtt_port=args.mqtt_port,
        uid=args.uid,
        drone_name=args.drone_name,
        yolo_model_path=args.yolo_model,
    )

    try:
        asyncio.run(drone.run())
    except KeyboardInterrupt:
        print("\n종료합니다.")


if __name__ == "__main__":
    main()
    main()
