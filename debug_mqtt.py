#!/usr/bin/env python3
"""
MQTT 연결 디버깅 스크립트
연결 문제를 진단합니다.
"""

import socket
import sys
import time

import paho.mqtt.client as mqtt


def test_network_connection(host, port):
    """네트워크 연결 테스트"""
    print(f"1. 네트워크 연결 테스트: {host}:{port}")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex((host, port))
        sock.close()

        if result == 0:
            print(f"   ✓ 포트 {port}가 열려있습니다.")
            return True
        else:
            print(f"   ✗ 포트 {port}에 연결할 수 없습니다. (에러 코드: {result})")
            return False
    except Exception as e:
        print(f"   ✗ 연결 오류: {e}")
        return False


def test_mqtt_connection(host, port, username=None, password=None):
    """MQTT 연결 테스트"""
    print(f"\n2. MQTT 브로커 연결 테스트: {host}:{port}")

    connected = False
    error_code = None

    def on_connect(client, userdata, flags, rc):
        nonlocal connected, error_code
        error_code = rc
        if rc == 0:
            connected = True
            print(f"   ✓ MQTT 연결 성공!")
        else:
            error_messages = {
                1: "잘못된 프로토콜 버전",
                2: "잘못된 클라이언트 식별자",
                3: "서버를 사용할 수 없음",
                4: "잘못된 사용자명 또는 비밀번호",
                5: "인증되지 않음",
            }
            msg = error_messages.get(rc, f"알 수 없는 오류 (코드: {rc})")
            print(f"   ✗ MQTT 연결 실패: {msg}")

    def on_disconnect(client, userdata, rc):
        if rc != 0:
            print(f"   ⚠ 예기치 않은 연결 끊김 (코드: {rc})")

    try:
        # 여러 프로토콜 버전 시도
        for protocol_version in [mqtt.MQTTv311, mqtt.MQTTv5]:
            protocol_name = (
                "MQTTv3.1.1" if protocol_version == mqtt.MQTTv311 else "MQTTv5"
            )
            print(f"   {protocol_name}로 시도 중...")

            client = mqtt.Client(
                client_id=f"test-client-{int(time.time())}", protocol=protocol_version
            )

            if username and password:
                client.username_pw_set(username, password)

            client.on_connect = on_connect
            client.on_disconnect = on_disconnect

            try:
                client.connect(host, port, keepalive=10)
                client.loop_start()

                # 연결 대기
                timeout = 5
                elapsed = 0
                while not connected and elapsed < timeout:
                    time.sleep(0.5)
                    elapsed += 0.5

                client.loop_stop()
                client.disconnect()

                if connected:
                    return True

            except Exception as e:
                print(f"   ✗ {protocol_name} 연결 오류: {e}")
                continue

        print(f"   ✗ 모든 프로토콜 버전에서 연결 실패")
        if error_code:
            print(f"   마지막 에러 코드: {error_code}")
        return False

    except Exception as e:
        print(f"   ✗ 테스트 오류: {e}")
        import traceback

        traceback.print_exc()
        return False


def test_mqtt_pub_sub(host, port):
    """MQTT 발행/구독 테스트"""
    print(f"\n3. MQTT 발행/구독 테스트")

    received = False

    def on_message(client, userdata, msg):
        nonlocal received
        received = True
        print(f"   ✓ 메시지 수신: {msg.payload.decode()}")

    try:
        # 구독자
        sub_client = mqtt.Client(client_id="test-subscriber", protocol=mqtt.MQTTv311)
        sub_client.on_message = on_message

        def on_connect_sub(client, userdata, flags, rc):
            if rc == 0:
                client.subscribe("test/debug")
                print("   구독 시작: test/debug")

        sub_client.on_connect = on_connect_sub
        sub_client.connect(host, port)
        sub_client.loop_start()

        time.sleep(1)

        # 발행자
        pub_client = mqtt.Client(client_id="test-publisher", protocol=mqtt.MQTTv311)
        pub_client.connect(host, port)
        pub_client.loop_start()

        time.sleep(0.5)
        pub_client.publish("test/debug", "Hello from debug script")
        print("   메시지 발행: test/debug")

        time.sleep(2)

        pub_client.loop_stop()
        pub_client.disconnect()
        sub_client.loop_stop()
        sub_client.disconnect()

        if received:
            print("   ✓ 발행/구독 테스트 성공")
            return True
        else:
            print("   ✗ 메시지를 받지 못했습니다.")
            return False

    except Exception as e:
        print(f"   ✗ 테스트 오류: {e}")
        import traceback

        traceback.print_exc()
        return False


def main():
    import argparse

    parser = argparse.ArgumentParser(description="MQTT 연결 디버깅")
    parser.add_argument(
        "--host", type=str, default="172.30.1.54", help="MQTT 브로커 주소"
    )
    parser.add_argument("--port", type=int, default=1883, help="MQTT 브로커 포트")
    parser.add_argument("--username", type=str, default=None, help="MQTT 사용자명")
    parser.add_argument("--password", type=str, default=None, help="MQTT 비밀번호")

    args = parser.parse_args()

    print("=" * 50)
    print("MQTT 연결 디버깅")
    print("=" * 50)
    print(f"브로커: {args.host}:{args.port}")
    if args.username:
        print(f"인증: {args.username}")
    print("=" * 50)

    # 테스트 실행
    network_ok = test_network_connection(args.host, args.port)

    if not network_ok:
        print("\n⚠ 네트워크 연결이 실패했습니다. 다음을 확인하세요:")
        print("  - 라즈베리파이 IP 주소가 올바른지")
        print("  - 같은 네트워크에 연결되어 있는지")
        print("  - 방화벽이 포트를 차단하지 않는지")
        return

    mqtt_ok = test_mqtt_connection(args.host, args.port, args.username, args.password)

    if mqtt_ok:
        test_mqtt_pub_sub(args.host, args.port)

    print("\n" + "=" * 50)
    if network_ok and mqtt_ok:
        print("✓ 기본 연결은 정상입니다.")
        print("영상 스트리밍 클라이언트를 다시 시도해보세요.")
    else:
        print("✗ 연결 문제가 발견되었습니다.")
        print("위의 오류 메시지를 확인하고 해결하세요.")
    print("=" * 50)


if __name__ == "__main__":
    main()
if __name__ == "__main__":
    main()
