#!/usr/bin/env python3
"""
컴퓨터에서 라즈베리파이의 WebRTC 영상 스트림을 받는 클라이언트 (MQTT 방식)
AI 처리를 위해 OpenCV로 프레임을 받아옵니다.

사용법:
    python receive_video_pc_mqtt.py --mqtt-host <MQTT_브로커_IP> --uid drone-camera-001
"""

import argparse
import asyncio
import json
import threading

import av
import cv2
import numpy as np
import paho.mqtt.client as mqtt
from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRelay


class VideoStreamReceiver:
    def __init__(self, mqtt_host, mqtt_port, uid, username=None, password=None):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.uid = uid
        self.username = username
        self.password = password

        # 고유한 클라이언트 ID 생성
        import uuid

        self.client_id = f"client-{uuid.uuid4().hex[:8]}"

        # MQTT 토픽 설정 (pi-webrtc의 토픽 구조에 맞춤)
        # 형식: {uid}/sdp/{client_id}/offer, {uid}/ice/{client_id}/offer
        self.sdp_topic = f"{uid}/sdp/{self.client_id}/offer"
        self.ice_topic = f"{uid}/ice/{self.client_id}/offer"

        self.mqtt_client = None
        self.pc = None
        self.video_track = None
        self.relay = MediaRelay()
        self.connected = False

    def setup_webrtc(self):
        """WebRTC 연결 설정 (더 이상 사용하지 않음 - run()에서 직접 설정)"""
        pass

    def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT 연결 콜백 (MQTT v5 호환)"""
        # MQTT v3.1.1과 v5 모두 지원
        # v3.1.1: flags는 int, rc는 int
        # v5: flags는 dict, rc는 reason code, properties는 dict

        # v5인 경우 flags가 dict
        if isinstance(flags, dict):
            reason_code = rc  # v5에서는 reason code
        else:
            reason_code = rc  # v3.1.1에서는 rc

        if reason_code == 0:
            print(f"✓ MQTT 브로커 연결 성공: {self.mqtt_host}:{self.mqtt_port}")
            print(f"클라이언트 ID: {self.client_id}")
            # Answer와 ICE를 받기 위한 토픽 구독
            # 라즈베리파이는 {uid}/sdp/{client_id} 형식으로 발행 (answer 접미사 없음)
            answer_sdp_topic = f"{self.uid}/sdp/{self.client_id}"
            answer_ice_topic = f"{self.uid}/ice/{self.client_id}"
            client.subscribe(answer_sdp_topic)
            client.subscribe(answer_ice_topic)
            print(f"토픽 구독 완료:")
            print(f"  - {answer_sdp_topic} (Answer 수신)")
            print(f"  - {answer_ice_topic} (ICE 후보 수신)")
            self.connected = True
        else:
            error_messages = {
                1: "잘못된 프로토콜 버전",
                2: "잘못된 클라이언트 식별자",
                3: "서버를 사용할 수 없음",
                4: "잘못된 사용자명 또는 비밀번호",
                5: "인증되지 않음",
            }
            error_msg = error_messages.get(
                reason_code, f"알 수 없는 오류 (코드: {reason_code})"
            )
            print(f"✗ MQTT 연결 실패: {error_msg}")
            self.connected = False

    def on_mqtt_message(self, client, userdata, msg):
        """MQTT 메시지 수신 콜백"""
        topic = msg.topic
        payload = msg.payload.decode("utf-8")

        print(f"📨 MQTT 메시지 수신: {topic}")

        try:
            data = json.loads(payload)

            if "sdp" in data:
                # SDP 메시지 처리
                sdp_type = data.get("type", "")
                sdp = data["sdp"]

                print(f"  SDP 타입: {sdp_type}")

                if sdp_type == "answer":
                    # Answer를 받으면 설정 (라즈베리파이에서 보낸 Answer)
                    print("  ✓ Answer 수신, WebRTC 연결 설정 중...")
                    asyncio.run_coroutine_threadsafe(self.handle_answer(sdp), self.loop)
                else:
                    print(f"  ⚠ 알 수 없는 SDP 타입: {sdp_type}")

            elif "candidate" in data or "sdpMid" in data:
                # ICE 후보 처리
                candidate = data.get("candidate", "")
                sdp_mid = data.get("sdpMid", "")
                sdp_mline_index = data.get("sdpMLineIndex", 0)

                if candidate:
                    print(f"  ✓ ICE 후보 수신: {sdp_mid}:{sdp_mline_index}")
                    asyncio.run_coroutine_threadsafe(
                        self.handle_ice_candidate(candidate, sdp_mid, sdp_mline_index),
                        self.loop,
                    )
                else:
                    print("  ⚠ 빈 ICE 후보")
            else:
                print(f"  ⚠ 알 수 없는 메시지 형식: {list(data.keys())}")

        except json.JSONDecodeError as e:
            print(f"  ✗ JSON 파싱 오류: {e}")
            print(f"  원본 메시지: {payload[:100]}...")
        except Exception as e:
            print(f"  ✗ 메시지 처리 오류: {e}")
            import traceback

            traceback.print_exc()

    async def handle_offer(self, offer_sdp):
        """Offer 처리 및 Answer 생성"""
        if not self.pc:
            self.setup_webrtc()

        try:
            offer = RTCSessionDescription(sdp=offer_sdp, type="offer")
            await self.pc.setRemoteDescription(offer)

            # Answer 생성
            answer = await self.pc.createAnswer()
            await self.pc.setLocalDescription(answer)

            # Answer를 MQTT로 전송 (라즈베리파이가 구독하는 토픽으로)
            # 실제로는 라즈베리파이가 Offer를 보내므로 이 부분은 사용되지 않을 수 있음
            answer_topic = f"{self.uid}/sdp/{self.client_id}/answer"
            answer_data = {"type": "answer", "sdp": answer.sdp}
            self.mqtt_client.publish(answer_topic, json.dumps(answer_data))
            print("Answer 전송 완료")

        except Exception as e:
            print(f"Offer 처리 오류: {e}")

    async def handle_answer(self, answer_sdp):
        """Answer 처리"""
        if not self.pc:
            print("  ✗ PeerConnection이 없습니다.")
            return

        try:
            # Answer 설정 전 상태 확인
            print(f"  Answer 설정 전 상태:")
            print(f"    ICE 수집: {self.pc.iceGatheringState}")
            print(f"    ICE 연결: {self.pc.iceConnectionState}")
            print(f"    시그널링: {self.pc.signalingState}")

            answer = RTCSessionDescription(sdp=answer_sdp, type="answer")
            await self.pc.setRemoteDescription(answer)
            print("  ✓ Answer 설정 완료")

            # Answer 설정 후 상태 확인
            print(f"  Answer 설정 후 상태:")
            print(f"    ICE 수집: {self.pc.iceGatheringState}")
            print(f"    ICE 연결: {self.pc.iceConnectionState}")
            print(f"    시그널링: {self.pc.signalingState}")
            print(f"    연결 상태: {self.pc.connectionState}")

            # Answer의 SDP에 ICE 후보가 포함되어 있는지 확인
            if "candidate" in answer_sdp.lower():
                print("  ℹ️ Answer SDP에 ICE 후보가 포함되어 있습니다.")
            else:
                print("  ⚠ Answer SDP에 ICE 후보가 없습니다.")

            # ICE 후보 수집이 시작되지 않았다면 재시작 시도
            if self.pc.iceGatheringState == "new":
                print("  ⚠ ICE 후보 수집이 시작되지 않았습니다.")
                print("  ICE 후보 수집을 재시작합니다...")
                # ICE 후보 수집을 트리거하기 위해 약간의 지연 후 상태 확인
                await asyncio.sleep(0.5)
                print(f"  ICE 수집 상태 (재확인): {self.pc.iceGatheringState}")

                # 만약 여전히 "new"라면, 로컬 설명을 다시 설정해보기
                if self.pc.iceGatheringState == "new" and self.pc.localDescription:
                    print("  ⚠ ICE 후보 수집 재시작 시도...")
                    # 로컬 설명을 다시 설정하여 ICE 후보 수집 트리거
                    try:
                        await self.pc.setLocalDescription(self.pc.localDescription)
                        await asyncio.sleep(0.5)
                        print(
                            f"  ICE 수집 상태 (재설정 후): {self.pc.iceGatheringState}"
                        )
                    except Exception as e:
                        print(f"  ⚠ 로컬 설명 재설정 실패: {e}")

        except Exception as e:
            print(f"  ✗ Answer 처리 오류: {e}")
            import traceback

            traceback.print_exc()

    def parse_candidate_string(self, candidate_str):
        """candidate 문자열을 파싱하여 필요한 값들을 추출"""
        # candidate 문자열 형식: "candidate:foundation component protocol priority ip port typ type ..."
        # 예: "candidate:4234997325 1 udp 2043278322 192.0.2.172 44323 typ host"

        if not candidate_str or not candidate_str.startswith("candidate:"):
            raise ValueError(f"잘못된 candidate 형식: {candidate_str}")

        # "candidate:" 접두사 제거
        candidate_str = candidate_str[10:]  # "candidate:" 길이

        parts = candidate_str.split()
        if len(parts) < 7:
            raise ValueError(f"candidate 파싱 실패: 충분한 정보가 없습니다")

        foundation = parts[0]
        component = int(parts[1])
        protocol = parts[2]
        priority = int(parts[3])
        ip = parts[4]
        port = int(parts[5])
        typ = parts[6]  # "typ"
        candidate_type = parts[7] if len(parts) > 7 else "host"

        return {
            "foundation": foundation,
            "component": component,
            "protocol": protocol,
            "priority": priority,
            "ip": ip,
            "port": port,
            "type": candidate_type,
        }

    async def handle_ice_candidate(self, candidate, sdp_mid, sdp_mline_index):
        """ICE 후보 처리"""
        if not self.pc:
            print("  ✗ PeerConnection이 없습니다.")
            return

        try:
            # candidate 문자열을 파싱
            parsed = self.parse_candidate_string(candidate)

            # aiortc의 RTCIceCandidate 생성
            # 형식: RTCIceCandidate(foundation, component, protocol, priority, ip, port, type, ...)
            ice_candidate = RTCIceCandidate(
                foundation=parsed["foundation"],
                component=parsed["component"],
                protocol=parsed["protocol"],
                priority=parsed["priority"],
                ip=parsed["ip"],
                port=parsed["port"],
                type=parsed["type"],
                sdpMid=sdp_mid,
                sdpMLineIndex=sdp_mline_index,
            )
            await self.pc.addIceCandidate(ice_candidate)
            print(f"  ✓ ICE 후보 추가 완료: {sdp_mid}:{sdp_mline_index}")
        except Exception as e:
            print(f"  ✗ ICE 후보 처리 오류: {e}")
            print(f"    candidate: {candidate[:100] if candidate else 'None'}...")
            import traceback

            traceback.print_exc()

    def connect_mqtt(self):
        """MQTT 연결"""
        # 고유한 클라이언트 ID로 MQTT 클라이언트 생성
        import uuid

        mqtt_client_id = f"webrtc-client-{uuid.uuid4().hex[:8]}"

        # MQTT v3.1.1 사용 (pi-webrtc는 v5지만, paho-mqtt v5 호환성 문제로 v3.1.1 사용)
        # MQTT 브로커는 v3.1.1과 v5를 모두 지원하므로 v3.1.1 사용
        self.mqtt_client = mqtt.Client(client_id=mqtt_client_id, protocol=mqtt.MQTTv311)
        print("MQTT v3.1.1 사용")

        if self.username and self.password:
            self.mqtt_client.username_pw_set(self.username, self.password)

        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        # 연결 옵션 설정
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        try:
            print(f"MQTT 브로커 연결 시도: {self.mqtt_host}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
            print("MQTT 연결 시도 중...")

            # 연결 확인을 위해 잠시 대기
            import time

            time.sleep(1)

            if not self.mqtt_client.is_connected():
                print("MQTT 연결 실패: 브로커에 연결할 수 없습니다.")
                print("확인 사항:")
                print(
                    f"  1. MQTT 브로커가 실행 중인지 확인: {self.mqtt_host}:{self.mqtt_port}"
                )
                print("  2. 방화벽 설정 확인")
                print("  3. 네트워크 연결 확인")
                return False

        except Exception as e:
            print(f"MQTT 연결 오류: {e}")
            import traceback

            traceback.print_exc()
            return False

        return True

    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT 연결 끊김 콜백"""
        if rc != 0:
            print(f"MQTT 연결이 예기치 않게 끊어졌습니다. (코드: {rc})")
        else:
            print("MQTT 연결이 정상적으로 종료되었습니다.")

    def process_frame(self, frame):
        """
        AI 처리를 위한 프레임 처리 함수
        여기에 YOLO, 객체 인식 등의 AI 모델을 추가하세요
        """
        # 예시: 그레이스케일 변환
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 여기에 AI 처리 코드 추가
        # 예: YOLO 객체 인식, 경로 계획 등

        return frame

    async def receive_frames(self):
        """비디오 프레임 수신 및 처리"""
        print("프레임 수신 대기 중...")
        print("  - Answer 수신 대기 중...")
        print("  - ICE 후보 교환 대기 중...")
        print("  - 비디오 트랙 수신 대기 중...")

        # 비디오 트랙이 준비될 때까지 대기
        timeout = 30
        elapsed = 0
        check_interval = 1.0

        while not self.video_track and elapsed < timeout:
            await asyncio.sleep(check_interval)
            elapsed += check_interval

            # 상태 출력
            if int(elapsed) % 5 == 0:
                if self.pc:
                    conn_state = self.pc.connectionState
                    ice_state = self.pc.iceConnectionState
                    sig_state = self.pc.signalingState
                    print(f"  대기 중... ({int(elapsed)}초)")
                    print(
                        f"    연결: {conn_state}, ICE: {ice_state}, 시그널링: {sig_state}"
                    )
                else:
                    print(f"  대기 중... ({int(elapsed)}초) - PeerConnection 없음")

        if not self.video_track:
            print("✗ 비디오 트랙을 받지 못했습니다.")
            if self.pc:
                print(f"  WebRTC 상태: {self.pc.connectionState}")
                print(f"  ICE 연결 상태: {self.pc.iceConnectionState}")
                print(f"  시그널링 상태: {self.pc.signalingState}")
            print("\n확인 사항:")
            print("  1. 라즈베리파이에서 카메라가 정상 작동하는지 확인")
            print("  2. MQTT 메시지가 정상적으로 교환되는지 확인")
            print("  3. 네트워크 연결 상태 확인")
            return

        print("프레임 수신 시작... (종료하려면 'q' 키를 누르세요)")

        try:
            while True:
                try:
                    frame = await self.video_track.recv()

                    # aiortc의 VideoFrame을 OpenCV 형식으로 변환
                    img = frame.to_ndarray(format="bgr24")

                    # AI 처리
                    processed_img = self.process_frame(img)

                    # 화면에 표시
                    cv2.imshow("Drone Camera (MQTT)", processed_img)

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                except Exception as e:
                    print(f"프레임 수신 오류: {e}")
                    break
        finally:
            cv2.destroyAllWindows()
            if self.pc:
                await self.pc.close()
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()

    async def run(self):
        """메인 실행 함수"""
        self.loop = asyncio.get_event_loop()

        # MQTT 연결
        if not self.connect_mqtt():
            print("MQTT 연결에 실패했습니다.")
            return

        # WebRTC 설정 (ICE 서버 포함)
        from aiortc import RTCConfiguration, RTCIceServer

        # STUN 서버 설정 (NAT 통과를 위해 필요)
        # 여러 STUN 서버를 사용하여 연결 성공률 향상
        ice_servers = [
            RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
            RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
            RTCIceServer(urls=["stun:stun2.l.google.com:19302"]),
        ]
        config = RTCConfiguration(iceServers=ice_servers)
        self.pc = RTCPeerConnection(configuration=config)
        print("WebRTC PeerConnection 생성 완료")
        print(f"  STUN 서버: {len(ice_servers)}개 설정됨")

        # 트랜시버 추가 (수신용 - ICE 후보 수집을 위해 필요)
        # 비디오 트랙을 수신하기 위해 비디오 트랜시버 추가
        self.pc.addTransceiver("video", direction="recvonly")
        print("비디오 수신용 트랜시버 추가 완료")

        # 트랙 수신 설정
        @self.pc.on("track")
        def on_track(track):
            print(f"🎥 트랙 수신: {track.kind} (ID: {track.id})")
            if track.kind == "video":
                self.video_track = self.relay.subscribe(track)
                print("  ✓ 비디오 트랙 준비 완료!")
            else:
                print(f"  ℹ️ 오디오 트랙 수신 (무시)")

        # 연결 상태 모니터링
        @self.pc.on("connectionstatechange")
        def on_connection_state_change():
            state = self.pc.connectionState
            print(f"🔗 WebRTC 연결 상태 변경: {state}")
            if state == "connected":
                print("  ✓ WebRTC 연결 완료!")
            elif state == "failed":
                print("  ✗ WebRTC 연결 실패")
            elif state == "disconnected":
                print("  ⚠ WebRTC 연결 끊김")

        # ICE 후보 전송 설정
        @self.pc.on("icecandidate")
        def on_ice_candidate(candidate):
            if candidate:
                ice_data = {
                    "candidate": candidate.candidate,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                }
                ice_topic = f"{self.uid}/ice/{self.client_id}/offer"
                try:
                    result = self.mqtt_client.publish(ice_topic, json.dumps(ice_data))
                    if result.rc == mqtt.MQTT_ERR_SUCCESS:
                        print(
                            f"📤 ICE 후보 전송: {candidate.sdpMid}:{candidate.sdpMLineIndex}"
                        )
                        print(f"    후보: {candidate.candidate[:50]}...")
                    else:
                        print(f"  ⚠ ICE 후보 전송 실패 (MQTT 오류 코드: {result.rc})")
                except Exception as e:
                    print(f"  ✗ ICE 후보 전송 오류: {e}")
            else:
                # None인 경우는 ICE 수집 완료를 의미
                print("  ✓ ICE 후보 수집 완료 (null candidate)")

        # ICE 수집 상태 모니터링
        @self.pc.on("icegatheringstatechange")
        def on_ice_gathering_state_change():
            state = self.pc.iceGatheringState
            print(f"🧊 ICE 수집 상태 변경: {state}")
            if state == "complete":
                print("  ✓ ICE 후보 수집 완료!")

        # ICE 연결 상태 모니터링
        @self.pc.on("iceconnectionstatechange")
        def on_ice_connection_state_change():
            state = self.pc.iceConnectionState
            print(f"🧊 ICE 연결 상태 변경: {state}")
            if state == "connected" or state == "completed":
                print("  ✓ ICE 연결 완료!")
            elif state == "failed":
                print("  ✗ ICE 연결 실패")
                print("  STUN/TURN 서버 설정을 확인하세요")

        # Offer 생성 및 전송
        try:
            offer = await self.pc.createOffer()

            # Offer SDP 확인
            print(f"Offer 생성 완료")
            print(f"  SDP 타입: {offer.type}")
            print(f"  SDP 길이: {len(offer.sdp)} bytes")
            print(f"  m= 라인 수: {offer.sdp.count('m=')}")

            await self.pc.setLocalDescription(offer)

            # ICE 후보 수집이 시작되었는지 확인
            print(f"  Offer 설정 후 상태:")
            print(f"    ICE 수집: {self.pc.iceGatheringState}")
            print(f"    ICE 연결: {self.pc.iceConnectionState}")
            print(f"    시그널링: {self.pc.signalingState}")
            print(f"    연결 상태: {self.pc.connectionState}")

            # ICE 후보 수집이 시작될 때까지 대기 (최대 2초)
            max_wait = 2.0
            wait_interval = 0.1
            waited = 0.0
            while self.pc.iceGatheringState == "new" and waited < max_wait:
                await asyncio.sleep(wait_interval)
                waited += wait_interval

            print(
                f"  대기 후 ICE 수집 상태: {self.pc.iceGatheringState} (대기 시간: {waited:.1f}초)"
            )

            # ICE 후보 수집이 시작되지 않았다면 경고
            if self.pc.iceGatheringState == "new":
                print("  ⚠ 경고: ICE 후보 수집이 시작되지 않았습니다!")
                print(
                    "  이는 네트워크 설정 문제이거나 aiortc 라이브러리 문제일 수 있습니다."
                )

            # SDP 정리 (빈 MID 제거 - BUNDLE 오류 해결)
            sdp_lines = offer.sdp.split("\n")
            cleaned_sdp = []
            valid_mids = set()

            # 먼저 모든 m= 섹션의 MID를 수집
            current_mid = None
            for line in sdp_lines:
                if line.startswith("m="):
                    # m= 섹션 시작
                    current_mid = None
                elif line.startswith("a=mid:"):
                    # MID 추출
                    current_mid = line.split(":", 1)[1].strip()
                    if current_mid:
                        valid_mids.add(current_mid)

            # SDP 재구성 (유효한 MID만 포함)
            for line in sdp_lines:
                if line.startswith("a=group:BUNDLE"):
                    # BUNDLE 그룹에서 유효한 MID만 포함
                    parts = line.split()
                    if len(parts) > 1:
                        valid_bundle_mids = []
                        for mid in parts[1:]:
                            # 따옴표 제거 및 검증
                            clean_mid = mid.strip("'\"")
                            if clean_mid and clean_mid in valid_mids:
                                valid_bundle_mids.append(clean_mid)

                        if valid_bundle_mids:
                            cleaned_sdp.append(
                                f"a=group:BUNDLE {' '.join(valid_bundle_mids)}"
                            )
                        # 유효한 MID가 없으면 BUNDLE 라인 제거
                    else:
                        # BUNDLE 라인에 MID가 없으면 제거
                        pass
                elif line.startswith("a=mid:"):
                    # MID 라인은 그대로 유지 (이미 valid_mids에 포함됨)
                    cleaned_sdp.append(line)
                else:
                    cleaned_sdp.append(line)

            cleaned_sdp_str = "\n".join(cleaned_sdp)

            # 디버깅: 정리된 SDP 확인
            if "a=group:BUNDLE" in cleaned_sdp_str:
                bundle_lines = [
                    l for l in cleaned_sdp_str.split("\n") if "a=group:BUNDLE" in l
                ]
                print(f"SDP BUNDLE 그룹: {bundle_lines}")

            # Offer를 MQTT로 전송
            offer_data = {"type": "offer", "sdp": cleaned_sdp_str}
            self.mqtt_client.publish(self.sdp_topic, json.dumps(offer_data))
            print(f"Offer 전송 완료: {self.sdp_topic}")

        except Exception as e:
            print(f"Offer 생성 오류: {e}")
            import traceback

            traceback.print_exc()

        # 프레임 수신 시작
        await self.receive_frames()


def main():
    parser = argparse.ArgumentParser(
        description="드론 카메라 영상 수신 클라이언트 (MQTT)"
    )
    parser.add_argument(
        "--mqtt-host", type=str, default="localhost", help="MQTT 브로커 주소"
    )
    parser.add_argument("--mqtt-port", type=int, default=1883, help="MQTT 브로커 포트")
    parser.add_argument(
        "--uid", type=str, default="drone-camera-001", help="라즈베리파이와 동일한 UID"
    )
    parser.add_argument(
        "--username", type=str, default=None, help="MQTT 사용자명 (선택사항)"
    )
    parser.add_argument(
        "--password", type=str, default=None, help="MQTT 비밀번호 (선택사항)"
    )

    args = parser.parse_args()

    receiver = VideoStreamReceiver(
        args.mqtt_host, args.mqtt_port, args.uid, args.username, args.password
    )

    try:
        asyncio.run(receiver.run())
    except KeyboardInterrupt:
        print("\n종료합니다.")


if __name__ == "__main__":
    main()
