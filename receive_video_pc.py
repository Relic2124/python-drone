#!/usr/bin/env python3
"""
컴퓨터에서 라즈베리파이의 WebRTC 영상 스트림을 받는 클라이언트
AI 처리를 위해 OpenCV로 프레임을 받아옵니다.

사용법:
    python receive_video_pc.py --host <라즈베리파이_IP> --port 8080 --room drone-camera-001
"""

import argparse
import asyncio
import json

import av
import cv2
import numpy as np
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer, MediaRelay


class VideoStreamReceiver:
    def __init__(self, host, port, room):
        self.host = host
        self.port = port
        self.room = room
        self.ws_url = f"ws://{host}:{port}/ws/{room}"
        self.pc = None
        self.video_track = None

    async def setup_webrtc(self):
        """WebRTC 연결 설정"""
        self.pc = RTCPeerConnection()

        # 비디오 트랙 수신 설정
        @self.pc.on("track")
        def on_track(track):
            print(f"트랙 수신: {track.kind}")
            if track.kind == "video":
                self.video_track = track

        return self.pc

    async def connect_websocket(self):
        """WebSocket으로 시그널링 서버에 연결"""
        try:
            async with websockets.connect(self.ws_url) as websocket:
                print(f"WebSocket 연결됨: {self.ws_url}")

                # WebRTC 설정
                pc = await self.setup_webrtc()

                # Offer 생성
                offer = await pc.createOffer()
                await pc.setLocalDescription(offer)

                # Offer를 서버로 전송
                await websocket.send(json.dumps({"type": "offer", "sdp": offer.sdp}))

                # Answer 수신
                response = await websocket.recv()
                data = json.loads(response)

                if data["type"] == "answer":
                    await pc.setRemoteDescription(
                        RTCSessionDescription(sdp=data["sdp"], type="answer")
                    )
                    print("WebRTC 연결 완료")

                    # ICE 후보 수신
                    while True:
                        message = await websocket.recv()
                        data = json.loads(message)

                        if data["type"] == "ice-candidate":
                            await pc.addIceCandidate(data["candidate"])
                        elif data["type"] == "error":
                            print(f"오류: {data['message']}")
                            break

        except Exception as e:
            print(f"연결 오류: {e}")

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
        if not self.video_track:
            print("비디오 트랙이 없습니다.")
            return

        print("프레임 수신 시작...")

        while True:
            try:
                frame = await self.video_track.recv()

                # aiortc의 VideoFrame을 OpenCV 형식으로 변환
                img = frame.to_ndarray(format="bgr24")

                # AI 처리
                processed_img = self.process_frame(img)

                # 화면에 표시 (선택사항)
                cv2.imshow("Drone Camera", processed_img)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            except Exception as e:
                print(f"프레임 수신 오류: {e}")
                break

        cv2.destroyAllWindows()

    async def run(self):
        """메인 실행 함수"""
        # WebSocket 연결 및 WebRTC 설정
        await self.connect_websocket()

        # 프레임 수신 시작
        await self.receive_frames()


def main():
    parser = argparse.ArgumentParser(description="드론 카메라 영상 수신 클라이언트")
    parser.add_argument(
        "--host", type=str, default="192.168.1.100", help="라즈베리파이 IP 주소"
    )
    parser.add_argument("--port", type=int, default=8080, help="WebSocket 포트")
    parser.add_argument(
        "--room", type=str, default="drone-camera-001", help="방 이름 (UID)"
    )

    args = parser.parse_args()

    receiver = VideoStreamReceiver(args.host, args.port, args.room)

    try:
        asyncio.run(receiver.run())
    except KeyboardInterrupt:
        print("\n종료합니다.")


if __name__ == "__main__":
    main()
