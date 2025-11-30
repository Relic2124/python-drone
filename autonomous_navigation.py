"""
드론 자율주행 모듈
YOLO 객체 감지를 사용한 장애물 회피 및 경로 계획
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class Detection:
    """감지된 객체 정보"""

    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    center: Tuple[int, int]  # 중심점 (x, y)


@dataclass
class ControlCommand:
    """드론 조종 명령"""

    roll: int  # 1-200 (100=중앙, 왼쪽-, 오른쪽+)
    pitch: int  # 1-200 (100=중앙, 뒤-, 앞+)
    yaw: int  # 1-200 (100=중앙, 왼쪽-, 오른쪽+)
    throttle: int  # 0-255


class AutonomousNavigation:
    """
    자율주행 로직 클래스
    YOLO 객체 감지를 사용하여 장애물을 회피하고 경로를 계획합니다.
    """

    def __init__(
        self,
        yolo_model_path: Optional[str] = None,
        confidence_threshold: float = 0.5,
        obstacle_classes: Optional[List[str]] = None,
        frame_width: int = 1280,
        frame_height: int = 720,
    ):
        """
        Args:
            yolo_model_path: YOLO 모델 파일 경로 (None이면 기본 모델 사용)
            confidence_threshold: 객체 감지 신뢰도 임계값
            obstacle_classes: 장애물로 간주할 클래스 목록 (None이면 모든 객체)
            frame_width: 프레임 너비
            frame_height: 프레임 높이
        """
        self.confidence_threshold = confidence_threshold
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.frame_center_x = frame_width // 2
        self.frame_center_y = frame_height // 2

        # 장애물 클래스 (기본값: 사람, 자동차, 자전거 등)
        self.obstacle_classes = obstacle_classes or [
            "person",
            "car",
            "truck",
            "bus",
            "bicycle",
            "motorcycle",
            "dog",
            "cat",
        ]

        # YOLO 모델 초기화
        self.yolo_model = None
        self.use_ultralytics = False

        # YOLO 모델 로드 시도
        try:
            # ultralytics YOLOv8 시도
            from ultralytics import YOLO

            if yolo_model_path:
                self.yolo_model = YOLO(yolo_model_path)
            else:
                # 기본 YOLOv8n 모델 사용 (nano - 가장 빠름)
                self.yolo_model = YOLO("yolov8n.pt")
            self.use_ultralytics = True
            print("✓ Ultralytics YOLO 모델 로드 완료")
        except ImportError:
            # OpenCV DNN으로 대체
            print("⚠ Ultralytics를 사용할 수 없습니다. OpenCV DNN을 사용합니다.")
            self._load_opencv_dnn(yolo_model_path)

        # 조종 파라미터
        self.mid = 100  # 중앙값
        self.speed = 10  # 기본 속도
        self.max_roll = 30  # 최대 Roll 변화량
        self.max_pitch = 30  # 최대 Pitch 변화량
        self.max_yaw = 20  # 최대 Yaw 변화량

        # 안전 파라미터
        self.safe_distance_threshold = 150  # 픽셀 단위 안전 거리
        self.danger_zone_ratio = 0.3  # 화면 중앙 30% 영역을 위험 구역으로 간주

    def _load_opencv_dnn(self, model_path: Optional[str] = None):
        """OpenCV DNN을 사용하여 YOLO 모델 로드"""
        # OpenCV DNN YOLO는 별도 설정이 필요합니다
        # 여기서는 기본 구조만 제공
        print("OpenCV DNN YOLO는 별도 설정이 필요합니다.")
        print("ultralytics를 설치하는 것을 권장합니다: pip install ultralytics")

    def detect_objects(self, frame: np.ndarray) -> List[Detection]:
        """
        프레임에서 객체를 감지합니다.

        Args:
            frame: BGR 형식의 이미지 프레임

        Returns:
            감지된 객체 목록
        """
        if self.yolo_model is None:
            return []

        detections = []

        if self.use_ultralytics:
            # Ultralytics YOLO 사용
            results = self.yolo_model(
                frame, conf=self.confidence_threshold, verbose=False
            )

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # 바운딩 박스 좌표
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = self.yolo_model.names[class_id]

                    # 중심점 계산
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    detections.append(
                        Detection(
                            class_id=class_id,
                            class_name=class_name,
                            confidence=confidence,
                            bbox=(x1, y1, x2, y2),
                            center=(center_x, center_y),
                        )
                    )
        else:
            # OpenCV DNN 사용 (구현 필요)
            pass

        return detections

    def filter_obstacles(self, detections: List[Detection]) -> List[Detection]:
        """장애물만 필터링"""
        return [
            det
            for det in detections
            if det.class_name.lower() in [cls.lower() for cls in self.obstacle_classes]
        ]

    def calculate_obstacle_direction(
        self, obstacles: List[Detection]
    ) -> Dict[str, float]:
        """
        장애물의 위치를 분석하여 회피 방향을 계산합니다.

        Returns:
            방향 정보 딕셔너리
            - left_danger: 왼쪽 위험도 (0.0-1.0)
            - right_danger: 오른쪽 위험도 (0.0-1.0)
            - center_danger: 중앙 위험도 (0.0-1.0)
            - avg_obstacle_x: 평균 장애물 X 위치
        """
        if not obstacles:
            return {
                "left_danger": 0.0,
                "right_danger": 0.0,
                "center_danger": 0.0,
                "avg_obstacle_x": self.frame_center_x,
            }

        # 화면을 3개 영역으로 분할
        left_boundary = self.frame_width // 3
        right_boundary = 2 * self.frame_width // 3

        left_danger = 0.0
        right_danger = 0.0
        center_danger = 0.0
        total_weight = 0.0
        weighted_x_sum = 0.0

        for obstacle in obstacles:
            x, y = obstacle.center
            x1, y1, x2, y2 = obstacle.bbox

            # 거리 기반 가중치 (가까울수록 위험)
            distance = np.sqrt(
                (x - self.frame_center_x) ** 2 + (y - self.frame_center_y) ** 2
            )
            weight = 1.0 / (1.0 + distance / 100.0)  # 거리 기반 가중치

            # 크기 기반 가중치 (클수록 위험)
            size = (x2 - x1) * (y2 - y1)
            size_weight = min(size / (self.frame_width * self.frame_height), 1.0)

            total_weight += weight * size_weight * obstacle.confidence
            weighted_x_sum += x * weight * size_weight * obstacle.confidence

            # 영역별 위험도 계산
            if x < left_boundary:
                left_danger += weight * size_weight * obstacle.confidence
            elif x > right_boundary:
                right_danger += weight * size_weight * obstacle.confidence
            else:
                center_danger += weight * size_weight * obstacle.confidence

        # 정규화
        if total_weight > 0:
            left_danger /= total_weight
            right_danger /= total_weight
            center_danger /= total_weight
            avg_obstacle_x = weighted_x_sum / total_weight
        else:
            avg_obstacle_x = self.frame_center_x

        return {
            "left_danger": min(left_danger, 1.0),
            "right_danger": min(right_danger, 1.0),
            "center_danger": min(center_danger, 1.0),
            "avg_obstacle_x": int(avg_obstacle_x),
        }

    def generate_control_command(
        self, frame: np.ndarray, current_roll: int = 100, current_pitch: int = 100
    ) -> ControlCommand:
        """
        객체 감지 결과를 기반으로 조종 명령을 생성합니다.

        Args:
            frame: 현재 프레임
            current_roll: 현재 Roll 값
            current_pitch: 현재 Pitch 값

        Returns:
            조종 명령
        """
        # 객체 감지
        detections = self.detect_objects(frame)
        obstacles = self.filter_obstacles(detections)

        # 기본값 (직진)
        roll = current_roll
        pitch = current_pitch
        yaw = self.mid
        throttle = 50  # 기본 추력

        if obstacles:
            # 장애물이 있는 경우 회피 로직
            danger_info = self.calculate_obstacle_direction(obstacles)

            # 위험도가 높으면 정지 또는 후진
            if danger_info["center_danger"] > 0.7:
                # 중앙에 장애물이 매우 가까움 - 후진
                pitch = max(1, current_pitch - self.max_pitch)
                throttle = 30
            elif danger_info["center_danger"] > 0.4:
                # 중앙에 장애물이 있음 - 감속
                throttle = 40
            else:
                # 중앙은 안전, 좌우 중 안전한 쪽으로 이동
                if danger_info["left_danger"] > danger_info["right_danger"]:
                    # 왼쪽이 더 위험 - 오른쪽으로 회피
                    roll = min(200, current_roll + self.max_roll)
                else:
                    # 오른쪽이 더 위험 - 왼쪽으로 회피
                    roll = max(1, current_roll - self.max_roll)

                # 앞으로 진행
                pitch = min(200, current_pitch + self.speed)
                throttle = 60
        else:
            # 장애물이 없으면 직진
            pitch = min(200, current_pitch + self.speed)
            throttle = 60

        # 값 범위 제한
        roll = max(1, min(200, roll))
        pitch = max(1, min(200, pitch))
        yaw = max(1, min(200, yaw))
        throttle = max(0, min(255, throttle))

        return ControlCommand(roll=roll, pitch=pitch, yaw=yaw, throttle=throttle)

    def draw_detections(
        self, frame: np.ndarray, detections: List[Detection]
    ) -> np.ndarray:
        """프레임에 감지 결과를 그립니다."""
        result_frame = frame.copy()

        for det in detections:
            x1, y1, x2, y2 = det.bbox
            center_x, center_y = det.center

            # 바운딩 박스 그리기
            color = (
                (0, 255, 0)
                if det.class_name.lower()
                in [cls.lower() for cls in self.obstacle_classes]
                else (255, 0, 0)
            )
            cv2.rectangle(result_frame, (x1, y1), (x2, y2), color, 2)

            # 라벨 그리기
            label = f"{det.class_name}: {det.confidence:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(
                result_frame,
                (x1, y1 - label_size[1] - 10),
                (x1 + label_size[0], y1),
                color,
                -1,
            )
            cv2.putText(
                result_frame,
                label,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )

            # 중심점 그리기
            cv2.circle(result_frame, (center_x, center_y), 5, color, -1)

        return result_frame
        return result_frame
