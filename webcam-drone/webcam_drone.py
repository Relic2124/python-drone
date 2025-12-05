import cv2
from ultralytics import YOLO

# 1. 모델 불러오기
# 'yolov8n.pt' : 사람, 컵, 핸드폰 등 80가지를 아는 기본 모델
# 'runs/detect/train/weights/best.pt' : 방금 내가 학습시킨 커스텀 모델
# model = YOLO('runs/detect/train/weights/best.pt')
model = YOLO('yolov8n-drone.pt')

# url = "http://192.168.35.74:8080/video" 

# 2. 웹캠 켜기
# 숫자 0은 보통 '노트북 기본 캠'을 의미합니다.
# USB 캠을 따로 꽂았다면 1번일 수도 있어요.
cap = cv2.VideoCapture(1)

# 웹캠이 안 켜지면 에러 메시지 띄우기
if not cap.isOpened():
    print("웹캠을 열 수 없어요! 카메라 연결을 확인해주세요.")
    exit()

fourcc = cv2.VideoWriter_fourcc(*'mp4v')   # 코덱
out = None
recording = False

try:
    print("[키] q: 종료, r: 녹화 시작/종료")
    # 3. 실시간으로 한 프레임씩 읽어서 예측하기 (무한 루프)
    while True:
        # 카메라에서 사진 한 장(frame) 읽어오기
        success, frame = cap.read()
        
        if not success:
            print("프레임을 읽을 수 없습니다.")
            break

        # 4. YOLO에게 물어보기 (예측)
        # stream=True: 영상처럼 연속된 데이터일 때 메모리를 아껴줍니다.
        results = model(frame, stream=True)

        # 5. 결과 그리기
        # results는 리스트 형태라서 반복문을 돌며 그림을 그려야 해요.
        for result in results:
            # plot(): 네모 박스와 이름표를 사진 위에 그려주는 함수
            annotated_frame = result.plot()
            
            # 6. 화면에 보여주기
            cv2.imshow("My YOLO Webcam", annotated_frame)
            if recording:
                out.write(annotated_frame)  # 프레임 저장

        # 7. 종료 조건 ('q' 키를 누르면 꺼짐)
        key = cv2.waitKey(1) & 0xFF
        if cv2.waitKey(1) == ord('q'): break
        elif key == ord('r'): # 녹화 시작/종료
            if not recording:
                h, w = frame.shape[:2]
                out = cv2.VideoWriter('./result/record.mp4', fourcc, 20.0, (w, h))
                recording = True
                print('녹화 시작')
            else:
                recording = False
                out.release()
                print('녹화 종료')

finally:
    # 8. 뒷정리 (카메라 끄고 창 닫기)
    cap.release()
    cv2.destroyAllWindows()