from pathlib import Path
import cv2
from ultralytics import YOLO

# 改成你的 best.pt 路徑
MODEL_PATH = r"C:\Users\brian\Desktop\school\mobile-robotics\wolf_rabbit_yolo\runs\wolf_rabbit_signs_v2\weights\best.pt"

def open_camera():
    # 先試最常見的主相機 index=0
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        return cap

    # Windows 有時候改用 DirectShow 比較穩
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if cap.isOpened():
        return cap

    # 再試第二顆相機
    cap = cv2.VideoCapture(1)
    if cap.isOpened():
        return cap

    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if cap.isOpened():
        return cap

    return None

def main():
    model = YOLO(MODEL_PATH)

    cap = open_camera()
    if cap is None:
        print("Cannot open camera. Try camera index 1 or close other apps using the webcam.")
        return

    # 可選：設定解析度
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("Webcam started. Press 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame from camera.")
            break

        # YOLO 即時推論
        results = model(frame, conf=0.25, verbose=False)

        # 把框與標籤畫回影像
        annotated = results[0].plot()

        cv2.imshow("Wolf/Rabbit Webcam Test", annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()