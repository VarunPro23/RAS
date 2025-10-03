import cv2
import time
from ultralytics import YOLO

# Find working camera
def find_camera():
    for i in range(3):
        for backend in [None, cv2.CAP_GSTREAMER, cv2.CAP_FFMPEG]:
            cap = cv2.VideoCapture(i) if backend is None else cv2.VideoCapture(i, backend)
            if cap.isOpened() and cap.read()[0]:
                return cap
            cap.release()
    raise RuntimeError("No camera found")

# Initialize
cap = find_camera()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
model = YOLO('yolov8s.pt')

print("Capturing for 3 seconds...")
frames = []
start = time.time()

# Capture phase
while time.time() - start < 3:
    ret, frame = cap.read()
    if ret:
        frames.append(frame)

print(f"Processing {len(frames)} frames...")

# Process phase
for i, frame in enumerate(frames):
    results = model(frame, verbose=False)[0]
    
    if results.boxes is not None:
        for box, conf, cls in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
            if conf >= 0.5:
                x1, y1, x2, y2 = map(int, box)
                label = f"{results.names[int(cls)]} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    cv2.putText(frame, f"Frame {i+1}/{len(frames)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imshow('YOLO Results', frame)
    
    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()