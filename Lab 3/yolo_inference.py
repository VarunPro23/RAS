import cv2
from ultralytics import YOLO

model = YOLO('yolov8s.pt')  

frame = cv2.imread(r"apple.png")

result = model(frame, verbose=False)

r = result[0]

if r.boxes is not None and len(r.boxes) > 0:

    xyxy = r.boxes.xyxy.cpu().numpy()

    conf = r.boxes.conf.cpu().numpy()
    cls_  = r.boxes.cls.cpu().numpy().astype(int)
    names = r.names  

    print("conf", conf)
    print("cls", cls_)
    print()
    print("names", names)

    for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls_):
        
        if conf >= 0.75:

            x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
            w, h = x2 - x1, y2 - y1
            cx, cy = x1 + w // 2, y1 + h // 2  
            label = f"{names[k]} {c:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 3, (0, 255, 255), -1)
            cv2.putText(frame, label, (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

cv2.imwrite(r"pred.png", frame)