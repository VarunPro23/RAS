import cv2
import time
from ultralytics import YOLO
from pydobot import Dobot

cap = cv2.VideoCapture('/dev/video0')

# Coordinates
PICKUP_POS = (240, 5, -42)
BASKET_A = (250, 143, 75)  # Food basket
BASKET_B = (-10, 143, 75)  # Vehicle basket
HOME_POS = (240, -4, 34, 0)  # Home position

#PORT
PORT = "/dev/ttyACM0"

# Categories
FOOD = {'apple', 'banana', 'pizza'}
VEHICLE = {'car', 'bicycle', 'airplane'}

# Initialize YOLO
model = YOLO('yolov8s.pt')
print("YOLO model loaded")

# Initialize Dobo
dobot = Dobot(port=PORT)

# Go to home position
dobot.move_to(*HOME_POS)

cycle = 1
# ---- Camera ON ----
cap = cv2.VideoCapture("/dev/video2")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Camera feed ON")

while True:
    print(f"\n=== Cycle {cycle} ===")
    print("Capturing for 3 seconds...")

    frames = []
    start = time.time()
    while time.time() - start < 5:
        ret, frame = cap.read()
        if not ret:
            continue
        frames.append(frame)

    print(f"Captured {len(frames)} frames")

    detected_objects = set()

    if frames:
        for frame in frames:
            # Run YOLO inference
            results = model(frame, verbose=False)[0]
            if results.boxes is not None:
                for box, conf, cls in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
                    if conf >= 0.90:
                        x1, y1, x2, y2 = map(int, box)
                        obj_name = results.names[int(cls)]
                        label = f"{obj_name} {conf:.2f}"
                        color = (0, 255, 0) if obj_name in FOOD else (0, 0, 255)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        if obj_name in FOOD or obj_name in VEHICLE:
                            detected_objects.add(obj_name)

            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nStopping experiment...")
                cap.release()
                cv2.destroyAllWindows()
                dobot.close()
                print("Cleanup complete!")
                exit(0)

    # Detect objects
    if detected_objects:
        print(f"Detected objects: {list(detected_objects)}")
        print("Waiting 2 seconds before pickup...")
        time.sleep(2)

        for obj in detected_objects:
            print(f"\nProcessing: {obj}")

            # Pickup
            dobot.move_to(PICKUP_POS[0], PICKUP_POS[1], PICKUP_POS[2] + 20, 0)
            dobot.move_to(*PICKUP_POS, 0)
            dobot.suck(True)
            time.sleep(2)
            dobot.move_to(PICKUP_POS[0], PICKUP_POS[1], PICKUP_POS[2] + 20, 0)

            # Destination
            if obj in FOOD:
                basket = BASKET_A
                print(f"→ FOOD detected: {obj} → Basket A")
            elif obj in VEHICLE:
                basket = BASKET_B
                print(f"→ VEHICLE detected: {obj} → Basket B")
            else:
                print(f"→ UNKNOWN category: {obj}, dropping here")
                dobot.suck(False)
                time.sleep(2)
                continue

            # Place in basket
            dobot.move_to(basket[0], basket[1], basket[2] + 20, 0)
            dobot.move_to(*basket, 0)
            dobot.suck(False)
            dobot.move_to(basket[0], basket[1], basket[2] + 20, 0)

        # Return home AFTER dropping
        print("Returning to home position...")
        dobot.move_to(*HOME_POS)
        print("✓ Cycle complete!")

    else:
        print("No objects detected this cycle")

    cycle += 1
    time.sleep(2)
