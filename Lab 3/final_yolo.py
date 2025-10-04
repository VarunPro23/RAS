import cv2
import time
from ultralytics import YOLO
from pydobot import Dobot

# Find working camera
# def find_camera():
#     cap = cv2.VideoCapture('/dev/video4')
#     if cap.isOpened() and cap.read()[0]:
#         # print(f"Camera found at index {i}")
#         return cap
#     cap.release()

cap = cv2.VideoCapture('/dev/video0')

# Coordinates
PICKUP_POS = (250, 12.03, -40.00)
BASKET_A = (258.29, -213.49, 41.72)  # Food basket
BASKET_B = (275.09, 240.00, 44.05)  # Vehicle basket
HOME_POS = (203.27, 10.36, -14.22, 0)  # Home position

#PORT
PORT = "/dev/ttyACM0"

# Categories
FOOD = {'apple', 'banana', 'pizza'}
VEHICLE = {'car', 'bicycle', 'airplane'}

# Initialize YOLO
model = YOLO('yolov8s.pt')
print("YOLO model loaded")

# Initialize Dobot
try:
    dobot = Dobot(port=PORT)
    print("✓ Real Dobot connected on /dev/ttyACM0!")
except Exception as e:    
    print("Using Dobot simulation")

# Go to home position
dobot.move_to(*HOME_POS)
print("At home position, starting continuous sorting...")

try:
    # ---- Camera ON ----
    # cap = find_camera()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("Camera feed ON")
    cycle = 1
    while True:
        print(f"\n=== Cycle {cycle} ===")


        print("Capturing for 3 seconds...")
        frames = []
        start = time.time()
        while time.time() - start < 5:
            ret, frame = cap.read()
            if not ret:
                continue

            # Run YOLO inference
            results = model(frame, verbose=False)[0]
            detected_objects = set()
            # Draw detections
            if results.boxes is not None:
                for box, conf, cls in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
                    if conf >= 0.90:
                        x1, y1, x2, y2 = map(int, box)
                        obj_name = results.names[int(cls)]
                        label = f"{obj_name} {conf:.2f}"
                        print("detected_object", label, VEHICLE, x1,y1,x2,y2, label.strip() in VEHICLE)
                        color = (0, 255, 0) if obj_name in FOOD else (0, 0, 255)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                        
                        if obj_name in FOOD or obj_name in VEHICLE:
                            print("adding", obj_name)
                            detected_objects.add(obj_name)
                        
                        

            frames.append(frame)

            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt

        print(f"Captured {len(frames)} frames")

        # # ---- Camera OFF ----
        # cap.release()
        # cv2.destroyAllWindows()
        # print("Camera feed OFF")

        # Process detections
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

except KeyboardInterrupt:
    print("\nStopping experiment...")
finally:
    try:
        cap.release()
    except:
        pass
    cv2.destroyAllWindows()
    dobot.move_to(*HOME_POS)
    dobot.close()
    print("Cleanup complete!")