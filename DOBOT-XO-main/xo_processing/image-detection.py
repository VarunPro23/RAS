import cv2
import time

camera_port = 0
capture_interval = 5
last_capture_time = time.time()

cap = cv2.VideoCapture(camera_port)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# -----------------------

if not cap.isOpened():
    print("Cannot open stream. Check the camera port.")
else:
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    # print(f"Frame size set to: {int(width)}x{int(height)}")

    while True:
        ret, frame = cap.read()

        current_time = time.time()

        if not ret:
            print("Je ne reçois pas l'image.")
            break
        
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # edges = cv2.Canny(blur, 50, 150)

    
        # cv2.imshow("Gris", gray)
        # cv2.imshow("Bords (Canny)", edges)
        
        cv2.imshow("Image originale", frame)

        if current_time - last_capture_time >= capture_interval:
            # print(f"Capturing image at interval: {current_time - last_capture_time:.2f}s")
            last_capture_time = current_time
            cv2.imwrite('image.jpg', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
cap.release()