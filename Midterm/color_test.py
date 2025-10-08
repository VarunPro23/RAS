import cv2
import numpy as np

# ------------------------------
# Open camera
# ------------------------------
cap = cv2.VideoCapture('/dev/video2')  # change if needed

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Resize for consistency
    frame = cv2.resize(frame, (640, 480))

    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ------------------------------
    # Define color ranges
    # ------------------------------

    # --- Red ---
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    # --- Yellow ---
    # Yellow hue ≈ 20–35 in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Apply morphological filters (reduce noise)
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)

    # ------------------------------
    # Detect Red (X)
    # ------------------------------
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_detected = False
    for cnt in contours_red:
        area = cv2.contourArea(cnt)
        if area > 500:  # ignore small noise
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "X", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            red_detected = True

    # ------------------------------
    # Detect Yellow (O)
    # ------------------------------
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    yellow_detected = False
    for cnt in contours_yellow:
        area = cv2.contourArea(cnt)
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(frame, "O", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
            yellow_detected = True

    # ------------------------------
    # Display detection results
    # ------------------------------
    if red_detected:
        cv2.putText(frame, "Detected: X (Red Block)", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        print("Detected: X (Red Block)")
    elif yellow_detected:
        cv2.putText(frame, "Detected: O (Yellow Block)", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
        print("Detected: O (Yellow Block)")

    # Display the video feed
    cv2.imshow("Tic Tac Toe Color Detection", frame)

    # Quit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
