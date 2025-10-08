import cv2

cap = cv2.VideoCapture('/dev/video2')

def show_coords(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked at (x={x}, y={y})")

cv2.namedWindow("Calibration")
cv2.setMouseCallback("Calibration", show_coords)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Calibration", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
