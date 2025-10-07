import cv2
import numpy as np
from XO_processing import split_into_cells, contenu_cell, split_boards

IMG = "image.jpg"

image = cv2.imread(IMG)

if image is None:
    print(f"Error: Could not load image. Make sure {IMG} is in the same folder.")
    exit()

height, width = image.shape[:2]

top_crop = 0
bottom_crop = 25
left_crop = 110
right_crop = 80

image = image[top_crop : height - bottom_crop, left_crop : width - right_crop]

# cv2.imshow("Detected Board", image)
# cv2.waitKey(0)


boards = split_boards(image)
board = boards[3]

# cv2.imshow("Detected Board", board)
# cv2.waitKey(0)



cells = split_into_cells(board)
statuses = contenu_cell(cells) 

board_state = np.array(statuses).reshape(3, 3)
print("Detected Board State:")
print(board_state)
print("-" * 20)

# cv2.imshow("Detected Board", board)
# cv2.waitKey(0)


cv2.destroyAllWindows()
