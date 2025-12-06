import cv2
import numpy as np
from heapq import heappush, heappop
import pickle

DT = cv2.distanceTransform


def dist_transform(free):
    return DT((free*255).astype('uint8'), cv2.DIST_L2, 3).astype(np.float32)

def remove_circles(binary_maze, circle_bboxes):

    ckpt_1 = None
    ckpt_2 = None

    for bbox in circle_bboxes:
        xmin, ymin, xmax, ymax = bbox


        binary_maze[int(ymin): int(ymax), int(xmin): int(xmax)] = 255
    

    return binary_maze

def astar(maze, start, goal, clearance_px=40, alpha=12.0, p=2.0, flag=False):
    h, w = maze.shape
    
    # Ensure start and goal are walkable
    if maze[start[0], start[1]] == 0:
        print(f"Start {start} is a wall!")
        return None
    if maze[goal[0], goal[1]] == 0:
        print(f"Goal {goal} is a wall!")
        return None
    
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    free = (maze == 255).astype(np.uint8)
    D = dist_transform(free)   

    if clearance_px > 0:
        # penalty in [0,1], high near walls, 0 beyond clearance
        penalty = np.clip((clearance_px - D) / max(clearance_px, 1e-6), 0.0, 1.0) ** p
    else:
        penalty = np.zeros_like(D, dtype=np.float32)

    # if flag:
    #     ylim1 = min(start[0], goal[0])
    #     ylim2 = max(start[0], goal[0])
    #     xlim1 = 0
    #     xlim2 = w
    # else:
    #     xlim1 = min(start[1], goal[1])
    #     xlim2 = max(start[1], goal[1])
    #     ylim1=0
    #     ylim2=h
    
    while open_set:
        _, current = heappop(open_set)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for dy, dx in [(-1,0), (1,0), (0,-1), (0,1)]:
            ny, nx = current[0] + dy, current[1] + dx
            neighbor = (ny, nx)
            
            if 0 <= ny < h  and 0 <= nx < w and maze[ny, nx] == 255:
                
                tentative_g = g_score[current] + 1
                step_cost = 1.0 + alpha * float(penalty[ny, nx])
                tentative_g = g_score[current] + step_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))
    
    return None


def run_astar_search(warped_maze, start, goal, transformed_bboxes, visualize=True):


    warped_gray = cv2.cvtColor(warped_maze, cv2.COLOR_BGR2GRAY)

    
   
    # print(transformed_bboxes), any pixel > 127 becomes 255 (white).
    binary_maze = cv2.adaptiveThreshold(
    warped_gray, 255,
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
    cv2.THRESH_BINARY,
    15, 5
)


    final_maze = remove_circles(binary_maze=binary_maze, circle_bboxes=transformed_bboxes)
    bin = final_maze.copy()

    cv2.circle(
        bin,                    # Image to draw on
        (start[1], start[0]),           # Center coordinates (x, y)
        radius=10,              # Radius of the circle
        color=(0, 255, 0),      # Color in BGR format (green)
        thickness=2             # Thickness (-1 for filled circle)
        ) 

    cv2.circle(
    bin,                    # Image to draw on
    (goal[1], goal[0]),           # Center coordinates (x, y)
    radius=10,              # Radius of the circle
    color=(0, 255, 0),      # Color in BGR format (green)
    thickness=2             # Thickness (-1 for filled circle)
)  

    print("start", start)
    print("goal", goal)
    
    # cv2.imshow("bin", bin)
    
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    if (start[0] / final_maze.shape[0]) <= 0.1 or (start[0] / final_maze.shape[0]) >= 0.9:
        flag = True
    
    else:
        flag=False

    path = astar(final_maze, start, goal, flag=flag)
    if visualize:
        img = warped_maze.copy()
        if path:
            for y, x in path:
                img[y, x] = [0, 0, 255]  # Red path
            img[start[0], start[1]] = [0, 255, 0]  # Green start
            img[goal[0], goal[1]] = [255, 0, 0]  # Blue goal
        
            cv2.imwrite('path_result.png', img)


    return path


if __name__ == "__main__":

    tf = [[286, 105, 328, 147], [2, 182, 40, 219]]
    start = (209, 30)
    goal = (115, 296)

    img = cv2.imread('maze.png')
    print(run_astar_search(img, start, goal, transformed_bboxes=tf))