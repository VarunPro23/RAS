import time
import math

# ---------------------------------------------------
# Mock Dobot (no hardware required)
# ---------------------------------------------------
class MockDobot:
    def __init__(self, port=None):
        print("✅ Mock Dobot connected (simulation mode)")
    def move_to(self, x, y, z):
        print(f"[MOCK] Moving to ({x:.1f}, {y:.1f}, {z:.1f})")
    def suck(self, state: bool):
        print(f"[MOCK] {'Sucking ON' if state else 'Sucking OFF'}")
    def close(self):
        print("✅ Mock Dobot closed")

Dobot = MockDobot()

# ---------------------------------------------------
# Dobot positions (simulated)
# ---------------------------------------------------
grid_positions = [
    [(255, -35, -40), (255, -15, -40), (255, 5, -40)],
    [(275, -35, -40), (275, -15, -40), (275, 5, -40)],
    [(295, -35, -40), (290, -15, -40), (290, 5, -40)]
]
source_position = (255, 104, -10)
current_source_z = source_position[2]
home_position = (253, -4, 34)

# ---------------------------------------------------
# Game Board
# ---------------------------------------------------
board = [[0, 0, 0],
         [0, 0, 0],
         [0, 0, 0]]

# ---------------------------------------------------
# Game Logic
# ---------------------------------------------------
def check_winner(b):
    for i in range(3):
        if b[i][0] == b[i][1] == b[i][2] != 0:
            return b[i][0]
        if b[0][i] == b[1][i] == b[2][i] != 0:
            return b[0][i]
    if b[0][0] == b[1][1] == b[2][2] != 0:
        return b[0][0]
    if b[0][2] == b[1][1] == b[2][0] != 0:
        return b[0][2]
    return 0

def is_moves_left(b):
    return any(0 in row for row in b)

def evaluate(b):
    w = check_winner(b)
    if w == 2:
        return 10
    elif w == 1:
        return -10
    return 0

def minimax(b, depth, is_max):
    score = evaluate(b)
    if score != 0 or not is_moves_left(b):
        return score
    if is_max:
        best = -math.inf
        for i in range(3):
            for j in range(3):
                if b[i][j] == 0:
                    b[i][j] = 2
                    best = max(best, minimax(b, depth + 1, False))
                    b[i][j] = 0
        return best
    else:
        best = math.inf
        for i in range(3):
            for j in range(3):
                if b[i][j] == 0:
                    b[i][j] = 1
                    best = min(best, minimax(b, depth + 1, True))
                    b[i][j] = 0
        return best

def find_best_move(b):
    best_val = -math.inf
    move = (-1, -1)
    for i in range(3):
        for j in range(3):
            if b[i][j] == 0:
                b[i][j] = 2
                val = minimax(b, 0, False)
                b[i][j] = 0
                if val > best_val:
                    best_val = val
                    move = (i, j)
    return move

# ---------------------------------------------------
# Simulated Dobot move
# ---------------------------------------------------
def pick_and_place_block(source, dest):
    global current_source_z
    sx, sy, _ = source
    dx, dy, dz = dest

    Dobot.move_to(*home_position)
    time.sleep(0.2)
    Dobot.move_to(sx, sy, current_source_z)
    Dobot.suck(True)
    time.sleep(0.2)
    Dobot.move_to(sx, sy, current_source_z + 30)
    Dobot.move_to(*home_position)
    Dobot.move_to(dx, dy, dz)
    Dobot.suck(False)
    Dobot.move_to(*home_position)
    current_source_z -= 10
    print(f"🤖 Robot placed at {dest}\n")

# ---------------------------------------------------
# Game Setup
# ---------------------------------------------------
choice = input("Who plays first? (human/robot): ").strip().lower()
if choice not in ["human", "robot"]:
    choice = "human"

if choice == "human":
    human_symbol, robot_symbol, player_turn = 1, 2, True
else:
    human_symbol, robot_symbol, player_turn = 2, 1, False

robot_has_played = False
print("\n🧠 Starting Tic Tac Toe simulation...")
print("Board format: row,col (0-based, bottom-left is (0,0))\n")

# ---------------------------------------------------
# Main Game Loop
# ---------------------------------------------------
while True:
    # ----------------------------
    # HUMAN TURN
    # ----------------------------
    if player_turn:
        print("Your turn! Current board:")
        for row in board:
            print(row)
        move = input("Enter your move as row,col (e.g. 0,2): ").strip()
        try:
            r, c = map(int, move.split(","))
            if 0 <= r < 3 and 0 <= c < 3:
                if board[r][c] == 0:
                    board[r][c] = human_symbol
                    player_turn = False
                    robot_has_played = False
                else:
                    print("❌ Cell already occupied!")
                    continue
            else:
                print("❌ Invalid coordinates (must be 0–2).")
                continue
        except:
            print("❌ Invalid format. Try again (e.g. 1,1).")
            continue

        # ✅ Check after human move
        winner = check_winner(board)
        if winner != 0:
            print("🏁 Human wins!" if winner == human_symbol else "🏁 Robot wins!")
            break
        if not is_moves_left(board):
            print("🤝 It's a draw!")
            break

    # ----------------------------
    # ROBOT TURN
    # ----------------------------
    if not player_turn and not robot_has_played:
        print("🤖 Robot's turn...")
        row, col = find_best_move(board)
        if row != -1 and col != -1:
            print(f"Robot chooses cell ({row},{col})")
            pick_and_place_block(source_position, grid_positions[row][col])
            board[row][col] = robot_symbol
        robot_has_played = True
        player_turn = True

        # ✅ Check after robot move
        winner = check_winner(board)
        if winner != 0:
            print("🏁 Human wins!" if winner == human_symbol else "🏁 Robot wins!")
            break
        if not is_moves_left(board):
            print("🤝 It's a draw!")
            break


Dobot.close()
print("\n✅ Game over.")
