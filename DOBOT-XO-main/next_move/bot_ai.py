# Constants for players
BOT = 'X'
PLAYER = 'O'
EMPTY = 'NA'

def check_winner(board):
    
    # Check rows and columns
    for i in range(3):
        if board[i][0] == board[i][1] == board[i][2] and board[i][0] != EMPTY:
            return board[i][0]
        if board[0][i] == board[1][i] == board[2][i] and board[0][i] != EMPTY:
            return board[0][i]

    # Check diagonals
    if board[0][0] == board[1][1] == board[2][2] and board[0][0] != EMPTY:
        return board[0][0]
    if board[0][2] == board[1][1] == board[2][0] and board[0][2] != EMPTY:
        return board[0][2]

    # Check for a draw (if no empty cells are left)
    if all(board[i][j] != EMPTY for i in range(3) for j in range(3)):
        return 'draw'

    # No winner yet
    return None

def minimax(board, is_maximizing):

    winner = check_winner(board)
    if winner is not None:
        if winner == BOT:
            return 1  # Bot wins
        elif winner == PLAYER:
            return -1 # Player wins
        elif winner == 'draw':
            return 0  # Draw

    if is_maximizing:
        best_score = -float('inf')
        for r in range(3):
            for c in range(3):
                if board[r][c] == EMPTY:
                    board[r][c] = BOT
                    score = minimax(board, False) # Switch to minimizing player
                    board[r][c] = EMPTY # Backtrack
                    best_score = max(score, best_score)
        return best_score
    else: # Minimizing player's turn
        best_score = float('inf')
        for r in range(3):
            for c in range(3):
                if board[r][c] == EMPTY:
                    board[r][c] = PLAYER
                    score = minimax(board, True) # Switch to maximizing player
                    board[r][c] = EMPTY # Backtrack
                    best_score = min(score, best_score)
        return best_score

def find_best_move(board):

    best_score = -float('inf')
    move = (-1, -1)

    for r in range(3):
        for c in range(3):
            if board[r][c] == EMPTY:
                board[r][c] = BOT
                score = minimax(board, False) # False because it's the player's turn next
                board[r][c] = EMPTY # Backtrack to the original state

                if score > best_score:
                    best_score = score
                    move = (r, c)
    return move
