import bot_ai
import time

# Create a mapping from number (1-9) to board coordinates (row, col)
# This makes it easy to convert user input.
# Example: 5 -> (1, 1) which is the center
COORDINATE_MAP = {
    1: (0, 0), 2: (0, 1), 3: (0, 2),
    4: (1, 0), 5: (1, 1), 6: (1, 2),
    7: (2, 0), 8: (2, 1), 9: (2, 2)
}

# The empty marker must be the same as in bot_ai.py
EMPTY = 'NA'

def print_board(board):
    """Prints the Tic-Tac-Toe board to the console."""
    print("\n-------------")
    for row in board:
        # Replace 'NA' with a space for cleaner display
        display_row = [cell if cell != EMPTY else ' ' for cell in row]
        print(f"| {display_row[0]} | {display_row[1]} | {display_row[2]} |")
        print("-------------")

def print_board_guide():
    """Prints the numbered guide for the user to choose a move."""
    print("\n--- Move Guide ---")
    print("Please use the following numbers to make your move:")
    print("-------------")
    print("| 1 | 2 | 3 |")
    print("-------------")
    print("| 4 | 5 | 6 |")
    print("-------------")
    print("| 7 | 8 | 9 |")
    print("-------------\n")

def get_player_move(board, player_symbol):
    """Prompts the user for a move, validates it, and returns the coordinates."""
    while True:
        try:
            move = int(input(f"Player '{player_symbol}', enter your move (1-9): "))
            if move < 1 or move > 9:
                print("Invalid input. Please enter a number between 1 and 9.")
                continue
            
            row, col = COORDINATE_MAP[move]
            
            if board[row][col] == EMPTY:
                return row, col
            else:
                print("That spot is already taken! Please choose another.")

        except ValueError:
            print("Invalid input. Please enter a number.")

def play_game():
    """The main function to run the Tic-Tac-Toe game."""
    board = [[EMPTY, EMPTY, EMPTY] for _ in range(3)]

    # --- Player Setup ---
    while True:
        choice = input("Do you want to play first? (y/n): ").lower()
        if choice in ['y', 'n']:
            break
        print("Invalid choice. Please enter 'y' or 'n'.")

    if choice == 'y':
        player_symbol = 'X'
        bot_symbol = 'O'
        current_turn = 'player'
    else:
        player_symbol = 'O'
        bot_symbol = 'X'
        current_turn = 'bot'

    # **Crucial Step**: Update the constants in the imported bot_ai module
    # This tells the Minimax algorithm which symbol it's playing as.
    bot_ai.BOT = bot_symbol
    bot_ai.PLAYER = player_symbol
    
    print(f"\nYou are playing as '{player_symbol}'. The bot is '{bot_symbol}'.")
    print_board_guide()
    time.sleep(2)

    # --- Main Game Loop ---
    while True:
        print_board(board)
        winner = bot_ai.check_winner(board)

        # Check for a winner or a draw
        if winner:
            if winner == 'draw':
                print("It's a draw!")
            elif winner == player_symbol:
                print(f"Congratulations! You ('{player_symbol}') won!")
            else:
                print(f"The Bot ('{bot_symbol}') won. Better luck next time!")
            break

        # --- Turn Handling ---
        if current_turn == 'player':
            row, col = get_player_move(board, player_symbol)
            board[row][col] = player_symbol
            current_turn = 'bot'
        else: # Bot's turn
            print("Bot is thinking...")
            time.sleep(1) # Add a small delay to simulate thinking
            
            move = bot_ai.find_best_move(board)
            if move != (-1, -1):
                board[move[0]][move[1]] = bot_symbol
                # Find the number corresponding to the bot's move for a nice printout
                move_number = [k for k, v in COORDINATE_MAP.items() if v == move][0]
                print(f"Bot chose box {move_number} ({move})")
            
            current_turn = 'player'

# --- Start the game ---
if __name__ == "__main__":
    play_game()

