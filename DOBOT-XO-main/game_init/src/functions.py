OUTPUT_TOOLPATH_FILE = "temp/toolpath.txt" 




def save_to_file(strokes, DOBOT_Z_MOVE, DOBOT_Z_DRAW, DOBOT_X_MIN, DOBOT_X_MAX, DOBOT_Y_MIN, DOBOT_Y_MAX):
    """Saves the final toolpath to a text file."""
    with open(OUTPUT_TOOLPATH_FILE, 'w') as f:
        # f.write(f"MOVETO,{DOBOT_X_MIN:.4f},{DOBOT_Y_MAX:.4f},{DOBOT_Z_MOVE:.4f}\n")
        # f.write(f"LINETO,{DOBOT_X_MIN:.4f},{DOBOT_Y_MAX:.4f},{DOBOT_Z_DRAW:.4f}\n")
        # f.write(f"LINETO,{DOBOT_X_MAX:.4f},{DOBOT_Y_MAX:.4f},{DOBOT_Z_DRAW:.4f}\n")
        # f.write(f"LINETO,{DOBOT_X_MAX:.4f},{DOBOT_Y_MIN:.4f},{DOBOT_Z_DRAW:.4f}\n")
        # f.write(f"LINETO,{DOBOT_X_MIN:.4f},{DOBOT_Y_MIN:.4f},{DOBOT_Z_DRAW:.4f}\n")
        # f.write(f"LINETO,{DOBOT_X_MIN:.4f},{DOBOT_Y_MAX:.4f},{DOBOT_Z_DRAW:.4f}\n")

        for stroke in strokes:
            if not stroke:
                continue
            
            # 1. Pen up, move to the start of the stroke
            start_point = stroke[0]
            f.write(f"MOVETO,{start_point[0]:.4f},{start_point[1]:.4f},{DOBOT_Z_MOVE:.4f}\n")
            
            # 2. Pen down
            f.write(f"LINETO,{start_point[0]:.4f},{start_point[1]:.4f},{DOBOT_Z_DRAW:.4f}\n")
            
            # 3. Draw the rest of the stroke
            for point in stroke[1:]:
                f.write(f"LINETO,{point[0]:.4f},{point[1]:.4f},{DOBOT_Z_DRAW:.4f}\n")
        
        # 4. Final pen up after the last stroke
        last_point = strokes[-1][-1]
        f.write(f"MOVETO,{last_point[0]:.4f},{last_point[1]:.4f},{DOBOT_Z_MOVE + 40:.4f}\n")
        f.write(f"LINETO,{146:.4f},{-220:.4f},{77:.4f}\n")


