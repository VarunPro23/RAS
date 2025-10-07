import src.DobotDllType as dType

# --- CONFIGURATION ---
COM_PORT = "COM3"  # Adjust to your Dobot's COM port

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}


Z_DOWN = -55
grid_pos = (240, -60)

grid_shape = [[(5,20), (55,20)],
              [(55,40), (5,40)],
              [(20,5), (20,55)],
              [(40,55), (40,5)]]
X_shape = [[(4,4), (16,16)],
           [(4,16),(16,4)]]
O_shape = [[(17, 10), (16, 14), (13, 16), (10, 17), (6, 16), (4, 13), (3, 10), (4, 6), (7, 4), (10, 3), (14, 4), (16, 7), (17, 10)]]

def offset_shape(shape, offset_vector):

    if not shape:
        return []

    offset_x, offset_y = offset_vector
    offsetted_shape = []

    for stroke in shape:
        offsetted_stroke = []
        for x, y in stroke:
            offsetted_stroke.append((x + offset_x, y + offset_y))
        offsetted_shape.append(offsetted_stroke)

    return offsetted_shape

def draw_shape(strokes, api):

    lastIndex = dType.GetQueuedCmdCurrentIndex(api)[0]
    for stroke in strokes:
        start_point = stroke[0]
        lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode, start_point[0], start_point[1], Z_DOWN, 0, isQueued=1)[0]
        for point in stroke[1:]:
            lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, point[0], point[1], Z_DOWN, 0, isQueued=1)[0]
    
    last_point_of_stroke = stroke[-1]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode, last_point_of_stroke[0], last_point_of_stroke[1], 0, 0, isQueued=1)[0]
    # --- Execute the command queue ---
    print("Toolpath loaded. Starting execution...")
    dType.SetQueuedCmdStartExec(api)

    # Wait for Executing Last Command
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        # Changed to .format() for Python 3.5 compatibility
        print("Executing command... Current index: {}/{}".format(dType.GetQueuedCmdCurrentIndex(api)[0], lastIndex), end='\r')

        dType.dSleep(200) # Wait and check status periodically

    # Stop to Execute Command Queued
    dType.SetQueuedCmdStopExec(api)
    print("\nDrawing complete.")

    dType.SetQueuedCmdClear(api)

def draw_symbole_O(n, m, api):
    symbole_pos = (240 +n*20,-60+m*20)
    X_shape_offset = offset_shape(X_shape, symbole_pos)
    O_shape_offset = offset_shape(O_shape, symbole_pos)

    draw_shape(O_shape_offset, api)

def draw_symbole_X(n, m, api):
    symbole_pos = (240 +n*20,-60+m*20)
    X_shape_offset = offset_shape(X_shape, symbole_pos)
    O_shape_offset = offset_shape(O_shape, symbole_pos)

    draw_shape(X_shape_offset, api)



def main():
    # Load Dll
    api = dType.load()
    # Connect Dobot
    state = dType.ConnectDobot(api, COM_PORT, 115200)[0]
    print("Connect status:", CON_STR[state])

    if (state != dType.DobotConnect.DobotConnect_NoError):
        print("Failed to connect to Dobot. Please check the COM port and connection.")
        return

    print("Dobot connected. ")
    
    # Clean Command Queued
    dType.SetQueuedCmdClear(api)

    # Set motion parameters for drawing
    # You can adjust these values for speed and precision
    dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued=1)
    dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)
    dType.SetPTPCommonParams(api, 80, 80, isQueued=1) # Velocity and Acceleration Ratios
    # Set jump height for MOVETO commands
    # The Z-limit should be higher than your move height
    dType.SetPTPJumpParams(api, 20, 50, isQueued=1) 

    # Async Home - this is the first command in the queue
    print("Homing robot...")
    dType.SetHOMECmd(api, temp=0, isQueued=1)

    print("darwing grid....")
    
    grid_0 = offset_shape(grid_shape, (180,-60))
    grid_1 = offset_shape(grid_shape, (180,0))
    grid_2 = offset_shape(grid_shape, (240,-60))
    grid_3 = offset_shape(grid_shape, (240,0))
    
    # draw_shape(grid_0, api)
    # draw_shape(grid_1, api)
    draw_shape(grid_2, api)
    # draw_shape(grid_3, api)

    # n = 2
    # m = 0
    # symbole_pos = (240 +n*20,-60+m*20)
    # X_shape_offset = offset_shape(X_shape, symbole_pos)
    # O_shape_offset = offset_shape(O_shape, symbole_pos)

    # draw_shape(O_shape_offset, api)

    draw_symbole_X(1, 1, api)
    draw_symbole_O(0, 0, api)
    draw_symbole_X(0, 2, api)






    # Disconnect Dobot
    dType.DisconnectDobot(api)
    print("Dobot disconnected.")

if __name__ == "__main__":
    main()
