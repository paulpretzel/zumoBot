import pyrealsense2 as rs
import numpy as np
import cv2
import socket
import serial
import time
import math
import sys
import csv

# --- 1. System & Comms Configuration ---
ZUMO_SERIAL_PORT = '/dev/ttyACM0'  # RPi USB port format
BAUD_RATE = 115200

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# --- 2. Kinematic Control Law Parameters ---
K_w = 2.5           # Proportional gain for turning
MAX_V = 0.3         # Max linear speed (m/s)
AVOID_V = 0.05      # Slow forward speed while avoiding
AVOID_W = 2.0       # Sharp turn speed for avoidance
GOAL_TOL = 0.1      # Distance threshold to stop (m)

# --- 3. Camera Depth Parameters ---
SAFE_DISTANCE = 0.35      # Meters to trigger avoidance
MIN_SCAN_DISTANCE = 0.1   # Ignore sensor noise too close to lens
MAX_SCAN_DISTANCE = 1.5   # Max distance we care about

# --- 4. Region of Interest (ROI) Slicing ---
# Assuming 640x480 resolution. 
# We slice a horizontal band in the middle to ignore the floor and ceiling.
# Tweak these numbers depending on how high the camera is mounted.
ROI_TOP_ROW = 200     
ROI_BOTTOM_ROW = 300  

# --- Main Logic ---
def run_autonomous_zumo():
    # 1. Setup Serial
    try:
        zumo_serial = serial.Serial(ZUMO_SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[HW] Connected to Zumo on {ZUMO_SERIAL_PORT}")
    except Exception as e:
        print(f"[ERR] Serial Error: {e}")
        sys.exit(1)

    # 2. Setup UDP (Non-Blocking)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False) 
    print(f"[HW] Listening for Mocap on port {UDP_PORT}")

    # 3. Setup Camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # RealSense hardware filters
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    
    print("[HW] Starting RealSense Pipeline...")
    profile = pipeline.start(config)
    
    # Get depth scale to convert raw values to meters
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # 4. Setup OpenCV & CSV
    cv2.namedWindow("Raw Camera", cv2.WINDOW_AUTOSIZE)
    csv_file = open('obstacle_log.csv', mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    
    # Simpler header for pure numpy slicing
    header = ['Timestamp', 'State', 'V_Cmd', 'W_Cmd', 'Left_Dist', 'Center_Dist', 'Right_Dist']
    csv_writer.writerow(header)
    print("[HW] CSV Logging Started.")

    mocap_data = None

    try:
        while True:
            # --- A. Grab Latest Mocap Data (Flush buffer) ---
            try:
                while True:
                    data, _ = sock.recvfrom(1024)
                    mocap_data = data.decode().split()
            except BlockingIOError:
                pass # Buffer is empty, move on with latest data

            # --- B. Grab & Process Camera Frame ---
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame: continue

            # Show Raw Video
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow("Raw Camera", color_image)
            cv2.waitKey(1)

            # Apply filters and convert depth to meters
            depth_frame = temporal.process(spatial.process(decimation.process(depth_frame)))
            depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale

            # --- C. Pure NumPy Obstacle Avoidance ---
            # 1. Crop to our horizontal Region of Interest (ROI)
            depth_roi = depth_image[ROI_TOP_ROW:ROI_BOTTOM_ROW, :]
            
            # 2. Filter out bad pixels (zero) and stuff too far away
            valid_mask = (depth_roi > MIN_SCAN_DISTANCE) & (depth_roi < MAX_SCAN_DISTANCE)
            valid_depths = np.where(valid_mask, depth_roi, np.inf)

            # 3. Split screen into Left, Center, and Right zones
            # 640 cols total -> Left: 0-213, Center: 213-426, Right: 426-640
            left_zone = valid_depths[:, :213]
            center_zone = valid_depths[:, 213:427]
            right_zone = valid_depths[:, 427:]

            # 4. Find the closest object in each zone
            min_left = np.min(left_zone) if left_zone.size > 0 else MAX_SCAN_DISTANCE
            min_center = np.min(center_zone) if center_zone.size > 0 else MAX_SCAN_DISTANCE
            min_right = np.min(right_zone) if right_zone.size > 0 else MAX_SCAN_DISTANCE

            V_cmd, W_cmd = 0.0, 0.0
            state_str = "MOCAP_GOAL"

            # Check if there is an obstacle directly in front
            if min_center < SAFE_DISTANCE:
                state_str = "OBSTACLE_AVOID"
                
                V_cmd = AVOID_V
                # Turn toward whichever side has more distance available
                W_cmd = AVOID_W if min_left > min_right else -AVOID_W

            # --- D. Mocap Go-To-Goal Logic (If Path is Clear) ---
            if state_str == "MOCAP_GOAL" and mocap_data:
                zumo_x, zumo_y = float(mocap_data[0]), float(mocap_data[1])
                relative_heading = float(mocap_data[5])
                goal_x, goal_y = float(mocap_data[6]), float(mocap_data[7])
                
                distance = math.sqrt((goal_x - zumo_x)**2 + (goal_y - zumo_y)**2)
                
                if distance < GOAL_TOL:
                    state_str = "GOAL_REACHED"
                    V_cmd, W_cmd = 0.0, 0.0
                else:
                    W_cmd = K_w * relative_heading
                    speed_scale = max(0.0, 1.0 - (abs(relative_heading) / (math.pi / 2)))
                    V_cmd = MAX_V * speed_scale

            # --- E. Execute & Log ---
            command = f"{V_cmd:.3f},{W_cmd:.3f}\n"
            zumo_serial.write(command.encode('utf-8'))
            
            # Write cleaner CSV row
            csv_row = [time.time(), state_str, round(V_cmd,3), round(W_cmd,3), 
                       round(min_left,3), round(min_center,3), round(min_right,3)]
            csv_writer.writerow(csv_row)

            print(f"[{state_str}] L:{min_left:.2f} C:{min_center:.2f} R:{min_right:.2f} | V:{V_cmd:.2f} W:{W_cmd:.2f}")

    except KeyboardInterrupt:
        print("\n[SYS] Shutting down safely...")
    finally:
        pipeline.stop()
        command = "0.0,0.0\n"
        zumo_serial.write(command.encode('utf-8'))
        zumo_serial.close()
        csv_file.close()
        cv2.destroyAllWindows()
        print("[SYS] Resources released.")

if __name__ == "__main__":
    run_autonomous_zumo()
