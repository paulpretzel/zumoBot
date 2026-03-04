import pyrealsense2 as rs
import numpy as np
import cv2
import serial
import time

# --- CONFIGURATION ---
COM_PORT = '/dev/ttyACM0'  # Standard Linux USB port for Arduino
BAUD_RATE = 115200
SAFE_DISTANCE = 0.35  # Stop if closer than 35 cm (0.35 meters)

def run_zumo():
    print("--- Zumo Driver + Live Feed Starting ---")

    # 1. Connect to the Zumo Arduino
    try:
        zumo = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)  # Give Arduino time to reboot
        print(f"SUCCESS: Connected to Zumo on {COM_PORT}")
    except Exception as e:
        print(f"ERROR: Could not connect to Zumo.\n{e}")
        zumo = None # We will allow it to run in "Vision Only" mode if unplugged

    # 2. Wake up the RealSense Camera
    print("Waking up RealSense...")
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable both Depth (for math) and Color (for your eyes)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    print("System Armed. Select the video window and press 'q' to stop.")
    
    try:
        while True:
            # Grab frames
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue

            # Convert color frame to numpy array for OpenCV
            color_image = np.asanyarray(color_frame.get_data())

            # 3. Check the "Center Patch" (y: 215-265, x: 295-345)
            distances = []
            for y in range(215, 265, 5):
                for x in range(295, 345, 5):
                    dist = depth_frame.get_distance(x, y)
                    if dist > 0.1:  # Ignore blind spot / camera errors
                        distances.append(dist)

            # 4. Make Decision & Update UI
            status_color = (0, 255, 0) # Default Green
            min_dist = 99.9
            
            if len(distances) > 0:
                min_dist = min(distances)
                
                if min_dist < SAFE_DISTANCE:
                    status_color = (0, 0, 255) # Turn Red
                    print(f"OBSTACLE! Stopping. (Dist: {min_dist:.2f}m)    ", end='\r')
                    if zumo: zumo.write(b'S')
                else:
                    print(f"PATH CLEAR. Driving. (Dist: {min_dist:.2f}m)    ", end='\r')
                    if zumo: zumo.write(b'G')
            else:
                if zumo: zumo.write(b'S') # Failsafe stop

            # 5. Draw the Targeting UI
            # Draw the square where the robot is looking
            cv2.rectangle(color_image, (295, 215), (345, 265), status_color, 2)
            
            # Draw the distance text on the screen
            text = f"Dist: {min_dist:.2f}m" if min_dist != 99.9 else "Dist: --"
            cv2.putText(color_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)

            # Show the video feed
            cv2.imshow('Zumo Robot View', color_image)

            # Check for 'q' key to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nManual Override: Shutting down...")
                break

    except KeyboardInterrupt:
        print("\nManual Override: Shutting down...")
    finally:
        if zumo:
            zumo.write(b'S')
            zumo.close()
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == "__main__":
    run_zumo()
