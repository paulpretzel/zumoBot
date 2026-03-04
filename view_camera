import pyrealsense2 as rs
import numpy as np
import cv2

def show_camera_feed():
    print("Initializing RealSense Camera...")
    
    # 1. Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # We use 640x480 at 30fps. It's a great balance of quality and performance for the Pi.
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 2. Start streaming
    pipeline.start(config)
    print("Camera feed started! Make sure you are in the desktop environment to see the window.")
    print("Press 'q' on the video window to quit.")

    try:
        while True:
            # Wait for a coherent pair of frames
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue

            # 3. Convert frames to numpy arrays for OpenCV
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 4. Apply colormap on depth image 
            # (Converts 16-bit depth to an 8-bit heatmap image)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 5. Stack both images horizontally side-by-side
            images = np.hstack((color_image, depth_colormap))

            # 6. Show the images in a window
            cv2.namedWindow('RealSense Live Feed (RGB + Depth)', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense Live Feed (RGB + Depth)', images)

            # Press 'q' to break the loop and close
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                print("Closing camera feed...")
                break

    finally:
        # Stop streaming and clean up windows
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    show_camera_feed()
