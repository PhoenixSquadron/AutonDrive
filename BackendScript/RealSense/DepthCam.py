import numpy as np
import cv2
import pyrealsense2 as rs

def display_depth_frame(depth_frame):
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    cv2.imshow('Depth Map', depth_colormap)
    cv2.waitKey(1)

if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()

            if not depth_frame:
                continue

            display_depth_frame(depth_frame)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
