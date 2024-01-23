import cv2
import numpy as np
import pyrealsense2 as rs

# Constants for line detection
THRESHOLD_MIN = 100
THRESHOLD_MAX = 255
CANNY_THRESHOLD1 = 50
CANNY_THRESHOLD2 = 150
HOUGH_RHO = 1
HOUGH_THETA = np.pi / 180
HOUGH_THRESHOLD = 50
HOUGH_MIN_LINE_LENGTH = 10
HOUGH_MAX_LINE_GAP = 10

def process_frame(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply thresholding to segment the line
    _, thresholded = cv2.threshold(gray, THRESHOLD_MIN, THRESHOLD_MAX, cv2.THRESH_BINARY)

    # Apply Canny edge detection
    edges = cv2.Canny(thresholded, CANNY_THRESHOLD1, CANNY_THRESHOLD2)

    # Apply Hough line transformation to detect lines
    lines = cv2.HoughLinesP(edges, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD, minLineLength=HOUGH_MIN_LINE_LENGTH,
                            maxLineGap=HOUGH_MAX_LINE_GAP)

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

    return frame

if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            processed_frame = process_frame(color_image)

            cv2.imshow('Line Following', processed_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
