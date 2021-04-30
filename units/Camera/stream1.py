import time
import cv2

# Cam properties
fps = 30.0
frame_width = 640
frame_height = 480
# Create capture
cap = cv2.VideoCapture(0)
flip_method = "flip"
# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
cap.set(cv2.CAP_PROP_FPS, fps)

# Define the gstreamer sink
gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=127.0.0.1 port=5000"


def __csi_pipeline(self, sensor_id=0):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            frame_width,
            frame_height,
            fps,
            flip_method,
            frame_width,
            frame_height,
        )
    )


# Check if cap is open
if cap.isOpened() is not True:
    print("Cannot open camera. Exiting.")
    quit()

# Create videowriter as a SHM sink
# out = cv2.VideoWriter(gst_str_rtp, 0, fps, (frame_width, frame_height), True)
out = cv2.VideoWriter(gst_str_rtp, 0, fps, (frame_width, frame_height), True)

# Loop it
while True:
    # Get the frame
    ret, frame = cap.read()
    # Check
    if ret is True:
        # Flip frame
        frame = cv2.flip(frame, 1)
        # Write to SHM
        out.write(frame)
    else:
        print("Camera error.")
        time.sleep(10)

cap.release()
