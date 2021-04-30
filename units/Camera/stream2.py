import sys
import cv2


def read_cam():
    cap = cv2.VideoCapture(
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink"
    )
    if cap.isOpened():
        while True:
            ret_val, img = cap.read()
            cv2.imwrite("image.png", img)
            print(ret_val)

    else:
        print("camera open failed")


if __name__ == "__main__":
    read_cam()
