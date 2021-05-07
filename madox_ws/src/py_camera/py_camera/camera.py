import cv2
import time
from threading import Lock


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


def gstreamer_pipeline(
    capture_width=3280,  # Set to your camera's highest resolution
    capture_height=2464,
    display_width=224,
    display_height=224,
    framerate=120,  # Set the value according to your camera
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


class Camera(metaclass=Singleton):
    def __init__(self, source, width, height):
        self._read_lock = Lock()
        # self.cap = cv2.VideoCapture(source)
        # self.cap = cv2.VideoCapture(
        #     gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER
        # )
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)320, height=(int)200,format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink"
        )
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def drawCrosshair(ret, frame):
        w = 320
        h = 200
        size = 10

        cv2.line(
            img=frame,
            pt1=(int(w / 2 - size), int(h / 2)),
            pt2=(int(w / 2 - 2 * size), int(h / 2)),
            color=(255, 255, 0),
            thickness=1,
            lineType=8,
            shift=0,
        )
        cv2.line(
            img=frame,
            pt1=(int(w / 2 + size), int(h / 2)),
            pt2=(int(w / 2 + 2 * size), int(h / 2)),
            color=(255, 255, 0),
            thickness=1,
            lineType=8,
            shift=0,
        )
        cv2.line(
            img=frame,
            pt1=(int(w / 2), int(h / 2) - size),
            pt2=(int(w / 2), int(h / 2) - 2 * size),
            color=(255, 255, 0),
            thickness=1,
            lineType=8,
            shift=0,
        )
        cv2.line(
            img=frame,
            pt1=(int(w / 2), int(h / 2) + size),
            pt2=(int(w / 2), int(h / 2) + 2 * size),
            color=(255, 255, 0),
            thickness=1,
            lineType=8,
            shift=0,
        )

    def draw_joy(ret, frame, x, y):
        thickness = 8
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_size = 0.3
        GREEN = (255, 255, 0)
        font_color = GREEN
        font_thickness = 1
        w = 320
        h = 200
        cv2.putText(
            frame,
            str(x),
            (5, h - 5),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            str(y),
            (40, h - 5),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def drawOverlay(ret, frame, f):
        thickness = 8
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_size = 0.3
        GREEN = (255, 255, 0)
        font_color = GREEN
        font_thickness = 1
        w = 320
        h = 200
        cv2.putText(
            frame,
            str(int(f)),
            (w - 20, h - 20),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def read(self, sleep_in_sec):
        self._read_lock.acquire()
        result = self.cap.read()
        time.sleep(sleep_in_sec)
        self._read_lock.release()
        return result

    def get_frame(self, sleep_in_sec):
        ret, frame = self.read(sleep_in_sec)

        if not ret:
            return None
        return frame

    def read_in_jpeg(self, sleep_in_sec, f):
        ret, frame = self.read(sleep_in_sec)
        # self.drawCrosshair(frame)
        # self.drawOverlay(frame, f)

        if not ret:
            return None
        ret, jpg = cv2.imencode(".jpg", frame)

        if not ret:
            return None
        return jpg

    def is_opened(self):
        return self.cap.isOpened()

    def release(self):
        self.cap.release()
