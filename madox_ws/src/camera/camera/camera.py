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
        # cv2.FONT_HERSHEY_SIMPLEX = 0
        from apscheduler.schedulers.background import BackgroundScheduler

        self.source = source
        self.width = width
        self.height = height
        self.scheduler = BackgroundScheduler()
        self.scheduler.start()
        if self.source == "jetson":
            capture_device = (
                "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)"
                + str(self.width)
                + ", height=(int)"
                + str(self.height)
                + ",format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink"
            )
            self.cap = cv2.VideoCapture(capture_device)
        else:
            self.source = int(source)
            self.cap = cv2.VideoCapture(self.source)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # self.cap = cv2.VideoCapture(
        #     gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER
        # )
        # Comment this and uncomment the code above to change the capture device (e.g.: use otehr cameras)
        #

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
