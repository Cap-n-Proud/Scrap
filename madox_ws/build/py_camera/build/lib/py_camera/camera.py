import cv2
import time
from threading import Lock
import json


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


def drawing_def():

    thickness = 8
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_size = 0.3
    GREEN = (255, 255, 0)
    font_color = GREEN
    font_thickness = 1
    w = 320
    h = 200


class Camera(metaclass=Singleton):
    def __init__(self, source, width, height):
        self._read_lock = Lock()
        # self.cap = cv2.VideoCapture(source)
        # self.cap = cv2.VideoCapture(
        #     gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER
        # )
        # cv2.FONT_HERSHEY_SIMPLEX = 0
        self.text_overlay_settings = '{ "thickness":0, "font":0, "font_size":0.3, "font_color": [255, 255, 0], "font_thickness": 1,"w":320, "h":240, "row_height": 10, "column":15, "padding": 42}'
        # parse x:
        self.text_settings = json.loads(self.text_overlay_settings)
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)320, height=(int)200,format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink"
        )
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def drawCrosshair(self, frame):
        w = self.text_settings["w"]
        h = self.text_settings["h"]
        size = 10
        v_shift = -20

        cv2.line(
            img=frame,
            pt1=(int(w / 2 - size), int(h / 2 + v_shift)),
            pt2=(int(w / 2 - 2 * size), int(h / 2 + v_shift)),
            color=(255, 255, 0),
            thickness=1,
            lineType=8,
            shift=0,
        )
        cv2.line(
            img=frame,
            pt1=(int(w / 2 + size), int(h / 2 + v_shift)),
            pt2=(int(w / 2 + 2 * size), int(h / 2 + v_shift)),
            color=(255, 255, 0),
            thickness=1,
            lineType=8,
            shift=0,
        )
        cv2.line(
            img=frame,
            pt1=(int(w / 2), int(h / 2) - size + v_shift),
            pt2=(int(w / 2), int(h / 2) - 2 * size + v_shift),
            color=(255, 255, 0),
            thickness=1,
            lineType=8,
            shift=0,
        )
        cv2.line(
            img=frame,
            pt1=(int(w / 2), int(h / 2) + size + v_shift),
            pt2=(int(w / 2), int(h / 2) + 2 * size + v_shift),
            color=(255, 255, 0),
            thickness=1,
            lineType=8,
            shift=0,
        )

    def draw_joy(self, frame, x, y):
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"]
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        w = self.text_settings["w"]
        h = self.text_settings["h"]
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"]
        column = self.text_settings["column"]

        cv2.putText(
            frame,
            str(x),
            (int(w - padding - 18 * column), int(h - padding)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            str(y),
            (int(w - padding - 16 * column), int(h - padding)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def drawOverlay(ret, frame, f):
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"]
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        w = self.text_settings["w"]
        h = self.text_settings["h"]
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"]
        column = self.text_settings["column"]
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

    def draw_power(self, frame, pow):
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"]
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        w = self.text_settings["w"]
        h = self.text_settings["h"]
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"]
        column = self.text_settings["column"]

        cv2.putText(
            frame,
            str(pow),
            (int(w - padding - 4 * column), int(h - padding)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def draw_CPU(self, frame, CPU):
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"]
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        w = self.text_settings["w"]
        h = self.text_settings["h"]
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"]
        column = self.text_settings["column"]

        cv2.putText(
            frame,
            str(CPU),
            (int(w - padding - 4 * column), int(h - padding - row_height)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def draw_FPS(self, frame, FPS):
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"]
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        w = self.text_settings["w"]
        h = self.text_settings["h"]
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"]
        column = self.text_settings["column"]

        cv2.putText(
            frame,
            str(FPS),
            (int(w - padding - 4 * column), int(h - padding - 2 * row_height)),
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
