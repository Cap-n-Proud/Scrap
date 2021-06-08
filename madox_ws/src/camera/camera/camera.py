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
        self.text_overlay_settings = '{ "thickness":0, "font":0, "font_size":0.2, "font_color": [255, 255, 0], "right_col":0.9, "left_col":0.1, "font_thickness": 1,"row_height": 10, "column":15, "padding": 30}'
        # parse x:
        self.text_settings = json.loads(self.text_overlay_settings)
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

    def print_text(self, frame, pos_x, pos_y, text):
        w = self.width
        h = self.height
        right_col = self.text_settings["right_col"]
        left_col = self.text_settings["left_col"]
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"] * w / 320
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"] * w / 320
        column = self.text_settings["column"] * w / 320

        cv2.putText(
            frame,
            str(text),
            (int(pos_x), int(pos_y)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def draw_IMU(self, frame, imu, temp, alt):
        w = self.width
        h = self.height
        right_col = self.text_settings["right_col"]
        left_col = self.text_settings["left_col"]
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"] * w / 320
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"] * w / 320
        column = self.text_settings["column"] * w / 320

        font_space = 40
        cv2.putText(
            frame,
            str(imu[0]),
            (int(left_col), int(padding)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            str(imu[1]),
            (int(left_col + font_space), int(padding)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            str(imu[2]),
            (int(left_col + 2 * font_space), int(padding)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )
        temp = str(temp) + " C"
        cv2.putText(
            frame,
            str(temp),
            (int(left_col), int(padding + row_height)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )
        alt = str(alt) + " m"
        cv2.putText(
            frame,
            str(alt),
            (int(left_col), int(padding + row_height)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def drawCrosshair(self, frame):
        w = self.width
        h = self.height
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
        w = self.width
        h = self.height
        right_col = self.text_settings["right_col"]
        left_col = self.text_settings["left_col"]
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"] * w / 320
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"] * w / 320
        column = self.text_settings["column"] * w / 320

        cv2.putText(
            frame,
            str(x),
            (int(left_col), int(h - padding)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            str(y),
            (int(left_col + 1 * column), int(h - padding)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def drawOverlay(ret, frame, f):
        w = self.width
        h = self.height
        right_col = self.text_settings["right_col"]
        left_col = self.text_settings["left_col"]
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"] * w / 320
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"] * w / 320
        column = self.text_settings["column"] * w / 320
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
        w = self.width
        h = self.height
        right_col = self.text_settings["right_col"]
        left_col = self.text_settings["left_col"]
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"] * w / 320
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"] * w / 320
        column = self.text_settings["column"] * w / 320

        cv2.putText(
            frame,
            str(pow),
            (int(right_col * w), int(h - padding)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def draw_CPU(self, frame, CPU):
        w = self.width
        h = self.height
        right_col = self.text_settings["right_col"]
        left_col = self.text_settings["left_col"]
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"] * w / 320
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"] * w / 320
        column = self.text_settings["column"] * w / 320

        cv2.putText(
            frame,
            str(CPU),
            (int(right_col * w), int(h - padding - row_height)),
            font,
            font_size,
            font_color,
            font_thickness,
            cv2.LINE_AA,
        )

    def draw_FPS(self, frame, FPS):
        w = self.width
        h = self.height
        right_col = self.text_settings["right_col"]
        left_col = self.text_settings["left_col"]
        thickness = self.text_settings["thickness"]
        font = self.text_settings["font"]
        font_size = self.text_settings["font_size"] * w / 320
        font_color = self.text_settings["font_color"]
        font_thickness = int(self.text_settings["font_thickness"])
        padding = self.text_settings["padding"]
        row_height = self.text_settings["row_height"] * w / 320
        column = self.text_settings["column"] * w / 320

        cv2.putText(
            frame,
            str(FPS),
            (int(right_col * w), int(h - padding - 2 * row_height)),
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
