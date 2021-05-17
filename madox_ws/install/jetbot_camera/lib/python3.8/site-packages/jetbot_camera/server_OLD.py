import rclpy
from rclpy.node import Node
import threading
from threading import Lock
import uuid

# import board

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray, Int16
from sensor_msgs.msg import Joy


import argparse
import cv2
import time

from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

from .camera import Camera
from .log import logger

from .debounce import ButtonHandler

URL_PATH_MJPG = "/camera.mjpg"
URL_PATH_FAVICON = "/favicon.ico"
SLEEP_IN_SEC = 0.050

x = 0
y = 0
display_config = 0
power_info = "N/A"
CPU_info = "N/A"
diff_fps = 1
flash_message = ""
take_snapshot = False


class CameraHandler(BaseHTTPRequestHandler):
    def __init__(self, request, client_address, server):
        self.document_root = server.get_document_root()
        self.camera = server.get_camera()

        super(CameraHandler, self).__init__(request, client_address, server)

    def flash_message(self, text, frame, pos_x=int(200), pos_y=int(20), duration=3):
        # self.camera.print_text(frame, pos_x, pos_y, text)
        # parse x:
        thickness = 0
        font = 0
        font_size = 0.3
        font_color = [255, 255, 0]
        font_thickness = 1

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
        self.clear_text(duration)

    def c_text():
        global flash_message
        flash_message = ""

    def clear_text(self, duration=3):
        self.scheduler.add_job(
            self.c_text,
            "interval",
            seconds=int(duration),
            id="clear_text",
            replace_existing=True,
        )

    def save_snapshot(self, im):
        # save snapshot when button is pressed down
        file_path = "snapshots/" + str(uuid.uuid1()) + ".jpg"
        # write snapshot to file (we use image value instead of camera because it's already in JPEG format)
        with open(file_path, "wb") as f:
            f.write(im)

    def do_GET(self):
        if self.path == URL_PATH_MJPG:
            self.send_response(200)

            self.send_header(
                "Content-type", "multipart/x-mixed-replace; boundary=--jpgboundary"
            )
            self.end_headers()
            while self.camera.is_opened():
                global diff_fps, flash_message, take_snapshot
                start_fps = time.time()
                frame = self.camera.get_frame(SLEEP_IN_SEC)
                # Does not work

                if display_config == 0:
                    self.camera.drawCrosshair(frame)
                    self.camera.draw_joy(frame, x, y)
                    self.camera.draw_power(frame, power_info)
                    self.camera.draw_CPU(frame, CPU_info)
                    self.camera.draw_FPS(frame, "FPS: " + str(int(1 / float(diff_fps))))
                    # self.camera.draw_power2(frame, "AAA")

                elif display_config == 1:
                    self.camera.drawCrosshair(frame)
                elif display_config == 2:
                    continue

                ret, jpg = cv2.imencode(".jpg", frame)
                if take_snapshot:
                    self.save_snapshot(jpg)
                    take_snapshot = False

                # jpg = self.camera.read_in_jpeg(SLEEP_IN_SEC, 1 / diff_fps)
                if jpg is None:
                    continue
                self.wfile.write("--jpgboundary".encode())
                self.send_header("Content-type", "image/jpeg")
                self.send_header("Content-length", str(jpg.nbytes))
                self.end_headers()
                self.wfile.write(jpg)
                endtime_fps = time.time()
                diff_fps = endtime_fps - start_fps

        elif self.path == URL_PATH_FAVICON:
            self.send_response(404)
            self.end_headers()
            self.wfile.write("favicon is not found".encode())
        else:
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            with open(self.document_root + "/index.html", "r") as f:

                self.wfile.write(f.read().encode())
        logger.info("thread is stopping ... [{path}]".format(path=self.path))


class Robot_Info(Node):
    def __init__(self):
        super().__init__("robot_info")

        self.joy_topic = self.create_subscription(Joy, "joy", self.joy_topic, 10)
        self.CPU_topic = self.create_subscription(
            String, "info_sys_CPU", self.CPU_topic, 10
        )
        self.power_topic = self.create_subscription(
            String, "info_sys_power", self.power_topic, 10
        )

    def power_topic(self, msg):
        global power_info
        power_info = msg.data

    def CPU_topic(self, msg):
        global CPU_info
        CPU_info = msg.data

    def take_snapshot():
        global flash_message, take_snapshot
        # flash_message = "test"
        take_snapshot = True

    def joy_topic(self, msg):
        # need to debounce the button: https://kaspars.net/blog/micropython-button-debounce

        global x, y, display_config
        x = round(msg.axes[0], 2)
        y = round(msg.axes[1], 2)
        if msg.buttons[9] == 1:
            if display_config >= 3:
                display_config = 0
            else:
                display_config += 1
        # cb = ButtonHandler(msg.buttons[5], take_snapshot, edge="both", bouncetime=600)
        # cb.start()


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    def set_camera(self, camera):
        self.camera = camera

    def get_camera(self):
        return self.camera

    def set_document_root(self, document_root):
        self.document_root = document_root

    def get_document_root(self):
        return self.document_root


def main(args=None):
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", type=str, default="192.168.1.164")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--directory", type=str, default="html")
    parser.add_argument("--device-index", type=int, default=0)
    args = parser.parse_args()

    camera = Camera(args.device_index, args.width, args.height)
    try:
        server = ThreadedHTTPServer((args.bind, args.port), CameraHandler)
        server.set_camera(camera)
        server.set_document_root(args.directory)
        logger.info("server started")

        thread2 = threading.Thread(target=server.serve_forever)
        thread2.start()
        r_info = Robot_Info()

        # server.serve_forever()
        # Setup and start the thread to read serial port    r_info = Robot_Info()

        thread_lock = Lock()
        # thread = threading.Thread(target=rclpy.spin, args=(server))
        thread = threading.Thread(target=rclpy.spin(r_info))
        thread.start()

    except KeyboardInterrupt:
        logger.info("server is stopping ...")
        camera.release()
        server.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    r_info.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
