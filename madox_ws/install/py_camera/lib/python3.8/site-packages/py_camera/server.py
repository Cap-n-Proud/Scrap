import rclpy
from rclpy.node import Node
import threading
from threading import Lock

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

URL_PATH_MJPG = "/camera.mjpg"
URL_PATH_FAVICON = "/favicon.ico"
SLEEP_IN_SEC = 0.050

x = 0
y = 0
display_config = 0
power_info = "N/A"
CPU_info = "N/A"
diff_fps = 1


class CameraHandler(BaseHTTPRequestHandler):
    def __init__(self, request, client_address, server):
        self.document_root = server.get_document_root()
        self.camera = server.get_camera()

        super(CameraHandler, self).__init__(request, client_address, server)

    def do_GET(self):
        if self.path == URL_PATH_MJPG:
            self.send_response(200)

            self.send_header(
                "Content-type", "multipart/x-mixed-replace; boundary=--jpgboundary"
            )
            self.end_headers()
            while self.camera.is_opened():
                global diff_fps
                start_fps = time.time()
                frame = self.camera.get_frame(SLEEP_IN_SEC)
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

        # server.serve_forever()
        r_info = Robot_Info()
        # Setup and start the thread to read serial port
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
