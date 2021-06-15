import cv2
import json

o_settings = '{ "thickness":0, "font":0, "font_size":0.2, "font_space":25, "font_color": [255, 255, 0], "right_col":0.9, "left_col":0.1, "font_thickness": 1,"row_height": 10, "column":15, "padding": 30}'
overlay_settings = json.loads(o_settings)


def draw_text(frame, pos_x, pos_y, text, width, height):
    w = width
    h = height
    right_col = overlay_settings["right_col"]
    left_col = overlay_settings["left_col"]
    thickness = overlay_settings["thickness"]
    font = overlay_settings["font"]
    font_size = overlay_settings["font_size"] * w / 320
    font_color = overlay_settings["font_color"]
    font_thickness = int(overlay_settings["font_thickness"])
    padding = overlay_settings["padding"]
    row_height = overlay_settings["row_height"] * w / 320
    column = overlay_settings["column"] * w / 320

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


def draw_IMU(frame, imu, temp, alt, width, height):
    w = width
    h = height
    right_col = overlay_settings["right_col"]
    left_col = overlay_settings["left_col"]
    thickness = overlay_settings["thickness"]
    font = overlay_settings["font"]
    font_size = overlay_settings["font_size"] * w / 320
    font_color = overlay_settings["font_color"]
    font_thickness = int(overlay_settings["font_thickness"])
    padding = overlay_settings["padding"]
    row_height = overlay_settings["row_height"] * w / 320
    column = overlay_settings["column"] * w / 320
    font_space = overlay_settings["font_space"]

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
        (int(left_col + 3 * font_space), int(padding)),
        font,
        font_size,
        font_color,
        font_thickness,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        str(imu[2]),
        (int(left_col + 6 * font_space), int(padding)),
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
        (int(left_col + 3 * font_space), int(padding + row_height)),
        font,
        font_size,
        font_color,
        font_thickness,
        cv2.LINE_AA,
    )


def drawCrosshair(frame, width, height):
    w = width
    h = height
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


def draw_joy(frame, x, y, width, height):
    w = width
    h = height
    right_col = overlay_settings["right_col"]
    left_col = overlay_settings["left_col"]
    thickness = overlay_settings["thickness"]
    font = overlay_settings["font"]
    font_size = overlay_settings["font_size"] * w / 320
    font_color = overlay_settings["font_color"]
    font_thickness = int(overlay_settings["font_thickness"])
    padding = overlay_settings["padding"]
    row_height = overlay_settings["row_height"] * w / 320
    column = overlay_settings["column"] * w / 320
    font_space = overlay_settings["font_space"]

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
        (int(left_col + 2 * font_space), int(h - padding)),
        font,
        font_size,
        font_color,
        font_thickness,
        cv2.LINE_AA,
    )


def drawOverlay(ret, frame, f, width, height):
    w = width
    h = height
    right_col = overlay_settings["right_col"]
    left_col = overlay_settings["left_col"]
    thickness = overlay_settings["thickness"]
    font = overlay_settings["font"]
    font_size = overlay_settings["font_size"] * w / 320
    font_color = overlay_settings["font_color"]
    font_thickness = int(overlay_settings["font_thickness"])
    padding = overlay_settings["padding"]
    row_height = overlay_settings["row_height"] * w / 320
    column = overlay_settings["column"] * w / 320
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


def draw_power(frame, pow, width, height):
    w = width
    h = height
    right_col = overlay_settings["right_col"]
    left_col = overlay_settings["left_col"]
    thickness = overlay_settings["thickness"]
    font = overlay_settings["font"]
    font_size = overlay_settings["font_size"] * w / 320
    font_color = overlay_settings["font_color"]
    font_thickness = int(overlay_settings["font_thickness"])
    padding = overlay_settings["padding"]
    row_height = overlay_settings["row_height"] * w / 320
    column = overlay_settings["column"] * w / 320

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


def draw_CPU(frame, CPU, width, height):
    w = width
    h = height
    right_col = overlay_settings["right_col"]
    left_col = overlay_settings["left_col"]
    thickness = overlay_settings["thickness"]
    font = overlay_settings["font"]
    font_size = overlay_settings["font_size"] * w / 320
    font_color = overlay_settings["font_color"]
    font_thickness = int(overlay_settings["font_thickness"])
    padding = overlay_settings["padding"]
    row_height = overlay_settings["row_height"] * w / 320
    column = overlay_settings["column"] * w / 320

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


def draw_FPS(frame, FPS, width, height):
    w = width
    h = height
    right_col = overlay_settings["right_col"]
    left_col = overlay_settings["left_col"]
    thickness = overlay_settings["thickness"]
    font = overlay_settings["font"]
    font_size = overlay_settings["font_size"] * w / 320
    font_color = overlay_settings["font_color"]
    font_thickness = int(overlay_settings["font_thickness"])
    padding = overlay_settings["padding"]
    row_height = overlay_settings["row_height"] * w / 320
    column = overlay_settings["column"] * w / 320

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
