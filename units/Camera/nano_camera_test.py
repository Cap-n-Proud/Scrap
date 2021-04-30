"""
Using the NanoCamera with CSI Camera
@author: Ayo Ayibiowu
"""
import cv2

# from nanocamera.NanoCam import Camera
import nanocamera as nano

if __name__ == "__main__":
    # Create the Camera instance
    camera = nano.Camera(flip=0, width=640, height=480, fps=30)
    # For multiple CSI camera
    # camera_2 = nano.Camera(device_id=1, flip=0, width=1280, height=800, fps=30)
    print("CSI Camera is now ready")
    # read the camera image
    frame = camera.read()
    # display the frame
    cv2.imwrite("image.png", frame)

    # close the camera instance
    camera.release()

    # remove camera object
    del camera
