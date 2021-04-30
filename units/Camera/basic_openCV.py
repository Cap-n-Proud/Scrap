import  cv2

pipeline = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1920, height=1080, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
print(cv2.getBuildInformation())
