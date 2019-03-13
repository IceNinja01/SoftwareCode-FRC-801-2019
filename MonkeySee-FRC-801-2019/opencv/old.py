# import numpy as np
# import cv2
#
# img = cv2.imread('/home/luke/ula_bb.png', 1)
# cv2.imshow('image', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# from pyimagesearch.shapedetector import ShapeDetector
# import argparse
# import imutils
# import numpy as np
import cv2
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from pycameras.camerastream import CameraStream, Cameras
from networktables import NetworkTablesInstance

team = 801

if __name__ == "__main__":
    ntinst = NetworkTablesInstance.getDefault()
    ntinst.startClientTeam(team)
    cameras = Cameras(team)
    while(True):
        # Capture frame-by-frame
        ret,frame, = cameras.get_streams()[0].getVideoFrame()
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Display the resulting frame
        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
