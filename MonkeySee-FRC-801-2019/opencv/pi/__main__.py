#!/usr/bin/env python3
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink, CvSource, VideoMode
import cv2
import numpy as np
from networktables import NetworkTablesInstance
from networktables.util import ntproperty
from pyimagesearch.shapedetector import ShapeDetector
import convenience

class Client(object):
	centerLoc = ntproperty("/SmartDashboard/centerLoc", (0,0))

cl = Client()

if (__name__ == "__main__"):
	team = 801
	ntinst = NetworkTablesInstance.getDefault()
	ntinst.startClientTeam(team)
	usbCamera = UsbCamera("USB Camera 0", 0)
	mjpegServer1 = MjpegServer("serve_USB Camera 0", 1181)
	mjpegServer1.setSource(usbCamera)
	cvSink = CvSink("opencv_USB Camera 0")
	cvSink.setSource(usbCamera)
	outputStream = CvSource("Blur", VideoMode.PixelFormat.kMJPEG, 320, 240, 15)
	mjpegServer2 = MjpegServer("serve_Blur Camera 1", 1182)
	mjpegServer2.setSource(outputStream)
	frame = np.zeros((320,240,3), 'uint8')
	iter = 0
	while(True):
		_, frame = cvSink.grabFrame(frame)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		lower_blue = np.array([110, 50, 50])
		upper_blue = np.array([130, 255, 255])
		mask = cv2.inRange(hsv, lower_blue, upper_blue)
		res = cv2.bitwise_and(frame,frame, mask=mask)

		res_gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		res_blur = cv2.GaussianBlur(res_gray, (5,5), 0)
		res_thresh = cv2.threshold(res_blur, 60, 255, cv2.THRESH_BINARY)[1]
		res_eroded = cv2.erode(res_thresh, np.ones((11,11)))
		res_dilated = cv2.dilate(res_eroded, np.ones((11,11)))
		res_color = cv2.cvtColor(res_dilated, cv2.COLOR_GRAY2BGR)

		resized = convenience.resize(res, width=300)
		ratio = res.shape[0] / float(resized.shape[0])

		gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5,5), 0)
		thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
		eroded = cv2.erode(thresh, np.ones((11,11)))
		dilated = cv2.dilate(eroded, np.ones((11,11)))

		cnts = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = convenience.grab_contours(cnts)
		sd = ShapeDetector()
		i = 0
		for c in cnts:
			M = cv2.moments(c)
			try:
				cX = int((M["m10"] / M["m00"]) * ratio)
				cY = int((M["m01"] / M["m00"]) * ratio)
				if i == 0:
					cl.centerLoc = (cX,cY)
			except ZeroDivisionError:
				print("Tried to divide by zero again.")
				continue
			shape = sd.detect(c)
			# multiply the contour (x, y)-coordinates by the resize ratio,
			# then draw the contours and the name of the shape on the image
			c = c.astype("float")
			c *= ratio
			c = c.astype("int")
			cv2.drawContours(res_color, [c], -1, (0, 255, 0), 2)
			cv2.putText(res_color, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
			i+=1
		outputStream.putFrame(res_color)
		iter+=1
		if iter >= 5:
			print("Alive")
			iter = 0
