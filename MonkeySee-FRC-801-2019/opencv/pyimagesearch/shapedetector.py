import cv2

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        #initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        #triangle
        if len(approx) == 3:
            shape = "triangle"
        #either a square or a rectangle
        elif len(approx) == 4:
            #compute the bounding box of the contour and use that to find the aspect ratio
            (x,y,w,h) = cv2.boundingRect(approx)
            ar = w/float(h)
            #a square will have an aspect ratio close to one, otherwise the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        #pentagon
        elif len(approx) == 5:
            shape = "pentagon"
        #if it is more than these, we assume the shape is a circle
        else:
            shape = "circle"
        return shape
