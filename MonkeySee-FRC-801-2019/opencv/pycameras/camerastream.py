import cv2

class CameraStream:
    def __init__(self, camera):
        self.camera = camera
    def getVideoFrame(self):
        return self.camera.read()

class Cameras:
    def __init__(self, team):
        self.cameras = []
        inst = CameraServer.getInstance()
        camera = UsbCamera("/dev/video0")
        server = inst.startAutomaticCapture(camera=camera, return_server=True)
        camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)
        self.cameras.append(server)
        self.camera_streams = []
        for camera in cameras:
            cam = CameraStream(camera)
            self.camera_streams.append(cam)
    def get_streams(self):
        return self.camera_streams
