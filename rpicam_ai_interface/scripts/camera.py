import traitlets
from traitlets.config.configurable import SingletonConfigurable
import atexit
import cv2
import threading
import numpy as np
import pdb
import time


class Camera(SingletonConfigurable):
    
    value = traitlets.Any()
    
    # config
    width = traitlets.Integer(default_value=720).tag(config=True)
    height = traitlets.Integer(default_value=720).tag(config=True)
    fps = traitlets.Integer(default_value=21).tag(config=True)
    capture_width = traitlets.Integer(default_value=720).tag(config=True)
    capture_height = traitlets.Integer(default_value=720).tag(config=True)

    def __init__(self, *args, **kwargs):
        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        super(Camera, self).__init__(*args, **kwargs)
        #pdb.set_trace()
        #self.stop()
        try:
            self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)

            self.start()
        except:
            self.stop()
            raise RuntimeError(
                'Could not initialize camera.  Please see error trace.')

        atexit.register(self.stop)

    def _capture_frames(self):
        cnt = 0
    
    def returnImage(self):
        cnt = 0
        while True:
            re, image1 = self.cap.read()
            time.sleep(1)
            # Stop the program on the ESC key
            if cnt == 1:
                break
            cnt=cnt+1
        return image1

    def _gst_str(self):
        return 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
                self.capture_width, self.capture_height, self.fps, self.width, self.height)
    
    def start(self):
        if not self.cap.isOpened():
            self.cap.open(self._gst_str(), cv2.CAP_GSTREAMER)
        if not hasattr(self, 'thread') or not self.thread.isAlive():
            self.thread = threading.Thread(target=self._capture_frames)
            self.thread.start()

    def stop(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'thread'):
            self.thread.join()
            
    def restart(self):
        self.stop()
        self.start()

def captureImage():
    cam = Camera()
    pic1 = cam.returnImage()
    return pic1

if __name__ == '__main__':
   pic = captureImage()
